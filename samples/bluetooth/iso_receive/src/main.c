/*
 * Copyright (c) 2021-2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci_types.h>
#include <zephyr/bluetooth/iso.h>
#include <zephyr/sys/byteorder.h>

#define TIMEOUT_SYNC_CREATE K_SECONDS(10)
#define NAME_LEN            30

#define BT_LE_SCAN_CUSTOM BT_LE_SCAN_PARAM(BT_LE_SCAN_TYPE_ACTIVE, \
					   BT_LE_SCAN_OPT_NONE, \
					   BT_GAP_SCAN_FAST_INTERVAL, \
					   BT_GAP_SCAN_FAST_WINDOW)

#define PA_RETRY_COUNT 6

#define BIS_ISO_CHAN_COUNT 2
#define SOURCE_COUNT       CONFIG_ISO_BROADCAST_SOURCE_COUNT
#define TOTAL_BIS_COUNT    (BIS_ISO_CHAN_COUNT * SOURCE_COUNT)

BUILD_ASSERT(TOTAL_BIS_COUNT <= CONFIG_BT_ISO_MAX_CHAN,
	     "CONFIG_BT_ISO_MAX_CHAN must be >= 2 * CONFIG_ISO_BROADCAST_SOURCE_COUNT");

static bool         per_adv_found;
static uint8_t      sources_found;
static bt_addr_le_t per_addr[SOURCE_COUNT];
static uint8_t      per_sid[SOURCE_COUNT];
static uint32_t     per_interval_us[SOURCE_COUNT];

static struct bt_le_per_adv_sync *pa_syncs[SOURCE_COUNT];
static struct bt_iso_big *bigs[SOURCE_COUNT];

static uint32_t     iso_recv_count;

static K_SEM_DEFINE(sem_per_adv, 0, 1);
static K_SEM_DEFINE(sem_per_sync, 0, 1);
static K_SEM_DEFINE(sem_per_sync_lost, 0, SOURCE_COUNT);
static K_SEM_DEFINE(sem_per_big_info, 0, 1);
static K_SEM_DEFINE(sem_big_sync, 0, BIS_ISO_CHAN_COUNT);
static K_SEM_DEFINE(sem_big_sync_lost, 0, TOTAL_BIS_COUNT);

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

#ifdef CONFIG_ISO_BLINK_LED0
static const struct gpio_dt_spec led_gpio = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
#define BLINK_ONOFF K_MSEC(500)

static struct k_work_delayable blink_work;
static bool                    led_is_on;
static bool                    blink;

static void blink_timeout(struct k_work *work)
{
	if (!blink) {
		return;
	}

	led_is_on = !led_is_on;
	gpio_pin_set_dt(&led_gpio, (int)led_is_on);

	k_work_schedule(&blink_work, BLINK_ONOFF);
}
#endif

static bool data_cb(struct bt_data *data, void *user_data)
{
	char *name = user_data;
	uint8_t len;

	switch (data->type) {
	case BT_DATA_NAME_SHORTENED:
	case BT_DATA_NAME_COMPLETE:
		len = MIN(data->data_len, NAME_LEN - 1);
		memcpy(name, data->data, len);
		name[len] = '\0';
		return false;
	default:
		return true;
	}
}

static const char *phy2str(uint8_t phy)
{
	switch (phy) {
	case 0: return "No packets";
	case BT_GAP_LE_PHY_1M: return "LE 1M";
	case BT_GAP_LE_PHY_2M: return "LE 2M";
	case BT_GAP_LE_PHY_CODED: return "LE Coded";
	default: return "Unknown";
	}
}

static bool is_source_already_found(const bt_addr_le_t *addr, uint8_t sid)
{
	for (uint8_t i = 0; i < sources_found; i++) {
		if (bt_addr_le_eq(&per_addr[i], addr) && per_sid[i] == sid) {
			return true;
		}
	}

	return false;
}

static void scan_recv(const struct bt_le_scan_recv_info *info,
		      struct net_buf_simple *buf)
{
	char name[NAME_LEN];

	(void)memset(name, 0, sizeof(name));

	bt_data_parse(buf, data_cb, name);

	printk("[DEVICE]: %s, AD evt type %u, Tx Pwr: %i, RSSI %i %s "
	       "C:%u S:%u D:%u SR:%u E:%u Prim: %s, Secn: %s, "
	       "Interval: 0x%04x (%u us), SID: %u\n",
	       bt_addr_le_str(info->addr), info->adv_type, info->tx_power, info->rssi, name,
	       (info->adv_props & BT_GAP_ADV_PROP_CONNECTABLE) != 0,
	       (info->adv_props & BT_GAP_ADV_PROP_SCANNABLE) != 0,
	       (info->adv_props & BT_GAP_ADV_PROP_DIRECTED) != 0,
	       (info->adv_props & BT_GAP_ADV_PROP_SCAN_RESPONSE) != 0,
	       (info->adv_props & BT_GAP_ADV_PROP_EXT_ADV) != 0,
	       phy2str(info->primary_phy), phy2str(info->secondary_phy),
	       info->interval, BT_CONN_INTERVAL_TO_US(info->interval), info->sid);

	if (!per_adv_found && info->interval &&
	    !is_source_already_found(info->addr, info->sid)) {
		per_adv_found = true;

		per_sid[sources_found] = info->sid;
		per_interval_us[sources_found] = BT_CONN_INTERVAL_TO_US(info->interval);
		bt_addr_le_copy(&per_addr[sources_found], info->addr);

		k_sem_give(&sem_per_adv);
	}
}

static struct bt_le_scan_cb scan_callbacks = {
	.recv = scan_recv,
};

static void sync_cb(struct bt_le_per_adv_sync *sync,
		    struct bt_le_per_adv_sync_synced_info *info)
{
	printk("PER_ADV_SYNC[%u]: [DEVICE]: %s synced, "
	       "Interval 0x%04x (%u ms), PHY %s\n",
	       bt_le_per_adv_sync_get_index(sync), bt_addr_le_str(info->addr),
	       info->interval, info->interval * 5 / 4, phy2str(info->phy));

	k_sem_give(&sem_per_sync);
}

static void term_cb(struct bt_le_per_adv_sync *sync,
		    const struct bt_le_per_adv_sync_term_info *info)
{
	printk("PER_ADV_SYNC[%u]: [DEVICE]: %s sync terminated\n",
	       bt_le_per_adv_sync_get_index(sync), bt_addr_le_str(info->addr));

	k_sem_give(&sem_per_sync_lost);
}

static void recv_cb(struct bt_le_per_adv_sync *sync,
		    const struct bt_le_per_adv_sync_recv_info *info,
		    struct net_buf_simple *buf)
{
	char data_str[129];

	bin2hex(buf->data, buf->len, data_str, sizeof(data_str));

	printk("PER_ADV_SYNC[%u]: [DEVICE]: %s, tx_power %i, "
	       "RSSI %i, CTE %u, data length %u, data: %s\n",
	       bt_le_per_adv_sync_get_index(sync), bt_addr_le_str(info->addr), info->tx_power,
	       info->rssi, info->cte_type, buf->len, data_str);
}

static void biginfo_cb(struct bt_le_per_adv_sync *sync,
		       const struct bt_iso_biginfo *biginfo)
{
	printk("BIG INFO[%u]: [DEVICE]: %s, sid 0x%02x, "
	       "num_bis %u, nse %u, interval 0x%04x (%u ms), "
	       "bn %u, pto %u, irc %u, max_pdu %u, "
	       "sdu_interval %u us, max_sdu %u, phy %s, "
	       "%s framing, %sencrypted\n",
	       bt_le_per_adv_sync_get_index(sync), bt_addr_le_str(biginfo->addr), biginfo->sid,
	       biginfo->num_bis, biginfo->sub_evt_count,
	       biginfo->iso_interval,
	       (biginfo->iso_interval * 5 / 4),
	       biginfo->burst_number, biginfo->offset,
	       biginfo->rep_count, biginfo->max_pdu, biginfo->sdu_interval,
	       biginfo->max_sdu, phy2str(biginfo->phy),
	       biginfo->framing ? "with" : "without",
	       biginfo->encryption ? "" : "not ");


	k_sem_give(&sem_per_big_info);
}

static struct bt_le_per_adv_sync_cb sync_callbacks = {
	.synced = sync_cb,
	.term = term_cb,
	.recv = recv_cb,
	.biginfo = biginfo_cb,
};

static void iso_recv(struct bt_iso_chan *chan, const struct bt_iso_recv_info *info,
		struct net_buf *buf)
{
	char data_str[128];
	size_t str_len;
	uint32_t count = 0; /* only valid if the data is a counter */

	if (buf->len == sizeof(count)) {
		count = sys_get_le32(buf->data);
		if (IS_ENABLED(CONFIG_ISO_ALIGN_PRINT_INTERVALS)) {
			iso_recv_count = count;
		}
	}

	if ((iso_recv_count % CONFIG_ISO_PRINT_INTERVAL) == 0) {
		str_len = bin2hex(buf->data, buf->len, data_str, sizeof(data_str));
		printk("Incoming data channel %p flags 0x%x seq_num %u ts %u len %u: "
		       "%s (counter value %u)\n", chan, info->flags, info->seq_num,
		       info->ts, buf->len, data_str, count);
	}

	iso_recv_count++;
}

static void iso_connected(struct bt_iso_chan *chan)
{
	const struct bt_iso_chan_path hci_path = {
		.pid = BT_ISO_DATA_PATH_HCI,
		.format = BT_HCI_CODING_FORMAT_TRANSPARENT,
	};
	int err;

	printk("ISO Channel %p connected\n", chan);

	err = bt_iso_setup_data_path(chan, BT_HCI_DATAPATH_DIR_CTLR_TO_HOST, &hci_path);
	if (err != 0) {
		printk("Failed to setup ISO RX data path: %d\n", err);
	}

	k_sem_give(&sem_big_sync);
}

static void iso_disconnected(struct bt_iso_chan *chan, uint8_t reason)
{
	printk("ISO Channel %p disconnected with reason 0x%02x\n",
	       chan, reason);

	if (reason != BT_HCI_ERR_OP_CANCELLED_BY_HOST) {
		k_sem_give(&sem_big_sync_lost);
	}
}

static struct bt_iso_chan_ops iso_ops = {
	.recv		= iso_recv,
	.connected	= iso_connected,
	.disconnected	= iso_disconnected,
};

static struct bt_iso_chan_io_qos iso_rx_qos[TOTAL_BIS_COUNT];
static struct bt_iso_chan_qos bis_iso_qos[TOTAL_BIS_COUNT];
static struct bt_iso_chan bis_iso_chan[TOTAL_BIS_COUNT];
static struct bt_iso_chan *bis[TOTAL_BIS_COUNT];

static struct bt_iso_big_sync_param big_sync_param[SOURCE_COUNT];

static void init_channels(void)
{
	for (uint8_t i = 0; i < TOTAL_BIS_COUNT; i++) {
		bis_iso_qos[i].rx = &iso_rx_qos[i];
		bis_iso_chan[i].ops = &iso_ops;
		bis_iso_chan[i].qos = &bis_iso_qos[i];
		bis[i] = &bis_iso_chan[i];
	}

	for (uint8_t src = 0; src < SOURCE_COUNT; src++) {
		big_sync_param[src].bis_channels = &bis[src * BIS_ISO_CHAN_COUNT];
		big_sync_param[src].num_bis = BIS_ISO_CHAN_COUNT;
		big_sync_param[src].bis_bitfield = BIT_MASK(BIS_ISO_CHAN_COUNT);
		big_sync_param[src].mse = BT_ISO_SYNC_MSE_ANY;
		big_sync_param[src].sync_timeout = 100; /* in 10 ms units */
	}
}

static void reset_semaphores(void)
{
	k_sem_reset(&sem_per_adv);
	k_sem_reset(&sem_per_sync);
	k_sem_reset(&sem_per_sync_lost);
	k_sem_reset(&sem_per_big_info);
	k_sem_reset(&sem_big_sync);
	k_sem_reset(&sem_big_sync_lost);
}

static void cleanup_syncs(uint8_t num_sources)
{
	for (uint8_t src = 0; src < num_sources; src++) {
		if (bigs[src]) {
			printk("BIG Sync Terminate source %u...", src);
			bt_iso_big_terminate(bigs[src]);
			printk("done.\n");
			bigs[src] = NULL;
		}
		if (pa_syncs[src]) {
			printk("Deleting Periodic Advertising Sync source %u...", src);
			bt_le_per_adv_sync_delete(pa_syncs[src]);
			printk("done.\n");
			pa_syncs[src] = NULL;
		}
	}
}

int main(void)
{
	struct bt_le_per_adv_sync_param sync_create_param;
	uint32_t sem_timeout_us;
	int err;

	iso_recv_count = 0;

	printk("Starting Synchronized Receiver Demo\n");
	printk("Configured to synchronize to %u broadcast source(s)\n", SOURCE_COUNT);

	init_channels();

#ifdef CONFIG_ISO_BLINK_LED0
	printk("Get reference to LED device...");

	if (!gpio_is_ready_dt(&led_gpio)) {
		printk("LED gpio device not ready.\n");
		return 0;
	}
	printk("done.\n");

	printk("Configure GPIO pin...");
	err = gpio_pin_configure_dt(&led_gpio, GPIO_OUTPUT_ACTIVE);
	if (err) {
		return 0;
	}
	printk("done.\n");

	k_work_init_delayable(&blink_work, blink_timeout);
#endif /* CONFIG_ISO_BLINK_LED0 */

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	printk("Scan callbacks register...");
	bt_le_scan_cb_register(&scan_callbacks);
	printk("success.\n");

	printk("Periodic Advertising callbacks register...");
	bt_le_per_adv_sync_cb_register(&sync_callbacks);
	printk("Success.\n");

	do {
		reset_semaphores();
		sources_found = 0;
		memset(pa_syncs, 0, sizeof(pa_syncs));
		memset(bigs, 0, sizeof(bigs));

#ifdef CONFIG_ISO_BLINK_LED0
		printk("Start blinking LED...\n");
		led_is_on = false;
		blink = true;
		gpio_pin_set_dt(&led_gpio, (int)led_is_on);
		k_work_reschedule(&blink_work, BLINK_ONOFF);
#endif /* CONFIG_ISO_BLINK_LED0 */

		/* Find and synchronize to each broadcast source */
		for (uint8_t src = 0; src < SOURCE_COUNT; src++) {
			printk("\n=== Setting up broadcast source %u of %u ===\n",
			       src + 1, SOURCE_COUNT);

			k_sem_reset(&sem_per_adv);
			k_sem_reset(&sem_per_sync);
			k_sem_reset(&sem_per_big_info);
			k_sem_reset(&sem_big_sync);

			printk("Start scanning...");
			err = bt_le_scan_start(BT_LE_SCAN_CUSTOM, NULL);
			if (err) {
				printk("failed (err %d)\n", err);
				return 0;
			}
			printk("success.\n");

			printk("Waiting for periodic advertising"
			       " (source %u)...\n", src + 1);
			per_adv_found = false;
			err = k_sem_take(&sem_per_adv, K_FOREVER);
			if (err) {
				printk("failed (err %d)\n", err);
				return 0;
			}
			printk("Found periodic advertising from source %u.\n",
			       src + 1);

			printk("Stop scanning...");
			err = bt_le_scan_stop();
			if (err) {
				printk("failed (err %d)\n", err);
				return 0;
			}
			printk("success.\n");

			printk("Creating Periodic Advertising Sync"
			       " (source %u)...", src + 1);
			bt_addr_le_copy(&sync_create_param.addr,
					&per_addr[sources_found]);
			sync_create_param.options = 0;
			sync_create_param.sid = per_sid[sources_found];
			sync_create_param.skip = 0;
			/* Multiple PA interval with retry count and
			 * convert to unit of 10 ms
			 */
			sync_create_param.timeout =
				(per_interval_us[sources_found] *
				 PA_RETRY_COUNT) / (10 * USEC_PER_MSEC);
			sem_timeout_us = per_interval_us[sources_found] *
					 PA_RETRY_COUNT;
			err = bt_le_per_adv_sync_create(&sync_create_param,
							&pa_syncs[src]);
			if (err) {
				printk("failed (err %d)\n", err);
				cleanup_syncs(src);
				break;
			}
			printk("success.\n");

			printk("Waiting for periodic sync"
			       " (source %u)...\n", src + 1);
			err = k_sem_take(&sem_per_sync,
					 K_USEC(sem_timeout_us));
			if (err) {
				printk("failed (err %d)\n", err);

				printk("Deleting Periodic Advertising Sync"
				       " source %u...", src + 1);
				bt_le_per_adv_sync_delete(pa_syncs[src]);
				pa_syncs[src] = NULL;
				printk("done.\n");
				cleanup_syncs(src);
				break;
			}
			printk("Periodic sync established"
			       " (source %u).\n", src + 1);

			printk("Waiting for BIG info"
			       " (source %u)...\n", src + 1);
			err = k_sem_take(&sem_per_big_info,
					 K_USEC(sem_timeout_us));
			if (err) {
				printk("failed (err %d)\n", err);
				cleanup_syncs(src + 1);
				break;
			}
			printk("BIG info received (source %u).\n", src + 1);

			printk("Create BIG Sync (source %u)...\n", src + 1);
			err = bt_iso_big_sync(pa_syncs[src],
					      &big_sync_param[src],
					      &bigs[src]);
			if (err) {
				printk("failed (err %d)\n", err);
				cleanup_syncs(src + 1);
				break;
			}
			printk("success.\n");

			for (uint8_t chan = 0U; chan < BIS_ISO_CHAN_COUNT;
			     chan++) {
				printk("Waiting for BIG sync chan %u"
				       " (source %u)...\n", chan, src + 1);
				err = k_sem_take(&sem_big_sync,
						 TIMEOUT_SYNC_CREATE);
				if (err) {
					break;
				}
				printk("BIG sync chan %u successful"
				       " (source %u).\n", chan, src + 1);
			}
			if (err) {
				printk("failed (err %d)\n", err);
				printk("BIG Sync Terminate source %u...",
				       src + 1);
				bt_iso_big_terminate(bigs[src]);
				bigs[src] = NULL;
				printk("done.\n");
				cleanup_syncs(src + 1);
				break;
			}
			printk("BIG sync established (source %u).\n",
			       src + 1);

			sources_found++;
		}

		if (err) {
			/* Setup of one or more sources failed, retry */
			printk("Source setup failed, retrying...\n");
			continue;
		}

		printk("\nAll %u broadcast source(s) synchronized.\n",
		       SOURCE_COUNT);

#ifdef CONFIG_ISO_BLINK_LED0
		printk("Stop blinking LED.\n");
		blink = false;
		/* If this fails, we'll exit early in the handler because blink
		 * is false.
		 */
		k_work_cancel_delayable(&blink_work);

		/* Keep LED on */
		led_is_on = true;
		gpio_pin_set_dt(&led_gpio, (int)led_is_on);
#endif /* CONFIG_ISO_BLINK_LED0 */

		/* Wait for any BIG sync channel loss */
		printk("Waiting for BIG sync lost...\n");
		err = k_sem_take(&sem_big_sync_lost, K_FOREVER);
		if (err) {
			printk("failed (err %d)\n", err);
			return 0;
		}
		printk("BIG sync lost detected.\n");

		/* Tear down all syncs and restart */
		printk("Cleaning up all syncs...\n");
		cleanup_syncs(SOURCE_COUNT);
		printk("All syncs cleaned up.\n");
	} while (true);
}
