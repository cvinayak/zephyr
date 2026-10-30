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

#include "iso_rx_stats.h"

#define TIMEOUT_SYNC_CREATE K_SECONDS(10)
#define NAME_LEN            30

/* Continuous scanning, scan window equal to scan interval, so that scanning uses the radio
 * between subevents of already synchronized BIGs.
 */
#define BT_LE_SCAN_CUSTOM BT_LE_SCAN_PARAM(BT_LE_SCAN_TYPE_ACTIVE, \
					   BT_LE_SCAN_OPT_NONE, \
					   BT_GAP_SCAN_FAST_INTERVAL_MIN, \
					   BT_GAP_SCAN_FAST_INTERVAL_MIN)

#define PA_RETRY_COUNT 6

BUILD_ASSERT(BIS_ISO_CHAN_COUNT > 0U, "Insufficient CONFIG_BT_ISO_MAX_CHAN");

static bool         per_adv_found;
static bool         per_adv_lost;
static bt_addr_le_t per_addr;
static uint8_t      per_sid;
static uint32_t     per_interval_us;

static uint32_t     iso_recv_count;

struct iso_rx_stats iso_rx_stats[ISO_CHAN_COUNT];

static K_SEM_DEFINE(sem_per_adv, 0, 1);
static K_SEM_DEFINE(sem_per_sync, 0, 1);
static K_SEM_DEFINE(sem_per_sync_lost, 0, 1);
static K_SEM_DEFINE(sem_per_big_info, 0, 1);
static K_SEM_DEFINE(sem_big_sync, 0, BIS_ISO_CHAN_COUNT);
static K_SEM_DEFINE(sem_big_sync_lost, 0, BIS_ISO_CHAN_COUNT);

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

	if (!per_adv_found && info->interval && info->sid == per_sid) {
		per_adv_found = true;

		per_interval_us = BT_CONN_INTERVAL_TO_US(info->interval);
		bt_addr_le_copy(&per_addr, info->addr);

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

	per_adv_lost = true;
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

static struct bt_iso_chan bis_iso_chan[ISO_CHAN_COUNT];

static struct iso_rx_stats *iso_rx_stats_get(const struct bt_iso_chan *chan)
{
	return &iso_rx_stats[ARRAY_INDEX(bis_iso_chan, chan)];
}

static void iso_rx_stats_update(struct bt_iso_chan *chan, const struct bt_iso_recv_info *info)
{
	struct iso_rx_stats *stats = iso_rx_stats_get(chan);

	if (stats->seq_num_valid) {
		uint16_t seq_num_expected = stats->seq_num_last + 1U;

		if (info->seq_num != seq_num_expected) {
			stats->seq_gap_cnt += (uint16_t)(info->seq_num - seq_num_expected);
			printk("ISO chan %p seq_num gap: expected %u, got %u\n", chan,
			       seq_num_expected, info->seq_num);
		}
	}

	stats->seq_num_last = info->seq_num;
	stats->seq_num_valid = true;

	if ((info->flags & BT_ISO_FLAGS_VALID) != 0U) {
		stats->rx_cnt++;
	} else {
		stats->lost_cnt++;
		printk("ISO chan %p seq_num %u ts %u SDU not valid, flags 0x%x\n", chan,
		       info->seq_num, info->ts, info->flags);
	}
}

static void iso_recv(struct bt_iso_chan *chan, const struct bt_iso_recv_info *info,
		struct net_buf *buf)
{
	char data_str[128];
	size_t str_len;
	uint32_t count = 0; /* only valid if the data is a counter */

	iso_rx_stats_update(chan, info);

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

	iso_rx_stats_get(chan)->connected = true;
	iso_rx_stats_get(chan)->seq_num_valid = false;

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

	iso_rx_stats_get(chan)->connected = false;

	if (reason != BT_HCI_ERR_OP_CANCELLED_BY_HOST) {
		k_sem_give(&sem_big_sync_lost);
	}
}

static struct bt_iso_chan_ops iso_ops = {
	.recv		= iso_recv,
	.connected	= iso_connected,
	.disconnected	= iso_disconnected,
};

static struct bt_iso_chan_io_qos iso_rx_qos[ISO_CHAN_COUNT];
static struct bt_iso_chan_qos bis_iso_qos[ISO_CHAN_COUNT];
static struct bt_iso_chan *bis[ISO_CHAN_COUNT];
static struct bt_iso_big_sync_param big_sync_param[BIG_SYNC_COUNT];

static void iso_chan_init(void)
{
	for (size_t i = 0U; i < ISO_CHAN_COUNT; i++) {
		bis_iso_qos[i].rx = &iso_rx_qos[i];
		bis_iso_chan[i].ops = &iso_ops;
		bis_iso_chan[i].qos = &bis_iso_qos[i];
		bis[i] = &bis_iso_chan[i];
	}

	for (size_t i = 0U; i < BIG_SYNC_COUNT; i++) {
		big_sync_param[i].bis_channels = &bis[i * BIS_ISO_CHAN_COUNT];
		big_sync_param[i].num_bis = BIS_ISO_CHAN_COUNT;
		big_sync_param[i].bis_bitfield = BIT_MASK(BIS_ISO_CHAN_COUNT);
		big_sync_param[i].mse = BT_ISO_SYNC_MSE_ANY; /* any number of subevents */
		big_sync_param[i].sync_timeout = 100; /* in 10 ms units */
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

int main(void)
{
	struct bt_le_per_adv_sync_param sync_create_param;
	struct bt_le_per_adv_sync *sync;
	struct bt_iso_big *big;
	uint32_t sem_timeout_us;
	int err;

	iso_recv_count = 0;
	iso_chan_init();

	printk("Starting Synchronized Receiver Demo\n");

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

	per_sid = 0U;

	do {
next_per_adv:
		reset_semaphores();
		per_adv_lost = false;

		printk("Start scanning...");
		err = bt_le_scan_start(BT_LE_SCAN_CUSTOM, NULL);
		if (err) {
			printk("failed (err %d)\n", err);
			return 0;
		}
		printk("success.\n");

wait_for_per_adv:
#ifdef CONFIG_ISO_BLINK_LED0
		printk("Start blinking LED...\n");
		led_is_on = false;
		blink = true;
		gpio_pin_set_dt(&led_gpio, (int)led_is_on);
		k_work_reschedule(&blink_work, BLINK_ONOFF);
#endif /* CONFIG_ISO_BLINK_LED0 */

		printk("Waiting for periodic advertising...\n");
		per_adv_found = false;
		err = k_sem_take(&sem_per_adv, K_FOREVER);
		if (err) {
			printk("failed (err %d)\n", err);
			return 0;
		}
		printk("Found periodic advertising.\n");

		printk("Creating Periodic Advertising Sync...");
		bt_addr_le_copy(&sync_create_param.addr, &per_addr);
		sync_create_param.options = 0;
		sync_create_param.sid = per_sid;
		sync_create_param.skip = 0;
		/* Multiple PA interval with retry count and convert to unit of 10 ms */
		sync_create_param.timeout = (per_interval_us * PA_RETRY_COUNT) /
						(10 * USEC_PER_MSEC);
		sem_timeout_us = per_interval_us * PA_RETRY_COUNT;
		err = bt_le_per_adv_sync_create(&sync_create_param, &sync);
		if (err) {
			printk("failed (err %d)\n", err);
			return 0;
		}
		printk("success.\n");

		printk("Waiting for periodic sync...\n");
		err = k_sem_take(&sem_per_sync, K_USEC(sem_timeout_us));
		if (err) {
			printk("failed (err %d)\n", err);

			printk("Deleting Periodic Advertising Sync...");
			err = bt_le_per_adv_sync_delete(sync);
			if (err) {
				printk("failed (err %d)\n", err);
				return 0;
			}

			goto wait_for_per_adv;
		}
		printk("Periodic sync established.\n");

		printk("Stop scanning...");
		err = bt_le_scan_stop();
		if (err) {
			printk("failed (err %d)\n", err);
			return 0;
		}
		printk("success.\n");

		printk("Waiting for BIG info...\n");
		err = k_sem_take(&sem_per_big_info, K_USEC(sem_timeout_us));
		if (err) {
			printk("failed (err %d)\n", err);

			if (per_adv_lost) {
				continue;
			}

			printk("Deleting Periodic Advertising Sync...");
			err = bt_le_per_adv_sync_delete(sync);
			if (err) {
				printk("failed (err %d)\n", err);
				return 0;
			}
			continue;
		}
		printk("Periodic sync established.\n");

big_sync_create:
		printk("Create BIG Sync...\n");
		err = bt_iso_big_sync(sync, &big_sync_param[per_sid], &big);
		if (err) {
			printk("failed (err %d)\n", err);
			return 0;
		}
		printk("success.\n");

		for (uint8_t chan = 0U; chan < BIS_ISO_CHAN_COUNT; chan++) {
			printk("Waiting for BIG sync chan %u...\n", chan);
			err = k_sem_take(&sem_big_sync, TIMEOUT_SYNC_CREATE);
			if (err) {
				break;
			}
			printk("BIG sync chan %u successful.\n", chan);
		}
		if (err) {
			printk("failed (err %d)\n", err);

			printk("BIG Sync Terminate...");
			err = bt_iso_big_terminate(big);
			if (err) {
				printk("failed (err %d)\n", err);
				return 0;
			}
			printk("done.\n");

			goto per_sync_lost_check;
		}
		printk("BIG sync established.\n");

		per_sid++;
		if (per_sid < BIG_SYNC_COUNT) {
			goto next_per_adv;
		}

		return 0;

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

		for (uint8_t chan = 0U; chan < BIS_ISO_CHAN_COUNT; chan++) {
			printk("Waiting for BIG sync lost chan %u...\n", chan);
			err = k_sem_take(&sem_big_sync_lost, K_FOREVER);
			if (err) {
				printk("failed (err %d)\n", err);
				return 0;
			}
			printk("BIG sync lost chan %u.\n", chan);
		}
		printk("BIG sync lost.\n");

per_sync_lost_check:
		printk("Check for periodic sync lost...\n");
		err = k_sem_take(&sem_per_sync_lost, K_NO_WAIT);
		if (err) {
			/* Periodic Sync active, go back to creating BIG Sync */
			goto big_sync_create;
		}
		printk("Periodic sync lost.\n");
	} while (true);
}
