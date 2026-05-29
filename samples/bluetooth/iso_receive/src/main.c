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
static bt_addr_le_t per_addr[SOURCE_COUNT];
static uint8_t      per_sid[SOURCE_COUNT];
static uint32_t     per_interval_us[SOURCE_COUNT];

static struct bt_le_per_adv_sync *pa_syncs[SOURCE_COUNT];
static struct bt_iso_big *bigs[SOURCE_COUNT];

/* Per-source state tracking for sync loss detection */
static volatile bool source_active[SOURCE_COUNT];
static volatile uint8_t channels_disconnected[SOURCE_COUNT];

/* Index of the source currently being set up, or -1 if none */
static int8_t setting_up_source = -1;

static uint32_t     iso_recv_count;

static K_SEM_DEFINE(sem_per_adv, 0, 1);
static K_SEM_DEFINE(sem_per_sync, 0, 1);
static K_SEM_DEFINE(sem_per_big_info, 0, 1);
static K_SEM_DEFINE(sem_big_sync, 0, BIS_ISO_CHAN_COUNT);
static K_SEM_DEFINE(sem_source_lost, 0, SOURCE_COUNT);

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
	for (uint8_t i = 0; i < SOURCE_COUNT; i++) {
		if (!source_active[i]) {
			continue;
		}
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

	if (setting_up_source >= 0 && !per_adv_found &&
	    info->interval &&
	    !is_source_already_found(info->addr, info->sid)) {
		per_adv_found = true;

		per_sid[setting_up_source] = info->sid;
		per_interval_us[setting_up_source] =
			BT_CONN_INTERVAL_TO_US(info->interval);
		bt_addr_le_copy(&per_addr[setting_up_source], info->addr);

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

	for (uint8_t src = 0; src < SOURCE_COUNT; src++) {
		if (pa_syncs[src] == sync) {
			pa_syncs[src] = NULL;
			source_active[src] = false;
			break;
		}
	}
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

	/* Only signal for the source currently being set up to avoid
	 * spurious wakeups from already-synced sources' periodic
	 * advertising.
	 */
	if (setting_up_source >= 0 &&
	    pa_syncs[setting_up_source] == sync) {
		k_sem_give(&sem_per_big_info);
	}
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
		for (uint8_t i = 0; i < TOTAL_BIS_COUNT; i++) {
			if (&bis_iso_chan[i] == chan) {
				uint8_t src = i / BIS_ISO_CHAN_COUNT;

				source_active[src] = false;
				channels_disconnected[src]++;
				if (channels_disconnected[src] >=
				    BIS_ISO_CHAN_COUNT) {
					/* All channels for this source have
					 * disconnected; BIG handle is now
					 * invalid.
					 */
					bigs[src] = NULL;
					k_sem_give(&sem_source_lost);
				}
				break;
			}
		}
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

/**
 * Set up (or re-establish) a single broadcast source.
 *
 * @param src         Source index (0-based).
 * @param full_resync If true, scan for a new periodic advertiser and create
 *                    PA sync before creating BIG sync. If false, reuse the
 *                    existing PA sync and only re-create the BIG sync.
 * @return 0 on success, negative error code on failure.
 */
static int setup_source(uint8_t src, bool full_resync)
{
	struct bt_le_per_adv_sync_param sync_create_param;
	uint32_t sem_timeout_us;
	int err;

	setting_up_source = src;
	source_active[src] = false;
	channels_disconnected[src] = 0;

	if (full_resync) {
		/* Scan for a broadcast source */
		k_sem_reset(&sem_per_adv);

		printk("Start scanning (source %u)...", src + 1);
		err = bt_le_scan_start(BT_LE_SCAN_CUSTOM, NULL);
		if (err) {
			printk("failed (err %d)\n", err);
			setting_up_source = -1;
			return err;
		}
		printk("success.\n");

		printk("Waiting for periodic advertising"
		       " (source %u)...\n", src + 1);
		per_adv_found = false;
		err = k_sem_take(&sem_per_adv, K_FOREVER);
		if (err) {
			printk("failed (err %d)\n", err);
			setting_up_source = -1;
			return err;
		}
		printk("Found periodic advertising from"
		       " source %u.\n", src + 1);

		printk("Stop scanning...");
		err = bt_le_scan_stop();
		if (err) {
			printk("failed (err %d)\n", err);
			setting_up_source = -1;
			return err;
		}
		printk("success.\n");

		/* Create PA sync */
		k_sem_reset(&sem_per_sync);

		printk("Creating Periodic Advertising Sync"
		       " (source %u)...", src + 1);
		bt_addr_le_copy(&sync_create_param.addr, &per_addr[src]);
		sync_create_param.options = 0;
		sync_create_param.sid = per_sid[src];
		sync_create_param.skip = 0;
		sync_create_param.timeout =
			(per_interval_us[src] * PA_RETRY_COUNT) /
			(10 * USEC_PER_MSEC);
		sem_timeout_us = per_interval_us[src] * PA_RETRY_COUNT;

		err = bt_le_per_adv_sync_create(&sync_create_param,
						&pa_syncs[src]);
		if (err) {
			printk("failed (err %d)\n", err);
			setting_up_source = -1;
			return err;
		}
		printk("success.\n");

		printk("Waiting for periodic sync"
		       " (source %u)...\n", src + 1);
		err = k_sem_take(&sem_per_sync, K_USEC(sem_timeout_us));
		if (err) {
			printk("failed (err %d)\n", err);
			bt_le_per_adv_sync_delete(pa_syncs[src]);
			pa_syncs[src] = NULL;
			setting_up_source = -1;
			return err;
		}
		printk("Periodic sync established"
		       " (source %u).\n", src + 1);
	}

	/* Wait for BIG info on this source's PA sync */
	k_sem_reset(&sem_per_big_info);
	sem_timeout_us = per_interval_us[src] * PA_RETRY_COUNT;

	printk("Waiting for BIG info (source %u)...\n", src + 1);
	err = k_sem_take(&sem_per_big_info, K_USEC(sem_timeout_us));
	if (err) {
		printk("failed (err %d)\n", err);
		if (full_resync && pa_syncs[src]) {
			bt_le_per_adv_sync_delete(pa_syncs[src]);
			pa_syncs[src] = NULL;
		}
		setting_up_source = -1;
		return err;
	}
	printk("BIG info received (source %u).\n", src + 1);

	/* Create BIG sync */
	k_sem_reset(&sem_big_sync);

	printk("Create BIG Sync (source %u)...\n", src + 1);
	err = bt_iso_big_sync(pa_syncs[src], &big_sync_param[src],
			      &bigs[src]);
	if (err) {
		printk("failed (err %d)\n", err);
		setting_up_source = -1;
		return err;
	}
	printk("success.\n");

	for (uint8_t chan = 0U; chan < BIS_ISO_CHAN_COUNT; chan++) {
		printk("Waiting for BIG sync chan %u"
		       " (source %u)...\n", chan, src + 1);
		err = k_sem_take(&sem_big_sync, TIMEOUT_SYNC_CREATE);
		if (err) {
			break;
		}
		printk("BIG sync chan %u successful"
		       " (source %u).\n", chan, src + 1);
	}
	if (err) {
		printk("failed (err %d)\n", err);
		bt_iso_big_terminate(bigs[src]);
		bigs[src] = NULL;
		setting_up_source = -1;
		return err;
	}

	printk("BIG sync established (source %u).\n", src + 1);
	source_active[src] = true;
	setting_up_source = -1;
	return 0;
}

int main(void)
{
	int err;

	iso_recv_count = 0;

	printk("Starting Synchronized Receiver Demo\n");
	printk("Configured to synchronize to %u broadcast source(s)\n",
	       SOURCE_COUNT);

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

#ifdef CONFIG_ISO_BLINK_LED0
	printk("Start blinking LED...\n");
	led_is_on = false;
	blink = true;
	gpio_pin_set_dt(&led_gpio, (int)led_is_on);
	k_work_reschedule(&blink_work, BLINK_ONOFF);
#endif /* CONFIG_ISO_BLINK_LED0 */

	/* Initial setup: establish all sources */
	for (uint8_t src = 0; src < SOURCE_COUNT; src++) {
		printk("\n=== Setting up broadcast source %u of %u ===\n",
		       src + 1, SOURCE_COUNT);

		while (true) {
			err = setup_source(src, true);
			if (err == 0) {
				break;
			}
			printk("Setup source %u failed (err %d),"
			       " retrying...\n", src + 1, err);
			k_sleep(K_SECONDS(1));
		}
	}

	printk("\nAll %u broadcast source(s) synchronized.\n",
	       SOURCE_COUNT);

#ifdef CONFIG_ISO_BLINK_LED0
	printk("Stop blinking LED.\n");
	blink = false;
	k_work_cancel_delayable(&blink_work);

	/* Keep LED on */
	led_is_on = true;
	gpio_pin_set_dt(&led_gpio, (int)led_is_on);
#endif /* CONFIG_ISO_BLINK_LED0 */

	/* Monitor for sync loss and re-establish only the lost source.
	 * Other sources continue receiving ISO data undisturbed.
	 */
	while (true) {
		printk("Monitoring for sync loss...\n");
		err = k_sem_take(&sem_source_lost, K_FOREVER);
		if (err) {
			printk("sem_source_lost take failed (err %d)\n", err);
			return 0;
		}

		for (uint8_t src = 0; src < SOURCE_COUNT; src++) {
			bool needs_full;

			if (source_active[src]) {
				continue;
			}

			if (channels_disconnected[src] < BIS_ISO_CHAN_COUNT) {
				/* Not all channels disconnected yet */
				continue;
			}

			/* Determine re-sync type:
			 * - PA sync still active (pa_syncs[src] != NULL):
			 *   only BIG sync was lost, re-create BIG sync.
			 * - PA sync also lost (pa_syncs[src] == NULL):
			 *   full re-establishment needed.
			 */
			needs_full = (pa_syncs[src] == NULL);

			printk("\nSource %u lost (%s),"
			       " re-establishing...\n", src + 1,
			       needs_full ? "PA + BIG" : "BIG only");

			while (true) {
				/* Re-check PA state on each retry, as PA
				 * sync may have been lost during a failed
				 * BIG-only re-sync attempt.
				 */
				needs_full = (pa_syncs[src] == NULL);
				err = setup_source(src, needs_full);
				if (err == 0) {
					break;
				}
				printk("Re-sync source %u failed (err %d),"
				       " retrying...\n", src + 1, err);
				k_sleep(K_SECONDS(1));
			}

			printk("Source %u re-established.\n", src + 1);
		}
	}
}
