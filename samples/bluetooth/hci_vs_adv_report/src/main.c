/* main.c - Application main entry point */

/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/byteorder.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/hci_vs.h>

BUILD_ASSERT(IS_ENABLED(CONFIG_BT_HAS_HCI_VS),
	     "This app requires Zephyr-specific HCI vendor extensions");

static void enable_vs_adv_reports(bool enable)
{
	struct bt_hci_cp_vs_set_adv_reports *cp;
	struct net_buf *buf;
	int err;

	buf = bt_hci_cmd_alloc(K_FOREVER);
	if (!buf) {
		printk("%s: Unable to allocate HCI command buffer\n",
		       __func__);
		return;
	}

	cp = net_buf_add(buf, sizeof(*cp));
	cp->enable = (uint8_t)enable;

	err = bt_hci_cmd_send(BT_HCI_OP_VS_SET_ADV_REPORTS, buf);
	if (err) {
		printk("Set VS adv reports err: %d\n", err);
	}
}

static bool vs_adv_report_cb(struct net_buf_simple *buf)
{
	struct bt_hci_evt_vs_le_adv_report *sep;
	struct bt_hci_evt_vs_le_adv_info *adv_info;
	struct bt_hci_evt_vs *vs;
	int8_t rssi;
	uint8_t chan_idx;

	vs = net_buf_simple_pull_mem(buf, sizeof(*vs));
	if (vs->subevent != BT_HCI_EVT_VS_LE_ADV_REPORT) {
		return false;
	}

	sep = net_buf_simple_pull_mem(buf, sizeof(*sep));
	if (sep->num_reports == 0U) {
		return true;
	}

	adv_info = (struct bt_hci_evt_vs_le_adv_info *)buf->data;

	rssi = *((int8_t *)(adv_info->data + adv_info->length));
	chan_idx = *((uint8_t *)(adv_info->data + adv_info->length + 1));

	printk("VS adv report: peer %s rssi %d chan %u\n",
	       bt_addr_le_str(&adv_info->addr), rssi, chan_idx);

	return true;
}

static void bt_ready(int err)
{
	struct bt_le_scan_param scan_param = {
		.type     = BT_HCI_LE_SCAN_PASSIVE,
		.options  = BT_LE_SCAN_OPT_NONE,
		.interval = BT_GAP_SCAN_FAST_INTERVAL,
		.window   = BT_GAP_SCAN_FAST_WINDOW,
	};

	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	printk("Bluetooth initialized\n");

	err = bt_hci_register_vnd_evt_cb(vs_adv_report_cb);
	if (err) {
		printk("VS event callback register err %d\n", err);
		return;
	}

	enable_vs_adv_reports(true);

	err = bt_le_scan_start(&scan_param, NULL);
	if (err) {
		printk("Scanning failed to start (err %d)\n", err);
		return;
	}

	printk("Scanning started\n");
}

int main(void)
{
	int err;

	printk("Starting VS Advertising Report sample\n");

	err = bt_enable(bt_ready);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
	}

	return 0;
}
