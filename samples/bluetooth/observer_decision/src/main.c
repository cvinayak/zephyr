/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Decision-Based Advertising Observer Sample
 *
 * This sample demonstrates extended scanning with decision-based
 * advertising filtering support as specified in Bluetooth Core
 * Specification v6.2.
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/sys/byteorder.h>

#define NAME_LEN 30

static void scan_recv(const struct bt_le_scan_recv_info *info,
		      struct net_buf_simple *buf)
{
	char addr_str[BT_ADDR_LE_STR_LEN];
	char name[NAME_LEN] = {0};
	uint8_t data_len;
	uint8_t data_type;

	bt_addr_le_to_str(info->addr, addr_str, sizeof(addr_str));

	printk("[DEVICE]: %s, RSSI %d, ", addr_str, info->rssi);

	/* Check for extended advertising */
	if (info->adv_type == BT_GAP_ADV_TYPE_EXT_ADV) {
		printk("Extended Advertising ");
	}

	/* Parse advertising data */
	while (buf->len > 1) {
		data_len = net_buf_simple_pull_u8(buf);
		if (data_len == 0) {
			break;
		}

		data_type = net_buf_simple_pull_u8(buf);
		data_len--;

		switch (data_type) {
		case BT_DATA_NAME_COMPLETE:
		case BT_DATA_NAME_SHORTENED:
			if (data_len < sizeof(name)) {
				memcpy(name, buf->data, data_len);
				name[data_len] = '\0';
				printk("Name: %s ", name);
			}
			net_buf_simple_pull(buf, data_len);
			break;

		case BT_DATA_MANUFACTURER_DATA:
			if (data_len >= 2) {
				uint16_t company_id = sys_get_le16(buf->data);
				printk("MFG: 0x%04x ", company_id);

				/* Print manufacturer data */
				if (data_len > 2) {
					printk("Data: ");
					for (int i = 2; i < data_len; i++) {
						printk("%c", buf->data[i]);
					}
					printk(" ");
				}
			}
			net_buf_simple_pull(buf, data_len);
			break;

		default:
			net_buf_simple_pull(buf, data_len);
			break;
		}
	}

	printk("\n");
}

static struct bt_le_scan_cb scan_callbacks = {
	.recv = scan_recv,
};

int main(void)
{
	struct bt_le_scan_param scan_param = {
		.type = BT_LE_SCAN_TYPE_PASSIVE,
		.options = BT_LE_SCAN_OPT_DECISION_BASED,
		.interval = BT_GAP_SCAN_FAST_INTERVAL,
		.window = BT_GAP_SCAN_FAST_WINDOW,
	};
	int err;

	printk("Starting Decision-Based Advertising Observer\n");

	/* Initialize Bluetooth */
	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	printk("Bluetooth initialized\n");

	/* Register scan callbacks */
	bt_le_scan_cb_register(&scan_callbacks);

	/* Start scanning with decision-based filtering */
	printk("Starting scan with decision-based filtering\n");
	err = bt_le_scan_start(&scan_param, NULL);
	if (err) {
		printk("Scan start failed (err %d)\n", err);
		return 0;
	}

	printk("Scanning successfully started\n");
	printk("Waiting for advertising reports...\n");

	/* Keep scanning indefinitely */
	while (1) {
		k_sleep(K_SECONDS(1));
	}

	return 0;
}
