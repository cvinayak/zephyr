/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Decision-Based Advertising Observer Test
 *
 * Test for extended scanning with decision-based advertising filtering.
 * Reuses observer_decision sample code with babblekit integration.
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/sys/byteorder.h>

#include "babblekit/testcase.h"
#include "babblekit/flags.h"

extern enum bst_result_t bst_result;

#define NAME_LEN 30
#define MIN_EXPECTED_REPORTS 3

DEFINE_FLAG_STATIC(flag_scan_started);
DEFINE_FLAG_STATIC(flag_adv_received);

static int adv_report_count;

static void scan_recv(const struct bt_le_scan_recv_info *info,
		      struct net_buf_simple *buf)
{
	char addr_str[BT_ADDR_LE_STR_LEN];
	char name[NAME_LEN] = {0};
	uint8_t data_len;
	uint8_t data_type;
	bool found_decision_data = false;

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

				/* Check for our decision data */
				if (data_len >= 10) {
					/* Check for "DECISION" string */
					if (memcmp(&buf->data[2], "DECISION", 8) == 0) {
						printk("Data: DECISION ");
						found_decision_data = true;
					}
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

	/* Count reports from our decision broadcaster */
	if (found_decision_data) {
		adv_report_count++;
		SET_FLAG(flag_adv_received);
		printk("Decision advertising report received (count: %d)\n", adv_report_count);
	}
}

static struct bt_le_scan_cb scan_callbacks = {
	.recv = scan_recv,
};

static void test_observer_main(void)
{
	struct bt_le_scan_param scan_param = {
		.type = BT_LE_SCAN_TYPE_PASSIVE,
		.options = BT_LE_SCAN_OPT_DECISION_BASED,
		.interval = BT_GAP_SCAN_FAST_INTERVAL,
		.window = BT_GAP_SCAN_FAST_WINDOW,
	};
	int err;

	printk("Decision-Based Advertising Observer Test\n");

	adv_report_count = 0;

	/* Initialize Bluetooth */
	err = bt_enable(NULL);
	if (err) {
		TEST_FAIL("Bluetooth init failed (err %d)\n", err);
		return;
	}

	printk("Bluetooth initialized\n");

	/* Register scan callbacks */
	bt_le_scan_cb_register(&scan_callbacks);

	/* Set decision instructions for filtering */
	static const uint8_t decision_instructions[] = {
		0x01,  /* Filter on device type */
		0x02,  /* Accept if capability matches */
		0x00,  /* No additional criteria */
	};

	printk("Setting decision instructions\n");
	err = bt_le_scan_set_decision_instructions(decision_instructions,
						   sizeof(decision_instructions));
	if (err) {
		TEST_FAIL("Failed to set decision instructions (err %d)\n", err);
		return;
	}

	printk("Decision instructions set (length: %d bytes)\n",
	       sizeof(decision_instructions));

	/* Start scanning with decision-based filtering */
	printk("Starting scan with decision-based filtering\n");
	err = bt_le_scan_start(&scan_param, NULL);
	if (err) {
		TEST_FAIL("Scan start failed (err %d)\n", err);
		return;
	}

	printk("Scanning successfully started\n");
	SET_FLAG(flag_scan_started);

	/* Wait for advertising reports */
	printk("Waiting for advertising reports...\n");

	/* Wait for at least one report or timeout */
	for (int i = 0; i < 8 && !TEST_FLAG(flag_adv_received); i++) {
		k_sleep(K_SECONDS(1));
	}

	if (!TEST_FLAG(flag_adv_received)) {
		TEST_FAIL("No advertising reports received\n");
		return;
	}

	/* Continue scanning for more reports */
	k_sleep(K_SECONDS(3));

	/* Stop scanning */
	err = bt_le_scan_stop();
	if (err) {
		TEST_FAIL("Scan stop failed (err %d)\n", err);
		return;
	}

	printk("Scanning stopped\n");
	printk("Total advertising reports received: %d\n", adv_report_count);

	/* Verify we received multiple reports */
	if (adv_report_count < MIN_EXPECTED_REPORTS) {
		TEST_FAIL("Too few advertising reports (%d < %d)\n", 
			  adv_report_count, MIN_EXPECTED_REPORTS);
		return;
	}

	printk("Test passed\n");
	TEST_PASS("Decision observer test passed");
}

static const struct bst_test_instance test_def[] = {
	{
		.test_id = "decision_observer",
		.test_descr = "Decision-based advertising observer test",
		.test_main_f = test_observer_main,
	},
	BSTEST_END_MARKER,
};

struct bst_test_list *test_decision_observer_install(struct bst_test_list *tests)
{
	return bst_add_tests(tests, test_def);
}

bst_test_install_t test_installers[] = {
	test_decision_observer_install,
	NULL,
};

int main(void)
{
	bst_main();
	return 0;
}
