/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Decision-Based Advertising Broadcaster Test
 *
 * Test for extended advertising with decision-based advertising filtering.
 * Reuses broadcaster_decision sample code with babblekit integration.
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/hci.h>

#include "babblekit/testcase.h"
#include "babblekit/flags.h"

extern enum bst_result_t bst_result;

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

/* Manufacturer ID for sample data */
#define COMPANY_ID 0x05F1 /* Nordic Semiconductor */

DEFINE_FLAG_STATIC(flag_adv_started);

static uint8_t mfg_data[] = {
	0xF1, 0x05,  /* Company ID (little-endian) */
	0x44, 0x45, 0x43, 0x49, 0x53, 0x49, 0x4F, 0x4E  /* "DECISION" */
};

static const struct bt_data ad[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
	BT_DATA(BT_DATA_MANUFACTURER_DATA, mfg_data, sizeof(mfg_data)),
};

static void test_broadcaster_main(void)
{
	struct bt_le_ext_adv *adv;
	struct bt_le_adv_param adv_param = {
		.id = BT_ID_DEFAULT,
		.sid = 0,
		.secondary_max_skip = 0,
		.options = BT_LE_ADV_OPT_EXT_ADV,
		.interval_min = BT_GAP_ADV_FAST_INT_MIN_2,
		.interval_max = BT_GAP_ADV_FAST_INT_MAX_2,
		.peer = NULL,
	};
	int err;

	printk("Decision-Based Advertising Broadcaster Test\n");

	/* Initialize Bluetooth */
	err = bt_enable(NULL);
	if (err) {
		TEST_FAIL("Bluetooth init failed (err %d)\n", err);
		return;
	}

	printk("Bluetooth initialized\n");

	/* Create extended advertising set */
	err = bt_le_ext_adv_create(&adv_param, NULL, &adv);
	if (err) {
		TEST_FAIL("Failed to create advertising set (err %d)\n", err);
		return;
	}

	printk("Extended advertising set created\n");

	/* Set advertising data */
	err = bt_le_ext_adv_set_data(adv, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		TEST_FAIL("Failed to set advertising data (err %d)\n", err);
		return;
	}

	printk("Advertising data set\n");

	/* Start extended advertising */
	err = bt_le_ext_adv_start(adv, BT_LE_EXT_ADV_START_DEFAULT);
	if (err) {
		TEST_FAIL("Failed to start extended advertising (err %d)\n", err);
		return;
	}

	printk("Extended advertising started\n");
	printk("Advertising with decision-based filtering support\n");

	SET_FLAG(flag_adv_started);

	/* Keep advertising for the duration of the test */
	k_sleep(K_SECONDS(5));

	/* Stop advertising */
	err = bt_le_ext_adv_stop(adv);
	if (err) {
		TEST_FAIL("Failed to stop advertising (err %d)\n", err);
		return;
	}

	printk("Advertising stopped\n");

	/* Delete advertising set */
	err = bt_le_ext_adv_delete(adv);
	if (err) {
		TEST_FAIL("Failed to delete advertising set (err %d)\n", err);
		return;
	}

	printk("Test passed\n");
	TEST_PASS("Decision broadcaster test passed");
}

static const struct bst_test_instance test_def[] = {
	{
		.test_id = "decision_broadcaster",
		.test_descr = "Decision-based advertising broadcaster test",
		.test_main_f = test_broadcaster_main,
	},
	BSTEST_END_MARKER,
};

struct bst_test_list *test_decision_broadcaster_install(struct bst_test_list *tests)
{
	return bst_add_tests(tests, test_def);
}

bst_test_install_t test_installers[] = {
	test_decision_broadcaster_install,
	NULL,
};

int main(void)
{
	bst_main();
	return 0;
}
