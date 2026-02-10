/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Decision-Based Advertising Broadcaster Sample
 *
 * This sample demonstrates extended advertising with decision-based
 * advertising filtering support as specified in Bluetooth Core
 * Specification v6.2.
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/hci.h>

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

/* Manufacturer ID for sample data */
#define COMPANY_ID 0x05F1 /* Nordic Semiconductor */

static uint8_t mfg_data[] = {
	0xF1, 0x05,  /* Company ID (little-endian) */
	0x44, 0x45, 0x43, 0x49, 0x53, 0x49, 0x4F, 0x4E  /* "DECISION" */
};

static const struct bt_data ad[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
	BT_DATA(BT_DATA_MANUFACTURER_DATA, mfg_data, sizeof(mfg_data)),
};

int main(void)
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

	printk("Starting Decision-Based Advertising Broadcaster\n");

	/* Initialize Bluetooth */
	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	printk("Bluetooth initialized\n");

	/* Create extended advertising set */
	err = bt_le_ext_adv_create(&adv_param, NULL, &adv);
	if (err) {
		printk("Failed to create advertising set (err %d)\n", err);
		return 0;
	}

	printk("Extended advertising set created\n");

	/* Set advertising data */
	err = bt_le_ext_adv_set_data(adv, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		printk("Failed to set advertising data (err %d)\n", err);
		return 0;
	}

	printk("Advertising data set\n");

#if defined(CONFIG_BT_CTLR_DECISION_BASED_FILTERING)
	/* Set decision data for decision-based filtering */
	static const uint8_t decision_data[] = {
		0x01,  /* Device type: sensor */
		0x02,  /* Capability: temperature measurement */
		0x05,  /* Battery level indicator */
		0xFF   /* Custom application data */
	};

	err = bt_le_ext_adv_set_decision_data(adv, decision_data, sizeof(decision_data));
	if (err) {
		printk("Failed to set decision data (err %d)\n", err);
		return 0;
	}

	printk("Decision data set (length: %d bytes)\n", sizeof(decision_data));
#endif /* CONFIG_BT_CTLR_DECISION_BASED_FILTERING */

	/* Start extended advertising */
	err = bt_le_ext_adv_start(adv, BT_LE_EXT_ADV_START_DEFAULT);
	if (err) {
		printk("Failed to start extended advertising (err %d)\n", err);
		return 0;
	}

	printk("Extended advertising started\n");
	printk("Advertising with decision-based filtering support\n");
	printk("Device name: %s\n", DEVICE_NAME);
	printk("Manufacturer data: DECISION\n");

	/* Keep advertising indefinitely */
	while (1) {
		k_sleep(K_SECONDS(1));
	}

	return 0;
}
