/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>

#include "subsys/bluetooth/controller/ll_sw/ull_cs_internal.h"

ZTEST_SUITE(test_cs_capabilities, NULL, NULL, NULL, NULL, NULL);

ZTEST(test_cs_capabilities, test_read_local_capabilities)
{
	struct bt_hci_rp_le_read_local_supported_capabilities rp;
	uint8_t status;

	status = ll_cs_read_local_supported_capabilities(&rp);

	zassert_equal(status, BT_HCI_ERR_SUCCESS, "Expected success");
	zassert_true(rp.num_config_supported > 0,
		     "Expected non-zero num_config_supported");
}

ZTEST_SUITE(test_cs_config, NULL, NULL, NULL, NULL, NULL);

ZTEST(test_cs_config, test_create_config)
{
	struct bt_hci_cp_le_cs_create_config cmd = {
		.handle = 0,
		.config_id = 0,
		.create_context = 0,
		.main_mode_type = BT_HCI_OP_LE_CS_MAIN_MODE_1,
		.sub_mode_type = BT_HCI_OP_LE_CS_SUB_MODE_UNUSED,
		.min_main_mode_steps = 2,
		.max_main_mode_steps = 10,
		.main_mode_repetition = 0,
		.mode_0_steps = 0,
		.role = BT_HCI_OP_LE_CS_INITIATOR_ROLE,
		.rtt_type = BT_HCI_OP_LE_CS_RTT_TYPE_AA_ONLY,
		.cs_sync_phy = BT_HCI_OP_LE_CS_CS_SYNC_1M,
		.channel_map = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
				0xFF, 0xFF, 0xFF, 0xFF, 0x1F},
		.channel_map_repetition = 1,
		.channel_selection_type = BT_HCI_OP_LE_CS_TEST_CHSEL_TYPE_3B,
		.ch3c_shape = BT_HCI_OP_LE_CS_TEST_CH3C_SHAPE_HAT,
		.ch3c_jump = 2,
	};
	uint8_t config_id;
	uint8_t status;

	status = ll_cs_create_config(&cmd, &config_id);

	zassert_not_equal(status, BT_HCI_ERR_SUCCESS,
			  "Expected error without connection");
}

ZTEST_SUITE(test_cs_settings, NULL, NULL, NULL, NULL, NULL);

ZTEST(test_cs_settings, test_set_channel_classification)
{
	uint8_t channel_map[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
				    0xFF, 0xFF, 0xFF, 0xFF, 0x1F};
	uint8_t status;

	status = ll_cs_set_channel_classification(channel_map);

	zassert_equal(status, BT_HCI_ERR_SUCCESS, "Expected success");
}
