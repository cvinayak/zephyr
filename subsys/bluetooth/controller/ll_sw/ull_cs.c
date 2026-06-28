/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Stub implementation of Channel Sounding ULL layer.
 * All ll_cs_* functions return appropriate stub values so that the HCI layer
 * compiles and links. The real implementation replaces this file in a
 * subsequent commit.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/bluetooth/hci_types.h>

#include "hal/ccm.h"
#include "hal/ticker.h"

#include "util/util.h"
#include "util/memq.h"
#include "util/mayfly.h"

#include "ticker/ticker.h"

#include "pdu_df.h"
#include "lll/pdu_vendor.h"
#include "pdu.h"

#include "lll.h"
#include "lll_conn.h"
#include "lll_conn_iso.h"

#include "ull_tx_queue.h"

#include "isoal.h"

#include "ull_iso_types.h"
#include "ull_cs_types.h"
#include "ull_conn_types.h"
#include "ull_conn_iso_types.h"

#include "ull_internal.h"
#include "ull_conn_internal.h"
#include "ull_iso_internal.h"
#include "ull_conn_iso_internal.h"
#include "ull_cs_internal.h"

#include "lll_cs.h"

#include "hal/debug.h"

int ull_cs_init(void)
{
	return 0;
}

int ull_cs_reset(void)
{
	return 0;
}

uint8_t ll_cs_read_local_supported_capabilities(
	struct bt_hci_rp_le_read_local_supported_capabilities *rp)
{
	ARG_UNUSED(rp);

	return BT_HCI_ERR_CMD_DISALLOWED;
}

uint8_t ll_cs_read_remote_supported_capabilities(uint16_t handle)
{
	ARG_UNUSED(handle);

	return BT_HCI_ERR_CMD_DISALLOWED;
}

uint8_t ll_cs_write_cached_remote_supported_capabilities(
	const struct bt_hci_cp_le_write_cached_remote_supported_capabilities *cmd)
{
	ARG_UNUSED(cmd);

	return BT_HCI_ERR_CMD_DISALLOWED;
}

uint8_t ll_cs_security_enable(uint16_t handle)
{
	ARG_UNUSED(handle);

	return BT_HCI_ERR_CMD_DISALLOWED;
}

uint8_t ll_cs_set_default_settings(
	const struct bt_hci_cp_le_cs_set_default_settings *cmd)
{
	ARG_UNUSED(cmd);

	return BT_HCI_ERR_CMD_DISALLOWED;
}

uint8_t ll_cs_read_remote_fae_table(uint16_t handle)
{
	ARG_UNUSED(handle);

	return BT_HCI_ERR_CMD_DISALLOWED;
}

uint8_t ll_cs_write_cached_remote_fae_table(
	const struct bt_hci_cp_le_write_cached_remote_fae_table *cmd)
{
	ARG_UNUSED(cmd);

	return BT_HCI_ERR_CMD_DISALLOWED;
}

uint8_t ll_cs_create_config(const struct bt_hci_cp_le_cs_create_config *cmd,
			    uint8_t *config_id)
{
	ARG_UNUSED(cmd);
	ARG_UNUSED(config_id);

	return BT_HCI_ERR_CMD_DISALLOWED;
}

uint8_t ll_cs_remove_config(uint16_t handle, uint8_t config_id)
{
	ARG_UNUSED(handle);
	ARG_UNUSED(config_id);

	return BT_HCI_ERR_CMD_DISALLOWED;
}

uint8_t ll_cs_set_channel_classification(const uint8_t *channel_map)
{
	ARG_UNUSED(channel_map);

	return BT_HCI_ERR_CMD_DISALLOWED;
}

uint8_t ll_cs_set_procedure_parameters(
	const struct bt_hci_cp_le_set_procedure_parameters *cmd)
{
	ARG_UNUSED(cmd);

	return BT_HCI_ERR_CMD_DISALLOWED;
}

uint8_t ll_cs_procedure_enable(uint16_t handle, uint8_t config_id,
			       uint8_t enable)
{
	ARG_UNUSED(handle);
	ARG_UNUSED(config_id);
	ARG_UNUSED(enable);

	return BT_HCI_ERR_CMD_DISALLOWED;
}

uint8_t ll_cs_test(const struct bt_hci_op_le_cs_test *cmd)
{
	ARG_UNUSED(cmd);

	return BT_HCI_ERR_CMD_DISALLOWED;
}

uint8_t ll_cs_test_end(void)
{
	return BT_HCI_ERR_CMD_DISALLOWED;
}

uint8_t ull_cs_filtered_channels_get(const uint8_t channel_map[10],
				     uint8_t *channels, uint8_t max)
{
	ARG_UNUSED(channel_map);
	ARG_UNUSED(channels);
	ARG_UNUSED(max);

	return 0;
}

void ull_cs_channels_shuffle(uint8_t *channels, uint8_t count, uint32_t seed)
{
	ARG_UNUSED(channels);
	ARG_UNUSED(count);
	ARG_UNUSED(seed);
}

uint8_t ull_cs_subevent_steps_plan(uint8_t main_mode, uint8_t sub_mode,
				   uint8_t main_mode_steps,
				   uint8_t main_mode_repetition,
				   uint8_t mode_0_steps, uint8_t rtt_type,
				   uint8_t *access_address,
				   const uint8_t *channels,
				   uint8_t channel_count,
				   struct lll_cs_step *steps,
				   uint8_t max_steps)
{
	ARG_UNUSED(main_mode);
	ARG_UNUSED(sub_mode);
	ARG_UNUSED(main_mode_steps);
	ARG_UNUSED(main_mode_repetition);
	ARG_UNUSED(mode_0_steps);
	ARG_UNUSED(rtt_type);
	ARG_UNUSED(access_address);
	ARG_UNUSED(channels);
	ARG_UNUSED(channel_count);
	ARG_UNUSED(steps);
	ARG_UNUSED(max_steps);

	return 0;
}

void ull_cs_procedure_start(uint16_t handle, uint8_t config_id,
			    uint8_t *access_address)
{
	ARG_UNUSED(handle);
	ARG_UNUSED(config_id);
	ARG_UNUSED(access_address);
}

void ull_cs_procedure_stop(uint16_t handle)
{
	ARG_UNUSED(handle);
}

uint8_t ull_cs_subevent_report(const struct lll_cs *lll,
			       uint8_t subevent_done_status,
			       uint8_t procedure_done_status)
{
	ARG_UNUSED(lll);
	ARG_UNUSED(subevent_done_status);
	ARG_UNUSED(procedure_done_status);

	return 0;
}
