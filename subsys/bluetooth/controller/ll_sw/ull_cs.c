/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/hci_types.h>

#include "util/util.h"
#include "util/memq.h"

#include "pdu.h"
#include "lll.h"
#include "lll_conn.h"

#include "ull_internal.h"
#include "ull_conn_types.h"
#include "ull_conn_internal.h"
#include "ull_cs_types.h"
#include "ull_cs_internal.h"

#include "hal/debug.h"

#if !defined(CONFIG_BT_CTLR_CHANNEL_SOUNDING_MAX_CONFIG)
#define CONFIG_BT_CTLR_CHANNEL_SOUNDING_MAX_CONFIG 4
#endif

static struct ll_cs_local_capabilities local_capabilities = {
	.num_config_supported = CONFIG_BT_CTLR_CHANNEL_SOUNDING_MAX_CONFIG,
	.max_consecutive_procedures_supported = 1,
	.num_antennas_supported = 1,
	.max_antenna_paths_supported = 1,
	.roles_supported = BT_HCI_OP_LE_CS_INITIATOR_ROLE_MASK |
			   BT_HCI_OP_LE_CS_REFLECTOR_ROLE_MASK,
	.modes_supported = BIT(BT_HCI_OP_LE_CS_MAIN_MODE_1),
	.rtt_capability = BIT(BT_HCI_OP_LE_CS_RTT_TYPE_AA_ONLY),
	.rtt_aa_only_n = 1,
	.rtt_sounding_n = 0,
	.rtt_random_payload_n = 0,
	.nadm_sounding_capability = 0,
	.nadm_random_capability = 0,
	.cs_sync_phys_supported = BIT(BT_HCI_OP_LE_CS_CS_SYNC_1M),
	.subfeatures_supported = 0,
	.t_ip1_times_supported = BIT(10),
	.t_ip2_times_supported = BIT(10),
	.t_fcs_times_supported = BIT(15),
	.t_pm_times_supported = BIT(10),
	.t_sw_time_supported = 10,
	.tx_snr_capability = 0,
};

static uint8_t channel_classification[10];

int ull_cs_init(void)
{
	memset(channel_classification, 0xFF, sizeof(channel_classification));
	return 0;
}

int ull_cs_reset(void)
{
	memset(channel_classification, 0xFF, sizeof(channel_classification));
	return 0;
}

uint8_t ll_cs_read_local_supported_capabilities(
	struct bt_hci_rp_le_read_local_supported_capabilities *rp)
{
	rp->status = BT_HCI_ERR_SUCCESS;
	rp->num_config_supported = local_capabilities.num_config_supported;
	rp->max_consecutive_procedures_supported =
		sys_cpu_to_le16(local_capabilities.max_consecutive_procedures_supported);
	rp->num_antennas_supported = local_capabilities.num_antennas_supported;
	rp->max_antenna_paths_supported = local_capabilities.max_antenna_paths_supported;
	rp->roles_supported = local_capabilities.roles_supported;
	rp->modes_supported = local_capabilities.modes_supported;
	rp->rtt_capability = local_capabilities.rtt_capability;
	rp->rtt_aa_only_n = local_capabilities.rtt_aa_only_n;
	rp->rtt_sounding_n = local_capabilities.rtt_sounding_n;
	rp->rtt_random_payload_n = local_capabilities.rtt_random_payload_n;
	rp->nadm_sounding_capability =
		sys_cpu_to_le16(local_capabilities.nadm_sounding_capability);
	rp->nadm_random_capability =
		sys_cpu_to_le16(local_capabilities.nadm_random_capability);
	rp->cs_sync_phys_supported = local_capabilities.cs_sync_phys_supported;
	rp->subfeatures_supported =
		sys_cpu_to_le16(local_capabilities.subfeatures_supported);
	rp->t_ip1_times_supported =
		sys_cpu_to_le16(local_capabilities.t_ip1_times_supported);
	rp->t_ip2_times_supported =
		sys_cpu_to_le16(local_capabilities.t_ip2_times_supported);
	rp->t_fcs_times_supported =
		sys_cpu_to_le16(local_capabilities.t_fcs_times_supported);
	rp->t_pm_times_supported =
		sys_cpu_to_le16(local_capabilities.t_pm_times_supported);
	rp->t_sw_time_supported = local_capabilities.t_sw_time_supported;
	rp->tx_snr_capability = local_capabilities.tx_snr_capability;

	return BT_HCI_ERR_SUCCESS;
}

uint8_t ll_cs_read_remote_supported_capabilities(uint16_t handle)
{
	struct ll_conn *conn;

	conn = ll_connected_get(handle);
	if (!conn) {
		return BT_HCI_ERR_UNKNOWN_CONN_ID;
	}

	return BT_HCI_ERR_SUCCESS;
}

uint8_t ll_cs_write_cached_remote_supported_capabilities(
	const struct bt_hci_cp_le_write_cached_remote_supported_capabilities *cmd)
{
	struct ll_conn *conn;
	struct ll_conn_cs_data *cs_data;
	uint16_t handle = sys_le16_to_cpu(cmd->handle);

	conn = ll_connected_get(handle);
	if (!conn) {
		return BT_HCI_ERR_UNKNOWN_CONN_ID;
	}

	cs_data = &conn->llcp.cs;

	cs_data->remote_capabilities.num_config_supported = cmd->num_config_supported;
	cs_data->remote_capabilities.max_consecutive_procedures_supported =
		sys_le16_to_cpu(cmd->max_consecutive_procedures_supported);
	cs_data->remote_capabilities.num_antennas_supported = cmd->num_antennas_supported;
	cs_data->remote_capabilities.max_antenna_paths_supported =
		cmd->max_antenna_paths_supported;
	cs_data->remote_capabilities.roles_supported = cmd->roles_supported;
	cs_data->remote_capabilities.modes_supported = cmd->modes_supported;
	cs_data->remote_capabilities.rtt_capability = cmd->rtt_capability;
	cs_data->remote_capabilities.rtt_aa_only_n = cmd->rtt_aa_only_n;
	cs_data->remote_capabilities.rtt_sounding_n = cmd->rtt_sounding_n;
	cs_data->remote_capabilities.rtt_random_payload_n = cmd->rtt_random_payload_n;
	cs_data->remote_capabilities.nadm_sounding_capability =
		sys_le16_to_cpu(cmd->nadm_sounding_capability);
	cs_data->remote_capabilities.nadm_random_capability =
		sys_le16_to_cpu(cmd->nadm_random_capability);
	cs_data->remote_capabilities.cs_sync_phys_supported = cmd->cs_sync_phys_supported;
	cs_data->remote_capabilities.subfeatures_supported =
		sys_le16_to_cpu(cmd->subfeatures_supported);
	cs_data->remote_capabilities.t_ip1_times_supported =
		sys_le16_to_cpu(cmd->t_ip1_times_supported);
	cs_data->remote_capabilities.t_ip2_times_supported =
		sys_le16_to_cpu(cmd->t_ip2_times_supported);
	cs_data->remote_capabilities.t_fcs_times_supported =
		sys_le16_to_cpu(cmd->t_fcs_times_supported);
	cs_data->remote_capabilities.t_pm_times_supported =
		sys_le16_to_cpu(cmd->t_pm_times_supported);
	cs_data->remote_capabilities.t_sw_time_supported = cmd->t_sw_time_supported;
	cs_data->remote_capabilities.tx_snr_capability = cmd->tx_snr_capability;
	cs_data->remote_capabilities_available = 1;

	return BT_HCI_ERR_SUCCESS;
}

uint8_t ll_cs_security_enable(uint16_t handle)
{
	struct ll_conn *conn;
	struct ll_conn_cs_data *cs_data;

	conn = ll_connected_get(handle);
	if (!conn) {
		return BT_HCI_ERR_UNKNOWN_CONN_ID;
	}

	cs_data = &conn->llcp.cs;
	cs_data->security_enabled = 1;

	return BT_HCI_ERR_SUCCESS;
}

uint8_t ll_cs_set_default_settings(
	const struct bt_hci_cp_le_cs_set_default_settings *cmd)
{
	struct ll_conn *conn;
	struct ll_conn_cs_data *cs_data;
	uint16_t handle = sys_le16_to_cpu(cmd->handle);

	conn = ll_connected_get(handle);
	if (!conn) {
		return BT_HCI_ERR_UNKNOWN_CONN_ID;
	}

	cs_data = &conn->llcp.cs;
	cs_data->role_enable = cmd->role_enable;
	cs_data->cs_sync_antenna_selection = cmd->cs_sync_antenna_selection;
	cs_data->max_tx_power = cmd->max_tx_power;

	return BT_HCI_ERR_SUCCESS;
}

uint8_t ll_cs_read_remote_fae_table(uint16_t handle)
{
	struct ll_conn *conn;

	conn = ll_connected_get(handle);
	if (!conn) {
		return BT_HCI_ERR_UNKNOWN_CONN_ID;
	}

	return BT_HCI_ERR_SUCCESS;
}

uint8_t ll_cs_write_cached_remote_fae_table(
	const struct bt_hci_cp_le_write_cached_remote_fae_table *cmd)
{
	struct ll_conn *conn;
	struct ll_conn_cs_data *cs_data;
	uint16_t handle = sys_le16_to_cpu(cmd->handle);

	conn = ll_connected_get(handle);
	if (!conn) {
		return BT_HCI_ERR_UNKNOWN_CONN_ID;
	}

	cs_data = &conn->llcp.cs;
	memcpy(cs_data->remote_fae_table, cmd->remote_fae_table,
	       sizeof(cs_data->remote_fae_table));
	cs_data->remote_fae_available = 1;

	return BT_HCI_ERR_SUCCESS;
}

uint8_t ll_cs_create_config(const struct bt_hci_cp_le_cs_create_config *cmd,
			    uint8_t *config_id_out)
{
	struct ll_conn *conn;
	struct ll_conn_cs_data *cs_data;
	uint16_t handle = sys_le16_to_cpu(cmd->handle);
	uint8_t config_id = cmd->config_id;

	conn = ll_connected_get(handle);
	if (!conn) {
		return BT_HCI_ERR_UNKNOWN_CONN_ID;
	}

	cs_data = &conn->llcp.cs;

	if (config_id >= CONFIG_BT_CTLR_CHANNEL_SOUNDING_MAX_CONFIG) {
		return BT_HCI_ERR_INVALID_PARAM;
	}

	cs_data->config[config_id].create_context = cmd->create_context;
	cs_data->config[config_id].main_mode_type = cmd->main_mode_type;
	cs_data->config[config_id].sub_mode_type = cmd->sub_mode_type;
	cs_data->config[config_id].min_main_mode_steps = cmd->min_main_mode_steps;
	cs_data->config[config_id].max_main_mode_steps = cmd->max_main_mode_steps;
	cs_data->config[config_id].main_mode_repetition = cmd->main_mode_repetition;
	cs_data->config[config_id].mode_0_steps = cmd->mode_0_steps;
	cs_data->config[config_id].role = cmd->role;
	cs_data->config[config_id].rtt_type = cmd->rtt_type;
	cs_data->config[config_id].cs_sync_phy = cmd->cs_sync_phy;
	memcpy(cs_data->config[config_id].channel_map, cmd->channel_map,
	       sizeof(cs_data->config[config_id].channel_map));
	cs_data->config[config_id].channel_map_repetition = cmd->channel_map_repetition;
	cs_data->config[config_id].channel_selection_type = cmd->channel_selection_type;
	cs_data->config[config_id].ch3c_shape = cmd->ch3c_shape;
	cs_data->config[config_id].ch3c_jump = cmd->ch3c_jump;

	cs_data->num_config++;
	*config_id_out = config_id;

	return BT_HCI_ERR_SUCCESS;
}

uint8_t ll_cs_remove_config(uint16_t handle, uint8_t config_id)
{
	struct ll_conn *conn;
	struct ll_conn_cs_data *cs_data;

	conn = ll_connected_get(handle);
	if (!conn) {
		return BT_HCI_ERR_UNKNOWN_CONN_ID;
	}

	cs_data = &conn->llcp.cs;

	if (config_id >= CONFIG_BT_CTLR_CHANNEL_SOUNDING_MAX_CONFIG) {
		return BT_HCI_ERR_INVALID_PARAM;
	}

	memset(&cs_data->config[config_id], 0, sizeof(cs_data->config[config_id]));

	if (cs_data->num_config > 0) {
		cs_data->num_config--;
	}

	return BT_HCI_ERR_SUCCESS;
}

uint8_t ll_cs_set_channel_classification(const uint8_t *channel_map)
{
	memcpy(channel_classification, channel_map, sizeof(channel_classification));
	return BT_HCI_ERR_SUCCESS;
}

uint8_t ll_cs_set_procedure_parameters(
	const struct bt_hci_cp_le_set_procedure_parameters *cmd)
{
	struct ll_conn *conn;
	struct ll_conn_cs_data *cs_data;
	uint16_t handle = sys_le16_to_cpu(cmd->handle);

	conn = ll_connected_get(handle);
	if (!conn) {
		return BT_HCI_ERR_UNKNOWN_CONN_ID;
	}

	cs_data = &conn->llcp.cs;
	cs_data->config_id = cmd->config_id;
	cs_data->max_procedure_len = sys_le16_to_cpu(cmd->max_procedure_len);
	cs_data->min_procedure_interval = sys_le16_to_cpu(cmd->min_procedure_interval);
	cs_data->max_procedure_interval = sys_le16_to_cpu(cmd->max_procedure_interval);
	cs_data->max_procedure_count = sys_le16_to_cpu(cmd->max_procedure_count);
	memcpy(cs_data->min_subevent_len, cmd->min_subevent_len,
	       sizeof(cs_data->min_subevent_len));
	memcpy(cs_data->max_subevent_len, cmd->max_subevent_len,
	       sizeof(cs_data->max_subevent_len));
	cs_data->tone_antenna_config_selection = cmd->tone_antenna_config_selection;
	cs_data->phy = cmd->phy;
	cs_data->tx_power_delta = cmd->tx_power_delta;
	cs_data->preferred_peer_antenna = cmd->preferred_peer_antenna;
	cs_data->snr_control_initiator = cmd->snr_control_initiator;
	cs_data->snr_control_reflector = cmd->snr_control_reflector;

	return BT_HCI_ERR_SUCCESS;
}

uint8_t ll_cs_procedure_enable(uint16_t handle, uint8_t config_id,
			       uint8_t enable)
{
	struct ll_conn *conn;
	struct ll_conn_cs_data *cs_data;

	conn = ll_connected_get(handle);
	if (!conn) {
		return BT_HCI_ERR_UNKNOWN_CONN_ID;
	}

	cs_data = &conn->llcp.cs;

	if (config_id >= CONFIG_BT_CTLR_CHANNEL_SOUNDING_MAX_CONFIG) {
		return BT_HCI_ERR_INVALID_PARAM;
	}

	cs_data->config_id = config_id;
	cs_data->procedure_enable = enable;

	return BT_HCI_ERR_SUCCESS;
}

uint8_t ll_cs_test(const struct bt_hci_op_le_cs_test *cmd)
{
	return BT_HCI_ERR_CMD_DISALLOWED;
}

uint8_t ll_cs_test_end(void)
{
	return BT_HCI_ERR_CMD_DISALLOWED;
}
