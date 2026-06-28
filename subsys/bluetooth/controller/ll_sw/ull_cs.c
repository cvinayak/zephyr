/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include "ull_llcp.h"

#include "lll_cs.h"

#include "hal/radio_cs.h"

#include "hal/debug.h"

/* CS procedure execution context. One per connection that has an active CS
 * procedure. Uses a controller ticker to periodically schedule CS events and a
 * mayfly to dispatch the LLL radio event from the ULL_HIGH ticker timeout into
 * the LLL execution context. The LLL radio event walks the
 * event -> subevent -> step hierarchy, captures the per-step measurement
 * results and reports the CS subevent results towards the ULL and the Host.
 */
struct ull_cs_proc {
	uint16_t handle;
	uint8_t  config_id;
	uint8_t  access_address[4];
	uint16_t procedure_counter;
	uint16_t procedure_count_max;
	uint32_t procedure_interval_us;
	uint16_t event_interval;
	uint8_t  subevents_per_event;
	uint16_t subevent_interval;
	uint8_t  subevent_len[3];
	uint8_t  active;
	uint8_t  ticker_id;
	/* LLL radio event context and its prepare mayfly. Used to schedule the
	 * event -> subevent -> step radio stepping and the per-step measurement
	 * capture. The struct ull_hdr provides the prepare reference counting
	 * expected by the LLL prepare/done pipeline.
	 */
	struct ull_hdr ull;
	struct lll_cs lll;
	memq_link_t lll_prepare_link;
	struct mayfly lll_prepare_mfy;
	struct lll_prepare_param lll_prepare_param;
};

#define ULL_CS_MAX_PROCEDURES CONFIG_BT_MAX_CONN
static struct ull_cs_proc cs_procs[ULL_CS_MAX_PROCEDURES];

/* CS Test mode state. Uses a dedicated procedure slot with the test
 * connection handle (0x0FFF) and configuration derived from the CS_TEST
 * command parameters.
 */
static struct {
	uint8_t  active;
	uint8_t  main_mode;
	uint8_t  sub_mode;
	uint8_t  main_mode_steps;
	uint8_t  main_mode_repetition;
	uint8_t  mode_0_steps;
	uint8_t  rtt_type;
	uint8_t  role;
	uint8_t  num_ant_paths;
	struct ull_cs_proc proc;
} cs_test_state;

static void cs_ticker_cb(uint32_t ticks_at_expire, uint32_t ticks_drift,
			 uint32_t remainder, uint16_t lazy, uint8_t force,
			 void *param);
static void cs_ticker_op_start_cb(uint32_t status, void *param);
static void cs_ticker_op_stop_cb(uint32_t status, void *param);
static void cs_proc_lll_init(struct ull_cs_proc *proc);

static struct ll_cs_local_capabilities local_capabilities = {
	.num_config_supported = CONFIG_BT_CTLR_CHANNEL_SOUNDING_MAX_CONFIG,
	.max_consecutive_procedures_supported = 1,
	.num_antennas_supported = 1,
	.max_antenna_paths_supported = 1,
	.roles_supported = BT_HCI_OP_LE_CS_INITIATOR_ROLE_MASK |
			   BT_HCI_OP_LE_CS_REFLECTOR_ROLE_MASK,
	.modes_supported = BIT(BT_HCI_OP_LE_CS_MAIN_MODE_1) |
			   BIT(BT_HCI_OP_LE_CS_MAIN_MODE_2),
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

static void cs_proc_lll_init(struct ull_cs_proc *proc)
{
	/* Initialise the LLL prepare reference counting and the back pointer
	 * from the LLL context to the owning ULL header, as expected by the
	 * LLL prepare/done pipeline. The mayfly carries the per-event
	 * struct lll_prepare_param from the ULL ticker callback into the LLL
	 * execution context.
	 */
	ull_hdr_init(&proc->ull);
	proc->lll.hdr.parent = &proc->ull;

	proc->lll_prepare_mfy._req = 0U;
	proc->lll_prepare_mfy._ack = 0U;
	proc->lll_prepare_mfy._link = &proc->lll_prepare_link;
	proc->lll_prepare_mfy.param = NULL;
	proc->lll_prepare_mfy.fp = lll_cs_prepare;
}

int ull_cs_init(void)
{
	memset(channel_classification, 0xFF, sizeof(channel_classification));

	for (uint8_t i = 0U; i < ULL_CS_MAX_PROCEDURES; i++) {
		cs_procs[i].active = 0U;
		cs_procs[i].ticker_id = TICKER_ID_CS_BASE + i;
		cs_proc_lll_init(&cs_procs[i]);
	}

	cs_test_state.active = 0U;
	cs_test_state.proc.ticker_id = TICKER_ID_CS_TEST;
	cs_proc_lll_init(&cs_test_state.proc);

	return 0;
}

int ull_cs_reset(void)
{
	memset(channel_classification, 0xFF, sizeof(channel_classification));

	for (uint8_t i = 0U; i < ULL_CS_MAX_PROCEDURES; i++) {
		if (cs_procs[i].active) {
			cs_procs[i].active = 0U;
			(void)ticker_stop(TICKER_INSTANCE_ID_CTLR,
					  TICKER_USER_ID_ULL_HIGH,
					  cs_procs[i].ticker_id,
					  cs_ticker_op_stop_cb, &cs_procs[i]);
		}
	}

	if (cs_test_state.active) {
		cs_test_state.active = 0U;
		(void)ticker_stop(TICKER_INSTANCE_ID_CTLR,
				  TICKER_USER_ID_ULL_HIGH,
				  cs_test_state.proc.ticker_id,
				  cs_ticker_op_stop_cb, &cs_test_state.proc);
	}

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

	return ull_cp_cs_read_remote_supported_capabilities(conn);
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

	return ull_cp_cs_security_enable(conn);
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

	return ull_cp_cs_read_remote_fae_table(conn);
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

	/* Validate main mode is supported (mode 0, 1, or 2) */
	if (cmd->main_mode_type > BT_HCI_OP_LE_CS_MAIN_MODE_2) {
		return BT_HCI_ERR_UNSUPP_FEATURE_PARAM_VAL;
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

	return ull_cp_cs_create_config(conn, config_id,
				       BT_HCI_LE_CS_CONFIG_ACTION_CREATED);
}

uint8_t ll_cs_remove_config(uint16_t handle, uint8_t config_id)
{
	struct ll_conn *conn;
	struct ll_conn_cs_data *cs_data;
	uint8_t err;

	conn = ll_connected_get(handle);
	if (!conn) {
		return BT_HCI_ERR_UNKNOWN_CONN_ID;
	}

	cs_data = &conn->llcp.cs;

	if (config_id >= CONFIG_BT_CTLR_CHANNEL_SOUNDING_MAX_CONFIG) {
		return BT_HCI_ERR_INVALID_PARAM;
	}

	/* Per spec section 6.38: if an active CS procedure is using this
	 * config, terminate it before removing the config.
	 */
	if (cs_data->procedure_enable && cs_data->config_id == config_id) {
		err = ull_cp_cs_procedure_enable(conn, config_id, 0U);
		if (err) {
			return err;
		}
		cs_data->procedure_enable = 0U;
	}

	memset(&cs_data->config[config_id], 0, sizeof(cs_data->config[config_id]));

	if (cs_data->num_config > 0) {
		cs_data->num_config--;
	}

	return ull_cp_cs_create_config(conn, config_id,
				       BT_HCI_LE_CS_CONFIG_ACTION_REMOVED);
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

	return ull_cp_cs_procedure_enable(conn, config_id, enable);
}

uint8_t ll_cs_test(const struct bt_hci_op_le_cs_test *cmd)
{
	if (cs_test_state.active) {
		return BT_HCI_ERR_CMD_DISALLOWED;
	}

	cs_test_state.main_mode = cmd->main_mode_type;
	cs_test_state.sub_mode = cmd->sub_mode_type;
	cs_test_state.main_mode_steps = 10U;
	cs_test_state.main_mode_repetition = cmd->main_mode_repetition;
	cs_test_state.mode_0_steps = cmd->mode_0_steps;
	cs_test_state.rtt_type = cmd->rtt_type;
	cs_test_state.role = cmd->role;
	cs_test_state.num_ant_paths = 1U;

	cs_test_state.proc.handle = BT_HCI_LE_CS_TEST_CONN_HANDLE;
	cs_test_state.proc.config_id = 0U;
	*((uint32_t *)cs_test_state.proc.access_address) = 0x8E89BED6UL;
	cs_test_state.proc.procedure_counter = 0U;
	cs_test_state.proc.procedure_count_max = 0U; /* Unlimited */
	cs_test_state.proc.procedure_interval_us = 100000U;
	cs_test_state.proc.subevents_per_event = 1U;
	cs_test_state.proc.active = 1U;
	cs_test_state.active = 1U;

	/* Populate the LLL radio event context for CS Test mode so the LLL can
	 * step through the test subevent and steps when the radio Channel
	 * Sounding hardware is present. CS Test mode allows all channels.
	 */
	{
		struct lll_cs *lll = &cs_test_state.proc.lll;

		lll->handle = BT_HCI_LE_CS_TEST_CONN_HANDLE;
		lll->config_id = 0U;
		memcpy(lll->access_address, cs_test_state.proc.access_address,
		       sizeof(lll->access_address));
		lll->role = (cs_test_state.role ==
			     BT_HCI_OP_LE_CS_INITIATOR_ROLE) ?
				    LLL_CS_ROLE_INITIATOR : LLL_CS_ROLE_REFLECTOR;
		lll->phy = PHY_1M;
		lll->main_mode = cs_test_state.main_mode;
		lll->sub_mode = cs_test_state.sub_mode;
		lll->main_mode_steps = cs_test_state.main_mode_steps;
		lll->main_mode_repetition = cs_test_state.main_mode_repetition;
		lll->mode_0_steps = cs_test_state.mode_0_steps;
		lll->rtt_type = cs_test_state.rtt_type;
		/* CS Test mode uses the Host configured channel classification
		 * when set, falling back to all usable Channel Sounding
		 * channels.
		 */
		memcpy(lll->channel_map, channel_classification,
		       sizeof(lll->channel_map));
		lll->subevents_per_event = 1U;
		lll->subevent_interval_us = 0U;
		lll->subevent_len_us = 0U;
		lll->step_interval_us = EVENT_IFS_US;
		lll->procedure_counter = 0U;
		lll->procedure_count_max = 0U;
	}

	{
		uint32_t ticks_interval =
			HAL_TICKER_US_TO_TICKS(cs_test_state.proc.procedure_interval_us);
		uint32_t ret;

		/* ll_cs_test() runs in the thread (HCI) execution context */
		ret = ticker_start(TICKER_INSTANCE_ID_CTLR,
				   TICKER_USER_ID_THREAD,
				   cs_test_state.proc.ticker_id,
				   ticker_ticks_now_get(), ticks_interval,
				   ticks_interval,
				   HAL_TICKER_REMAINDER(cs_test_state.proc.procedure_interval_us),
				   TICKER_NULL_LAZY, TICKER_NULL_SLOT,
				   cs_ticker_cb, &cs_test_state.proc,
				   cs_ticker_op_start_cb, &cs_test_state.proc);
		LL_ASSERT((ret == TICKER_STATUS_SUCCESS) ||
			  (ret == TICKER_STATUS_BUSY));
	}

	return BT_HCI_ERR_SUCCESS;
}

uint8_t ll_cs_test_end(void)
{
	if (!cs_test_state.active) {
		return BT_HCI_ERR_CMD_DISALLOWED;
	}

	cs_test_state.active = 0U;
	cs_test_state.proc.active = 0U;
	(void)ticker_stop(TICKER_INSTANCE_ID_CTLR, TICKER_USER_ID_THREAD,
			  cs_test_state.proc.ticker_id, cs_ticker_op_stop_cb,
			  &cs_test_state.proc);

	return BT_HCI_ERR_SUCCESS;
}

/* Total number of Channel Sounding RF channel indices (2402..2480 MHz). */
#define ULL_CS_CHAN_COUNT 79U

/* Default single tone duration T_PM in microseconds. */
#define ULL_CS_TONE_DURATION_US 40U

/* Value used by the host to indicate that no sub mode is configured. */
#define ULL_CS_SUB_MODE_NONE 0xFFU

/* Returns true if the given Channel Sounding channel index (0..78) is allowed
 * to be used for a CS step. The indices 0, 1, 23, 24, 25, 77 and 78 are not
 * allowed (they overlap the LE primary advertising channels and the band
 * guards). Refer to the Bluetooth Core specification, Vol 6, Part A, Section 5.
 */
static bool ull_cs_chan_is_usable(uint8_t index)
{
	if ((index == 0U) || (index == 1U) || (index == 23U) ||
	    (index == 24U) || (index == 25U) || (index == 77U) ||
	    (index == 78U)) {
		return false;
	}

	return index < ULL_CS_CHAN_COUNT;
}

uint8_t ull_cs_filtered_channels_get(const uint8_t channel_map[10],
				     uint8_t *channels, uint8_t max)
{
	uint8_t count = 0U;

	for (uint8_t index = 0U; index < ULL_CS_CHAN_COUNT; index++) {
		if (count >= max) {
			break;
		}

		/* Skip channels not allowed for Channel Sounding. */
		if (!ull_cs_chan_is_usable(index)) {
			continue;
		}

		/* Skip channels not enabled in the agreed channel map. */
		if ((channel_map[index >> 3] & BIT(index & 0x07)) == 0U) {
			continue;
		}

		channels[count] = index;
		count++;
	}

	return count;
}

void ull_cs_channels_shuffle(uint8_t *channels, uint8_t count, uint32_t seed)
{
	/* Fisher-Yates shuffle driven by a linear congruential generator seeded
	 * with the procedure/event counter, providing the per-procedure channel
	 * reordering of the channel selection algorithm #1.
	 */
	uint32_t state = seed ? seed : 1U;

	for (uint8_t i = count; i > 1U; i--) {
		uint8_t j;
		uint8_t tmp;

		state = (state * 1664525U) + 1013904223U;
		j = (uint8_t)(state % i);

		tmp = channels[i - 1U];
		channels[i - 1U] = channels[j];
		channels[j] = tmp;
	}
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
	uint8_t chan_idx = 0U;
	uint8_t count = 0U;

	if ((channels == NULL) || (channel_count == 0U) || (steps == NULL) ||
	    (max_steps == 0U)) {
		return 0U;
	}

	/* Helper to populate a single step with the next channel from the list. */
#define ULL_CS_STEP_ADD(step_mode)                                             \
	do {                                                                   \
		struct lll_cs_step *step = &steps[count];                      \
		memcpy(step->access_address, access_address, sizeof(step->access_address)); \
		step->mode = (step_mode);                                      \
		step->channel_index = channels[chan_idx];                      \
		step->rtt_type = rtt_type;                                     \
		step->num_ant_paths = 1U;                                      \
		step->num_tones = 1U;                                          \
		step->tone_duration_us = ULL_CS_TONE_DURATION_US;              \
		step->antenna_selection = 0U;                                  \
		chan_idx = (uint8_t)((chan_idx + 1U) % channel_count);         \
		count++;                                                       \
	} while (0)

	/* Mode-0 steps used for frequency offset and timing recovery. */
	for (uint8_t i = 0U; (i < mode_0_steps) && (count < max_steps); i++) {
		ULL_CS_STEP_ADD(LLL_CS_MODE_0);
	}

	/* Main mode steps, interleaving sub mode steps every
	 * main_mode_repetition steps when a sub mode is configured.
	 */
	for (uint8_t i = 0U; (i < main_mode_steps) && (count < max_steps); i++) {
		ULL_CS_STEP_ADD(main_mode);

		if ((sub_mode != ULL_CS_SUB_MODE_NONE) &&
		    (main_mode_repetition != 0U) &&
		    (((i + 1U) % main_mode_repetition) == 0U) &&
		    (count < max_steps)) {
			ULL_CS_STEP_ADD(sub_mode);
		}
	}

#undef ULL_CS_STEP_ADD

	return count;
}

/*
 * CS Subevent Result Reporting
 *
 * The Channel Sounding subevent results are generated by the LLL radio event
 * (see lll_cs.c) which captures the per-step RTT timestamps and IQ/phase (PCT)
 * samples from the radio Channel Sounding hardware. Once a subevent completes
 * the LLL calls ull_cs_subevent_report() which serializes the captured per-step
 * results into a NODE_RX_TYPE_CS_SUBEVENT_RESULT notification and enqueues it
 * towards the ULL. The HCI layer then encodes the LE CS Subevent Result meta
 * event towards the Host so that the sample/application can process it.
 *
 * The serialization helpers below are pure (no allocation) and operate on the
 * planned steps together with the per-step results captured by the LLL.
 */

/* Serialize mode-0 step data for the initiator role into the output buffer.
 * Returns the number of bytes written.
 */
static uint8_t cs_step_data_mode_0_encode(uint8_t *buf)
{
	struct bt_hci_le_cs_step_data_mode_0_initiator *d =
		(struct bt_hci_le_cs_step_data_mode_0_initiator *)buf;

	d->packet_quality_aa_check = 0U;
	d->packet_quality_bit_errors = 0U;
	d->packet_rssi = 0x7FU; /* Not available */
	d->packet_antenna = 0U;
	d->measured_freq_offset = 0U;

	return (uint8_t)sizeof(*d);
}

/* Serialize mode-1 step data into the output buffer from the RTT timestamp
 * captured by the LLL for the step. Returns the number of bytes written.
 */
static uint8_t cs_step_data_mode_1_encode(uint8_t *buf, uint32_t rtt)
{
	struct bt_hci_le_cs_step_data_mode_1 *d =
		(struct bt_hci_le_cs_step_data_mode_1 *)buf;

	d->packet_quality_aa_check = 0U;
	d->packet_quality_bit_errors = 0U;
	d->packet_nadm = 0xFFU; /* Not available */
	d->packet_rssi = 0x7FU; /* Not available */

	/* The radio captured Round Trip Time is reported as the Time of
	 * Arrival/Time of Departure value used by the Host distance estimation.
	 */
	d->toa_tod_initiator = (int16_t)(rtt & 0xFFFFU);
	d->packet_antenna = 0U;

	return (uint8_t)sizeof(*d);
}

/* Serialize mode-2 step data (tone/PBR) into the output buffer from the
 * IQ/phase sample captured by the LLL for the step. Produces one tone_info
 * entry per antenna path. Returns the number of bytes written.
 */
static uint8_t cs_step_data_mode_2_encode(uint8_t *buf, uint8_t num_ant_paths,
					  const struct lll_cs_iq_sample *iq)
{
	struct bt_hci_le_cs_step_data_mode_2 *d =
		(struct bt_hci_le_cs_step_data_mode_2 *)buf;
	uint8_t len;

	d->antenna_permutation_index = 0U;
	len = (uint8_t)sizeof(*d);

	/* Pack the captured 12-bit signed I and Q components of the tone into
	 * the 24-bit phase correction term of each antenna path tone_info.
	 */
	for (uint8_t ap = 0U; ap < num_ant_paths; ap++) {
		struct bt_hci_le_cs_step_data_tone_info *ti =
			(struct bt_hci_le_cs_step_data_tone_info *)&buf[len];
		int16_t phase_i = iq->i;
		int16_t phase_q = iq->q;

		ti->phase_correction_term[0] = (uint8_t)(phase_i & 0xFFU);
		ti->phase_correction_term[1] = (uint8_t)(((phase_i >> 8) & 0x0FU) |
							 ((phase_q & 0x0FU) << 4));
		ti->phase_correction_term[2] = (uint8_t)((phase_q >> 4) & 0xFFU);
		ti->quality_indicator = 0U; /* Good quality */
		ti->extension_indicator = 0U; /* No extension */
		len += (uint8_t)sizeof(*ti);
	}

	return len;
}

/* Serialize the per-step results of a complete subevent into the provided
 * node_rx_csound buffer using the LLL captured RTT timestamps and IQ/phase
 * samples. Returns the total step_data_len written.
 */
static uint8_t cs_subevent_result_build(struct node_rx_csound *cs,
					const struct lll_cs_step *steps,
					const uint32_t *step_rtt,
					const struct lll_cs_iq_sample *step_iq,
					uint8_t step_count)
{
	uint8_t offset = 0U;
	uint8_t reported = 0U;

	for (uint8_t i = 0U; i < step_count; i++) {
		struct bt_hci_evt_le_cs_subevent_result_step *hdr;
		uint8_t data_len = 0U;

		/* Check there is space for at least the step header */
		if ((offset + 3U) > NODE_RX_CS_STEP_DATA_MAX) {
			break;
		}

		hdr = (struct bt_hci_evt_le_cs_subevent_result_step *)
			&cs->step_data[offset];
		hdr->step_mode = steps[i].mode;
		hdr->step_channel = steps[i].channel_index;

		uint8_t *data_buf = &cs->step_data[offset + 3U];
		uint8_t space = (uint8_t)(NODE_RX_CS_STEP_DATA_MAX - offset - 3U);

		switch (steps[i].mode) {
		case LLL_CS_MODE_0:
			if (space >= sizeof(struct bt_hci_le_cs_step_data_mode_0_initiator)) {
				data_len = cs_step_data_mode_0_encode(data_buf);
			}
			break;
		case LLL_CS_MODE_1:
			if (space >= sizeof(struct bt_hci_le_cs_step_data_mode_1)) {
				data_len = cs_step_data_mode_1_encode(
					data_buf, step_rtt[i]);
			}
			break;
		case LLL_CS_MODE_2:
			if (space >= (sizeof(struct bt_hci_le_cs_step_data_mode_2) +
				      steps[i].num_ant_paths *
				      sizeof(struct bt_hci_le_cs_step_data_tone_info))) {
				data_len = cs_step_data_mode_2_encode(
					data_buf, steps[i].num_ant_paths,
					&step_iq[i]);
			}
			break;
		default:
			break;
		}

		if (data_len == 0U && steps[i].mode != LLL_CS_MODE_0) {
			/* Could not fit step data, stop here */
			break;
		}

		hdr->step_data_length = data_len;
		offset += 3U + data_len;
		reported++;
	}

	cs->num_steps_reported = reported;
	cs->step_data_len = offset;

	return offset;
}

uint8_t ull_cs_subevent_report(const struct lll_cs *lll,
			       uint8_t subevent_done_status,
			       uint8_t procedure_done_status)
{
	struct node_rx_pdu *ntf;
	struct node_rx_csound *cs;

	/* Allocate a receive node from the LLL receive buffer pool. This runs
	 * in the LLL radio event context, mirroring the connection receive
	 * path. If no buffer is available the subevent result is dropped.
	 */
	if (!ull_pdu_rx_alloc_peek(1)) {
		return 0U;
	}

	ntf = ull_pdu_rx_alloc();

	ntf->hdr.type = NODE_RX_TYPE_CS_SUBEVENT_RESULT;
	ntf->hdr.handle = lll->handle;

	cs = (struct node_rx_csound *)ntf->pdu;
	memset(cs, 0, sizeof(*cs));
	cs->status = BT_HCI_ERR_SUCCESS;
	cs->config_id = lll->config_id;
	cs->start_acl_conn_event = 0U;
	cs->procedure_counter = lll->procedure_counter;
	cs->frequency_compensation = 0U;
	cs->reference_power_level = 0;
	cs->procedure_done_status = procedure_done_status;
	cs->subevent_done_status = subevent_done_status;
	cs->num_antenna_paths = 1U;

	cs_subevent_result_build(cs, lll->steps, lll->step_rtt, lll->step_iq,
				 lll->step_count);

	ull_rx_put(ntf->hdr.link, ntf);

	return 1U;
}

/* Controller ticker timeout for a CS procedure. Runs in the ULL_HIGH execution
 * context at every CS procedure (event) interval. It schedules the LLL radio
 * event that performs the actual event -> subevent -> step stepping and the
 * per-step measurement capture; the LLL reports the CS subevent results towards
 * the ULL and the Host. The ULL owns the procedure lifecycle (procedure
 * counter, completion and ticker stop).
 */
static void cs_ticker_cb(uint32_t ticks_at_expire, uint32_t ticks_drift,
			 uint32_t remainder, uint16_t lazy, uint8_t force,
			 void *param)
{
	struct ull_cs_proc *proc = param;
	struct lll_prepare_param *p;
	uint32_t ret;
	uint8_t ref;

	ARG_UNUSED(ticks_drift);

	if (!proc->active) {
		return;
	}

	/* Schedule the LLL radio event that performs the actual
	 * event -> subevent -> step stepping and reports the captured CS
	 * subevent results. The LLL prepare reference taken here is released by
	 * the LLL done pipeline when the event completes.
	 */
	proc->lll.procedure_counter = proc->procedure_counter;

	ref = ull_ref_inc(&proc->ull);
	LL_ASSERT(ref);

	p = &proc->lll_prepare_param;
	p->ticks_at_expire = ticks_at_expire;
	p->remainder = remainder;
	p->lazy = lazy;
	p->force = force;
	p->param = &proc->lll;
	proc->lll_prepare_mfy.param = p;

	ret = mayfly_enqueue(TICKER_USER_ID_ULL_HIGH, TICKER_USER_ID_LLL,
			     0U, &proc->lll_prepare_mfy);
	LL_ASSERT(!ret);

	/* Advance the procedure counter and stop the ticker once the configured
	 * maximum number of CS procedures has been reached.
	 */
	proc->procedure_counter++;

	if ((proc->procedure_count_max != 0U) &&
	    (proc->procedure_counter >= proc->procedure_count_max)) {
		proc->active = 0U;
		(void)ticker_stop(TICKER_INSTANCE_ID_CTLR,
				  TICKER_USER_ID_ULL_HIGH, proc->ticker_id,
				  cs_ticker_op_stop_cb, proc);
	}
}

static void cs_ticker_op_start_cb(uint32_t status, void *param)
{
	ARG_UNUSED(param);

	LL_ASSERT_ERR(status == TICKER_STATUS_SUCCESS);
}

static void cs_ticker_op_stop_cb(uint32_t status, void *param)
{
	ARG_UNUSED(param);

	LL_ASSERT_ERR((status == TICKER_STATUS_SUCCESS) ||
		      (status == TICKER_STATUS_FAILURE));
}

void ull_cs_procedure_start(uint16_t handle, uint8_t config_id,
			    uint8_t *access_address)
{
	struct ull_cs_proc *proc = NULL;
	struct ll_conn *conn;
	struct ll_conn_cs_data *cs_data;
	uint32_t ticks_interval;
	uint32_t interval_us;
	uint32_t ret;

	conn = ll_connected_get(handle);
	if (!conn) {
		return;
	}

	cs_data = &conn->llcp.cs;

	/* Find a free procedure slot or reuse existing for this handle */
	for (uint8_t i = 0U; i < ULL_CS_MAX_PROCEDURES; i++) {
		if (cs_procs[i].active && cs_procs[i].handle == handle) {
			/* Already running, update */
			proc = &cs_procs[i];
			break;
		}
		if (!cs_procs[i].active && !proc) {
			proc = &cs_procs[i];
		}
	}

	if (!proc) {
		return;
	}

	/* If reusing an active slot, stop the running ticker first */
	if (proc->active) {
		proc->active = 0U;
		(void)ticker_stop(TICKER_INSTANCE_ID_CTLR,
				  TICKER_USER_ID_ULL_HIGH, proc->ticker_id,
				  cs_ticker_op_stop_cb, proc);
	}

	proc->handle = handle;
	proc->config_id = config_id;
	memcpy(proc->access_address, access_address, sizeof(proc->access_address));
	proc->procedure_counter = 0U;
	proc->procedure_count_max = cs_data->max_procedure_count;

	/* Adopt the scheduling parameters negotiated with the peer in the
	 * LL_CS_REQ/RSP/IND exchange when available, otherwise fall back to
	 * the host-provided procedure parameters.
	 */
	if (cs_data->schedule.valid) {
		proc->procedure_count_max = cs_data->schedule.procedure_count;
		proc->event_interval = cs_data->schedule.event_interval;
		proc->subevents_per_event = cs_data->schedule.subevents_per_event;
		proc->subevent_interval = cs_data->schedule.subevent_interval;
		memcpy(proc->subevent_len, cs_data->schedule.subevent_len,
		       sizeof(proc->subevent_len));
	} else {
		proc->event_interval = 1U;
		proc->subevents_per_event = 1U;
		proc->subevent_interval = 0U;
		memcpy(proc->subevent_len, cs_data->max_subevent_len,
		       sizeof(proc->subevent_len));
	}

	if (proc->subevents_per_event == 0U) {
		proc->subevents_per_event = 1U;
	}

	/* Derive the CS event interval in microseconds. The negotiated
	 * procedure interval is expressed in connection interval units; the
	 * connection interval itself is in 1.25 ms units. Clamp to a minimum
	 * of 50 ms to bound the rate of scheduled CS events in this
	 * software-only path.
	 */
	if (cs_data->schedule.valid && (cs_data->schedule.procedure_interval != 0U) &&
	    (conn->lll.interval != 0U)) {
		interval_us = (uint32_t)cs_data->schedule.procedure_interval *
			      (uint32_t)conn->lll.interval * 1250U;
	} else {
		interval_us = (uint32_t)cs_data->max_procedure_interval * 1250U;
	}
	if (interval_us < 50000U) {
		interval_us = 50000U;
	}
	proc->procedure_interval_us = interval_us;

	/* Populate the LLL radio event context from the negotiated control
	 * procedure configuration so the LLL can step through the
	 * event -> subevent -> step hierarchy using the radio Channel Sounding
	 * hardware. The CS_SYNC PHY defaults to 1M.
	 */
	{
		struct lll_cs *lll = &proc->lll;
		uint32_t subevent_len_us;
		uint32_t subevent_interval_us;
		uint8_t  steps;

		lll->handle = handle;
		lll->config_id = config_id;
		memcpy(lll->access_address, access_address, sizeof(lll->access_address));
		lll->role = (cs_data->config[config_id].role ==
			     BT_HCI_OP_LE_CS_INITIATOR_ROLE) ?
				    LLL_CS_ROLE_INITIATOR : LLL_CS_ROLE_REFLECTOR;
		lll->phy = PHY_1M;
		lll->main_mode = cs_data->config[config_id].main_mode_type;
		lll->sub_mode = cs_data->config[config_id].sub_mode_type;
		lll->main_mode_steps =
			cs_data->config[config_id].max_main_mode_steps;
		lll->main_mode_repetition =
			cs_data->config[config_id].main_mode_repetition;
		lll->mode_0_steps = cs_data->config[config_id].mode_0_steps;
		lll->rtt_type = cs_data->config[config_id].rtt_type;
		memcpy(lll->channel_map, cs_data->config[config_id].channel_map,
		       sizeof(lll->channel_map));

		lll->subevents_per_event = proc->subevents_per_event;
		lll->procedure_counter = 0U;
		lll->procedure_count_max = proc->procedure_count_max;

		/* The negotiated subevent length is a 24-bit microsecond value
		 * and the subevent interval is expressed in 0.625 ms units. The
		 * per-step start-to-start spacing is derived by spreading the
		 * planned steps across the subevent length.
		 */
		subevent_len_us = sys_get_le24(proc->subevent_len);
		subevent_interval_us = (uint32_t)proc->subevent_interval * 625U;
		if (subevent_interval_us == 0U) {
			subevent_interval_us = subevent_len_us;
		}
		lll->subevent_len_us = subevent_len_us;
		lll->subevent_interval_us = subevent_interval_us;

		steps = (uint8_t)(lll->mode_0_steps + lll->main_mode_steps);
		if (steps == 0U) {
			steps = 1U;
		}
		lll->step_interval_us = (subevent_len_us != 0U) ?
					(subevent_len_us / steps) : EVENT_IFS_US;
		if (lll->step_interval_us == 0U) {
			lll->step_interval_us = EVENT_IFS_US;
		}
	}

	ticks_interval = HAL_TICKER_US_TO_TICKS(interval_us);

	proc->active = 1U;
	ret = ticker_start(TICKER_INSTANCE_ID_CTLR, TICKER_USER_ID_ULL_HIGH,
			   proc->ticker_id, ticker_ticks_now_get(),
			   ticks_interval, ticks_interval,
			   HAL_TICKER_REMAINDER(interval_us), TICKER_NULL_LAZY,
			   TICKER_NULL_SLOT, cs_ticker_cb, proc,
			   cs_ticker_op_start_cb, proc);
	LL_ASSERT((ret == TICKER_STATUS_SUCCESS) ||
		  (ret == TICKER_STATUS_BUSY));
}

void ull_cs_procedure_stop(uint16_t handle)
{
	for (uint8_t i = 0U; i < ULL_CS_MAX_PROCEDURES; i++) {
		if (cs_procs[i].active && cs_procs[i].handle == handle) {
			cs_procs[i].active = 0U;
			(void)ticker_stop(TICKER_INSTANCE_ID_CTLR,
					  TICKER_USER_ID_ULL_HIGH,
					  cs_procs[i].ticker_id,
					  cs_ticker_op_stop_cb, &cs_procs[i]);
			break;
		}
	}
}
