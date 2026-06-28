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
#include "lll_clock.h"
#include "lll/lll_vendor.h"

#include "ull_tx_queue.h"

#include "isoal.h"

#include "ull_iso_types.h"
#include "ull_cs_types.h"
#include "ull_cs_drbg.h"
#include "ull_conn_types.h"
#include "ull_conn_iso_types.h"

#include "ull_internal.h"
#include "ull_conn_internal.h"
#include "ull_iso_internal.h"
#include "ull_conn_iso_internal.h"
#include "ull_cs_internal.h"

#include "ull_llcp.h"

#include "lll_cs.h"

#include "hal/radio.h"
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
	/* CS DRBG instance and channel selection algorithm #3b state used to
	 * plan the steps of the CS subevents of this procedure.
	 */
	struct ull_cs_drbg drbg;
	uint16_t step_counter;
	uint8_t  min_main_mode_steps;
	uint8_t  max_main_mode_steps;
	uint8_t  chm_repetition;
	uint8_t  chm_repetition_count;
	uint8_t  filtered_count;
	uint8_t  mode_0_index;
	uint8_t  non_mode_0_index;
	uint8_t  filtered[LLL_CS_MAX_CHANNELS];
	uint8_t  mode_0_channels[LLL_CS_MAX_CHANNELS];
	uint8_t  non_mode_0_channels[LLL_CS_MAX_CHANNELS];
	uint16_t procedure_counter;
	uint16_t procedure_count_max;
	uint32_t procedure_interval_us;
	uint16_t event_interval;
	uint8_t  subevents_per_event;
	uint16_t subevent_interval;
	uint8_t  subevent_len[3];
	uint8_t  active;
	uint8_t  ticker_id;
	uint8_t  is_test;
	/* Procedure instance state, see CS_STATE_*. Ownership of the planned
	 * steps in the LLL context follows the state: the ULL_LOW planner in
	 * CS_STATE_PLAN and CS_STATE_DONE, the ACL event hook in
	 * CS_STATE_READY, and the ticker and LLL in CS_STATE_SCHEDULED and
	 * CS_STATE_STARTED.
	 */
	volatile uint8_t state;
	/* ACL connection event counter that anchors the procedure instance
	 * being planned or scheduled, procedure interval in ACL connection
	 * events and offset from the anchoring ACL anchor point.
	 */
	uint16_t instance_event_counter;
	uint16_t procedure_interval;
	uint32_t offset_us;
	/* Latest ACL connection event counter seen by the ACL event hook. */
	volatile uint16_t acl_event_counter;
	/* Planner mayflies, one per caller execution context. */
	memq_link_t plan_link_lll;
	struct mayfly plan_mfy_lll;
	memq_link_t plan_link_ull;
	struct mayfly plan_mfy_ull;
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

/* CS procedure instance states. */
#define CS_STATE_IDLE      0U /* No procedure. */
#define CS_STATE_PLAN      1U /* Steps of the instance to be planned. */
#define CS_STATE_READY     2U /* Steps planned, waiting for the anchor. */
#define CS_STATE_SCHEDULED 3U /* CS ticker started for the instance. */
#define CS_STATE_STARTED   4U /* LLL radio event prepared. */
#define CS_STATE_DONE      5U /* Instance done, advance to the next one. */

/* Time from the start of the CS radio event (radio timer start) to the
 * anchor of the first step, covering the radio and the step planning setup.
 */
#define CS_LEAD_US 1000U

/* Additional reflector receive window widening covering the clock drift
 * over the offset from the ACL anchor point.
 */
#define CS_WINDOW_MARGIN_US 16U

/* Value used by the host to indicate that no sub mode is configured. */
#define ULL_CS_SUB_MODE_NONE 0xFFU

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
static void cs_proc_chan_sel_init(struct ull_cs_proc *proc, const uint8_t *iv,
				  const uint8_t *in, const uint8_t *pv, uint16_t step_counter,
				  uint8_t chm_repetition);
static void cs_subevent_plan(struct ull_cs_proc *proc);
static void cs_plan(void *param);

static struct ll_cs_local_capabilities local_capabilities = {
	.num_config_supported = CONFIG_BT_CTLR_CHANNEL_SOUNDING_MAX_CONFIG,
	.max_consecutive_procedures_supported = 1,
	.num_antennas_supported = 1,
	.max_antenna_paths_supported = 1,
	.roles_supported = BT_HCI_OP_LE_CS_INITIATOR_ROLE_MASK |
			   BT_HCI_OP_LE_CS_REFLECTOR_ROLE_MASK,
	/* Mode-3 is not supported. */
	.modes_supported = 0U,
	.rtt_capability = BIT(BT_HCI_OP_LE_CS_RTT_TYPE_AA_ONLY),
	.rtt_aa_only_n = 1,
	.rtt_sounding_n = 0,
	.rtt_random_payload_n = 0,
	.nadm_sounding_capability = 0,
	.nadm_random_capability = 0,
	.cs_sync_phys_supported = BIT(BT_HCI_OP_LE_CS_CS_SYNC_1M),
	.subfeatures_supported = 0,
	/* Only the mandatory T_IP1, T_IP2, T_FCS and T_PM times are supported:
	 * 145 us, 145 us, 150 us and 40 us.
	 */
	.t_ip1_times_supported = 0U,
	.t_ip2_times_supported = 0U,
	.t_fcs_times_supported = 0U,
	.t_pm_times_supported = 0U,
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

	proc->plan_mfy_lll._req = 0U;
	proc->plan_mfy_lll._ack = 0U;
	proc->plan_mfy_lll._link = &proc->plan_link_lll;
	proc->plan_mfy_lll.param = proc;
	proc->plan_mfy_lll.fp = cs_plan;

	proc->plan_mfy_ull._req = 0U;
	proc->plan_mfy_ull._ack = 0U;
	proc->plan_mfy_ull._link = &proc->plan_link_ull;
	proc->plan_mfy_ull.param = proc;
	proc->plan_mfy_ull.fp = cs_plan;

	proc->state = CS_STATE_IDLE;
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
	cs_test_state.proc.is_test = 1U;

	return 0;
}

int ull_cs_reset(void)
{
	memset(channel_classification, 0xFF, sizeof(channel_classification));

	for (uint8_t i = 0U; i < ULL_CS_MAX_PROCEDURES; i++) {
		cs_procs[i].state = CS_STATE_IDLE;
		if (cs_procs[i].active) {
			cs_procs[i].active = 0U;
			(void)ticker_stop(TICKER_INSTANCE_ID_CTLR,
					  TICKER_USER_ID_ULL_HIGH,
					  cs_procs[i].ticker_id,
					  cs_ticker_op_stop_cb, &cs_procs[i]);
		}
	}

	cs_test_state.proc.state = CS_STATE_IDLE;
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
		lll->n_ap = 1U;
		lll->subevents_per_event = 1U;
		lll->subevent_interval_us = 0U;
		lll->subevent_len_us = 0U;
		lll->procedure_counter = 0U;
		lll->procedure_count_max = 0U;

		/* TODO: CS Test mode has no timing reference between the two
		 * devices; the reflector receive window is not aligned to the
		 * initiator transmissions.
		 */
		lll->remainder = 0U;
		lll->start_offset_us = CS_LEAD_US;
		lll->window_us = CS_LEAD_US / 2U;
		lll->acl_periph = 0U;
		lll->acl_phy = PHY_1M;
		lll->acl_ww_us = 0U;

		/* CS Test mode uses all-zero CS security vectors to
		 * instantiate the CS DRBG.
		 */
		{
			static const uint8_t zero[ULL_CS_DRBG_IV_SIZE];

			cs_test_state.proc.min_main_mode_steps = lll->main_mode_steps;
			cs_test_state.proc.max_main_mode_steps = lll->main_mode_steps;
			cs_proc_chan_sel_init(&cs_test_state.proc, zero, zero, zero, 0U, 1U);
		}

		cs_test_state.proc.state = CS_STATE_PLAN;
		cs_plan(&cs_test_state.proc);
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
	cs_test_state.proc.state = CS_STATE_IDLE;
	(void)ticker_stop(TICKER_INSTANCE_ID_CTLR, TICKER_USER_ID_THREAD,
			  cs_test_state.proc.ticker_id, cs_ticker_op_stop_cb,
			  &cs_test_state.proc);

	return BT_HCI_ERR_SUCCESS;
}

/* Total number of Channel Sounding RF channel indices (2402..2480 MHz). */
#define ULL_CS_CHAN_COUNT 79U


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

/* Return the next channel of the mode-0 channel selection algorithm #3b
 * list, shuffling the filtered channel list with cr1 whenever the list has
 * been exhausted.
 */
static uint8_t cs_chan_mode_0_next(struct ull_cs_proc *proc)
{
	if (proc->mode_0_index >= proc->filtered_count) {
		proc->mode_0_index = 0U;

		ull_cs_drbg_cr1(&proc->drbg, proc->step_counter, ULL_CS_DRBG_TID_HOP_CHAN_MODE_0,
				proc->filtered, proc->mode_0_channels, proc->filtered_count);
	}

	return proc->mode_0_channels[proc->mode_0_index++];
}

/* Get the next channel of the non-mode-0 channel selection algorithm #3b
 * list. The list shuffled at the start of the CS procedure is used channel
 * map repetition times. Returns false when the list has been used the
 * required number of times, which ends the CS procedure.
 */
static bool cs_chan_non_mode_0_next(struct ull_cs_proc *proc, uint8_t *chan)
{
	if (proc->non_mode_0_index >= proc->filtered_count) {
		if (proc->chm_repetition_count <= 1U) {
			return false;
		}

		proc->chm_repetition_count--;
		proc->non_mode_0_index = 0U;
	}

	*chan = proc->non_mode_0_channels[proc->non_mode_0_index++];

	return true;
}

/* Add a step to the planned subevent. Returns false, without consuming CS
 * DRBG output, if the step does not fit in the subevent or, for a non-mode-0
 * step, the channel map has been used the required number of times.
 */
static bool cs_step_add(struct ull_cs_proc *proc, uint8_t mode)
{
	struct lll_cs *lll = &proc->lll;
	struct lll_cs_step *step;
	uint32_t initiator_aa;
	uint32_t reflector_aa;
	uint32_t duration_us;
	uint8_t chan;

	if (lll->step_count >= LLL_CS_MAX_STEPS) {
		return false;
	}

	duration_us = lll->duration_us + lll_cs_step_len_us(mode, lll->n_ap);
	if (lll->step_count != 0U) {
		duration_us += LLL_CS_T_FCS_US;
	}

	if ((lll->subevent_len_us != 0U) && (duration_us > lll->subevent_len_us)) {
		return false;
	}

	if (mode == LLL_CS_MODE_0) {
		chan = cs_chan_mode_0_next(proc);
	} else if (!cs_chan_non_mode_0_next(proc, &chan)) {
		return false;
	}

	step = &lll->steps[lll->step_count];

	ull_cs_drbg_aa_get(&proc->drbg, proc->step_counter, &initiator_aa, &reflector_aa);
	sys_put_le32(initiator_aa, step->access_address);
	sys_put_le32(reflector_aa, step->access_address_r);

	step->mode = mode;
	step->channel_index = chan;

	lll->duration_us = duration_us;
	lll->step_count++;
	proc->step_counter++;

	return true;
}

/* Plan the steps of a CS procedure into the LLL context. Each CS procedure is
 * executed as a single CS subevent. The plan starts with the mode-0 steps
 * followed by a random number (CS DRBG) of main mode steps, each such run
 * followed by one sub mode step when a sub mode is configured. The procedure
 * ends when the non-mode-0 channel map has been used channel map repetition
 * times, or when the subevent is full. Channels are selected using channel
 * selection algorithm #3b and the CS_SYNC access addresses are generated by
 * the CS DRBG.
 *
 * TODO: CS procedures spanning multiple CS subevents or CS events, and main
 * mode repetition, are not supported.
 */
static void cs_subevent_plan(struct ull_cs_proc *proc)
{
	struct lll_cs *lll = &proc->lll;

	lll->step_count = 0U;
	lll->duration_us = 0U;
	lll->procedure_counter = proc->procedure_counter;
	lll->start_acl_conn_event = proc->instance_event_counter;

	if (proc->filtered_count == 0U) {
		return;
	}

	/* Shuffle the non-mode-0 channel list at the start of the procedure */
	ull_cs_drbg_cr1(&proc->drbg, proc->step_counter, ULL_CS_DRBG_TID_HOP_CHAN_NON_MODE_0,
			proc->filtered, proc->non_mode_0_channels, proc->filtered_count);
	proc->non_mode_0_index = 0U;
	proc->chm_repetition_count = proc->chm_repetition;

	for (uint8_t i = 0U; i < lll->mode_0_steps; i++) {
		if (!cs_step_add(proc, LLL_CS_MODE_0)) {
			return;
		}
	}

	while (true) {
		uint8_t main_steps;

		main_steps = ull_cs_drbg_main_mode_steps_get(&proc->drbg, proc->step_counter,
							     proc->min_main_mode_steps,
							     proc->max_main_mode_steps);
		main_steps = MAX(main_steps, 1U);
		for (uint8_t i = 0U; i < main_steps; i++) {
			if (!cs_step_add(proc, lll->main_mode)) {
				return;
			}
		}

		if (lll->sub_mode == ULL_CS_SUB_MODE_NONE) {
			continue;
		}

		if (!cs_step_add(proc, lll->sub_mode)) {
			return;
		}
	}
}

/* Prepare the CS DRBG and channel selection state of a CS procedure. */
static void cs_proc_chan_sel_init(struct ull_cs_proc *proc, const uint8_t *iv,
				  const uint8_t *in, const uint8_t *pv, uint16_t step_counter,
				  uint8_t chm_repetition)
{
	ull_cs_drbg_init(&proc->drbg, iv, in, pv);

	proc->step_counter = step_counter;
	proc->chm_repetition = MAX(chm_repetition, 1U);
	proc->chm_repetition_count = 0U;
	proc->filtered_count = ull_cs_filtered_channels_get(proc->lll.channel_map,
							    proc->filtered,
							    sizeof(proc->filtered));
	/* Force a shuffle of both lists on first use */
	proc->mode_0_index = proc->filtered_count;
	proc->non_mode_0_index = proc->filtered_count;
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

/* Serialize the step data of a step for the local role into the output
 * buffer. Returns the number of bytes written.
 */
static uint8_t cs_step_data_encode(uint8_t *buf, const struct lll_cs *lll,
				   const struct lll_cs_step *step,
				   const struct lll_cs_step_result *res)
{
	bool initiator = (lll->role == LLL_CS_ROLE_INITIATOR);

	switch (step->mode) {
	case LLL_CS_MODE_0: {
		struct bt_hci_le_cs_step_data_mode_0_initiator *d =
			(struct bt_hci_le_cs_step_data_mode_0_initiator *)buf;

		d->packet_quality_aa_check = res->aa_quality;
		d->packet_quality_bit_errors = 0U;
		d->packet_rssi = (uint8_t)res->rssi;
		d->packet_antenna = 1U;
		if (!initiator) {
			return (uint8_t)sizeof(struct bt_hci_le_cs_step_data_mode_0_reflector);
		}

		d->measured_freq_offset = sys_cpu_to_le16((uint16_t)res->freq_offset);

		return (uint8_t)sizeof(*d);
	}

	case LLL_CS_MODE_1: {
		struct bt_hci_le_cs_step_data_mode_1 *d =
			(struct bt_hci_le_cs_step_data_mode_1 *)buf;

		d->packet_quality_aa_check = res->aa_quality;
		d->packet_quality_bit_errors = 0U;
		d->packet_nadm = 0xFFU; /* Not available */
		d->packet_rssi = (uint8_t)res->rssi;
		/* ToA-ToD on the initiator, ToD-ToA on the reflector; both
		 * share the same field position in the step data.
		 */
		d->toa_tod_initiator = sys_cpu_to_le16((uint16_t)res->time_diff);
		d->packet_antenna = 1U;

		return (uint8_t)sizeof(*d);
	}

	case LLL_CS_MODE_2: {
		struct bt_hci_le_cs_step_data_mode_2 *d =
			(struct bt_hci_le_cs_step_data_mode_2 *)buf;
		uint8_t len = (uint8_t)sizeof(*d);

		d->antenna_permutation_index = 0U;

		/* One tone per antenna path followed by the tone extension
		 * slot, in which no tone is transmitted.
		 */
		for (uint8_t ap = 0U; ap <= lll->n_ap; ap++) {
			struct bt_hci_le_cs_step_data_tone_info *ti =
				(struct bt_hci_le_cs_step_data_tone_info *)&buf[len];
			int16_t i = (ap == 0U) ? res->iq.i : 0;
			int16_t q = (ap == 0U) ? res->iq.q : 0;

			ti->phase_correction_term[0] = (uint8_t)(i & 0xFF);
			ti->phase_correction_term[1] = (uint8_t)(((i >> 8) & 0x0F) |
								 ((q & 0x0F) << 4));
			ti->phase_correction_term[2] = (uint8_t)((q >> 4) & 0xFF);
			if (ap < lll->n_ap) {
				/* TODO: Only the first antenna path is measured */
				ti->quality_indicator = (ap == 0U) ? res->tone_quality :
						       RADIO_CS_TONE_QUALITY_UNAVAILABLE;
				ti->extension_indicator = 0U;
			} else {
				ti->quality_indicator = RADIO_CS_TONE_QUALITY_UNAVAILABLE;
				ti->extension_indicator = 1U;
			}
			len += (uint8_t)sizeof(*ti);
		}

		return len;
	}

	default:
		return 0U;
	}
}

/* Serialize the results of up to NODE_RX_CS_MAX_STEPS steps, starting at
 * step index first, into the node_rx_csound. Returns the number of steps
 * serialized.
 */
static uint8_t cs_subevent_result_build(struct node_rx_csound *cs, const struct lll_cs *lll,
					uint8_t first, uint8_t count)
{
	uint8_t reported = 0U;
	uint16_t offset = 0U;

	for (uint8_t i = first; (i < count) && (reported < NODE_RX_CS_MAX_STEPS); i++) {
		uint8_t buf[3U + sizeof(struct bt_hci_le_cs_step_data_mode_2) +
			    ((LLL_CS_N_AP_MAX + 1U) *
			     sizeof(struct bt_hci_le_cs_step_data_tone_info))];
		uint8_t data_len;

		data_len = cs_step_data_encode(&buf[3], lll, &lll->steps[i], &lll->results[i]);
		if ((offset + 3U + data_len) > NODE_RX_CS_STEP_DATA_MAX) {
			break;
		}

		buf[0] = lll->steps[i].mode;
		buf[1] = lll->steps[i].channel_index;
		buf[2] = data_len;
		memcpy(&cs->step_data[offset], buf, 3U + data_len);

		offset += 3U + data_len;
		reported++;
	}

	cs->num_steps_reported = reported;
	cs->step_data_len = (uint8_t)offset;

	return reported;
}

uint8_t ull_cs_subevent_report(const struct lll_cs *lll, uint8_t step_count,
			       uint8_t abort_reason)
{
	uint8_t nodes;
	uint8_t first;

	/* Steps of a subevent are reported in a LE CS Subevent Result event
	 * followed by LE CS Subevent Result Continue events. All the receive
	 * nodes are allocated upfront from the LLL receive buffer pool; if not
	 * enough buffers are available the subevent results are dropped.
	 */
	nodes = MAX(DIV_ROUND_UP(step_count, NODE_RX_CS_MAX_STEPS), 1U);
	if (!ull_pdu_rx_alloc_peek(nodes)) {
		return 0U;
	}

	first = 0U;
	for (uint8_t n = 0U; n < nodes; n++) {
		struct node_rx_csound *cs;
		struct node_rx_pdu *ntf;
		bool last;

		ntf = ull_pdu_rx_alloc();
		ntf->hdr.type = NODE_RX_TYPE_CS_SUBEVENT_RESULT;
		ntf->hdr.handle = lll->handle;

		cs = (struct node_rx_csound *)ntf->pdu;
		memset(cs, 0, sizeof(*cs));
		cs->status = BT_HCI_ERR_SUCCESS;
		cs->config_id = lll->config_id;
		cs->start_acl_conn_event = lll->start_acl_conn_event;
		cs->procedure_counter = lll->procedure_counter;
		if (lll->role == LLL_CS_ROLE_INITIATOR) {
			cs->frequency_compensation = 0U;
		} else {
			cs->frequency_compensation =
				BT_HCI_LE_CS_SUBEVENT_RESULT_FREQ_COMPENSATION_NOT_AVAILABLE;
		}
		cs->reference_power_level = RADIO_TXP_DEFAULT;
		cs->num_antenna_paths = lll->n_ap;
		cs->continued = (n != 0U) ? 1U : 0U;

		first += cs_subevent_result_build(cs, lll, first, step_count);

		/* Steps that do not fit in the allocated nodes are dropped */
		last = (n == (nodes - 1U));
		if (!last) {
			cs->procedure_done_status = BT_HCI_LE_CS_PROCEDURE_DONE_STATUS_PARTIAL;
			cs->subevent_done_status = BT_HCI_LE_CS_SUBEVENT_DONE_STATUS_PARTIAL;
		} else if (abort_reason != 0U) {
			cs->procedure_done_status = BT_HCI_LE_CS_PROCEDURE_DONE_STATUS_ABORTED;
			cs->subevent_done_status = BT_HCI_LE_CS_SUBEVENT_DONE_STATUS_ABORTED;
			cs->abort_reason = (uint8_t)((abort_reason << 4) |
					   BT_HCI_LE_CS_PROCEDURE_ABORT_REASON_UNSPECIFIED);
		} else {
			cs->procedure_done_status = BT_HCI_LE_CS_PROCEDURE_DONE_STATUS_COMPLETE;
			cs->subevent_done_status = BT_HCI_LE_CS_SUBEVENT_DONE_STATUS_COMPLETE;
		}

		ull_rx_put(ntf->hdr.link, ntf);
	}

	return 1U;
}

/* Prepare the LLL radio event of the planned procedure instance. Runs in the
 * ULL_HIGH execution context.
 */
static void cs_lll_prepare(struct ull_cs_proc *proc, uint32_t ticks_at_expire,
			   uint32_t remainder, uint16_t lazy, uint8_t force)
{
	struct lll_prepare_param *p;
	uint32_t ret;
	uint8_t ref;

	proc->state = CS_STATE_STARTED;

	/* The LLL prepare reference taken here is released by the LLL done
	 * pipeline when the event completes.
	 */
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
}

/* Controller ticker timeout for a CS procedure instance. Runs in the ULL_HIGH
 * execution context. For a connection, the ticker is a single shot ticker
 * started from the anchoring ACL connection event; for the CS Test mode it is
 * a periodic ticker.
 */
static void cs_ticker_cb(uint32_t ticks_at_expire, uint32_t ticks_drift,
			 uint32_t remainder, uint16_t lazy, uint8_t force,
			 void *param)
{
	struct ull_cs_proc *proc = param;

	ARG_UNUSED(ticks_drift);

	if (!proc->active) {
		return;
	}

	if (proc->is_test) {
		if (proc->state != CS_STATE_READY) {
			return;
		}

		proc->lll.remainder = remainder;
	} else if (proc->state != CS_STATE_SCHEDULED) {
		return;
	}

	cs_lll_prepare(proc, ticks_at_expire, remainder, lazy, force);
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

static void cs_ticker_op_one_shot_cb(uint32_t status, void *param)
{
	struct ull_cs_proc *proc = param;

	/* A CS event that could not be scheduled is handled as a missed
	 * procedure instance by the next ACL event hook.
	 */
	ARG_UNUSED(proc);
	ARG_UNUSED(status);
}

static void cs_plan_request(struct ull_cs_proc *proc, uint8_t caller_id)
{
	struct mayfly *mfy;
	uint32_t ret;

	mfy = (caller_id == TICKER_USER_ID_LLL) ? &proc->plan_mfy_lll : &proc->plan_mfy_ull;

	ret = mayfly_enqueue(caller_id, TICKER_USER_ID_ULL_LOW, 1U, mfy);
	LL_ASSERT(!ret || (ret == 1U));
}

/* Advance to the next procedure instance. */
static void cs_instance_next(struct ull_cs_proc *proc)
{
	proc->procedure_counter++;
	proc->instance_event_counter += proc->procedure_interval;
}

/* End of the CS procedures, the configured procedure count was reached. */
static void cs_proc_end(struct ull_cs_proc *proc)
{
	struct ll_conn *conn;

	proc->state = CS_STATE_IDLE;
	proc->active = 0U;

	if (proc->is_test) {
		return;
	}

	/* Preserve the CS step counter so that DRBG output is never reused by
	 * a subsequent CS procedure.
	 */
	conn = ll_connected_get(proc->handle);
	if (conn != NULL) {
		conn->llcp.cs.step_counter = proc->step_counter;
	}
}

/* Plan the steps of the next procedure instance. Runs in the ULL_LOW execution
 * context. Each procedure instance is planned exactly once and in order, so
 * that the CS DRBG output of the local and the peer device stays aligned;
 * instances whose anchoring ACL connection event has already passed are
 * planned and skipped.
 */
static void cs_plan(void *param)
{
	struct ull_cs_proc *proc = param;

	if (!proc->active) {
		return;
	}

	if (proc->state == CS_STATE_DONE) {
		cs_instance_next(proc);
		proc->state = CS_STATE_PLAN;
	}

	if (proc->state != CS_STATE_PLAN) {
		return;
	}

	while (true) {
		if ((proc->procedure_count_max != 0U) &&
		    (proc->procedure_counter >= proc->procedure_count_max)) {
			cs_proc_end(proc);

			return;
		}

		cs_subevent_plan(proc);

		if (proc->is_test ||
		    ((int16_t)(proc->instance_event_counter - proc->acl_event_counter) > 0)) {
			break;
		}

		cs_instance_next(proc);
	}

	proc->state = CS_STATE_READY;
}

void ull_cs_lll_done(struct lll_cs *lll)
{
	struct ull_cs_proc *proc = CONTAINER_OF(lll, struct ull_cs_proc, lll);

	proc->state = CS_STATE_DONE;

	cs_plan_request(proc, TICKER_USER_ID_LLL);
}

static struct ull_cs_proc *cs_proc_get(uint16_t handle)
{
	for (uint8_t i = 0U; i < ULL_CS_MAX_PROCEDURES; i++) {
		if (cs_procs[i].active && (cs_procs[i].handle == handle)) {
			return &cs_procs[i];
		}
	}

	return NULL;
}

void ull_cs_conn_prepare(struct ll_conn *conn, uint32_t ticks_at_expire, uint32_t remainder,
			 uint16_t lazy)
{
	struct ull_cs_proc *proc;
	uint16_t event_counter;
	uint32_t ticks_slot;
	uint32_t ww_us;
	uint32_t ret;
	int16_t diff;

	proc = cs_proc_get(conn->lll.handle);
	if (proc == NULL) {
		return;
	}

	event_counter = ull_conn_event_counter_at_prepare(conn);
	proc->acl_event_counter = event_counter;
	diff = (int16_t)(event_counter - proc->instance_event_counter);

	if (proc->state == CS_STATE_SCHEDULED) {
		/* The CS ticker of the instance did not expire before this
		 * ACL connection event, handle it as missed.
		 */
		if (diff > 0) {
			(void)ticker_stop(TICKER_INSTANCE_ID_CTLR, TICKER_USER_ID_ULL_HIGH,
					  proc->ticker_id, cs_ticker_op_stop_cb, proc);
			proc->state = CS_STATE_DONE;
			cs_plan_request(proc, TICKER_USER_ID_ULL_HIGH);
		}

		return;
	}

	if (proc->state != CS_STATE_READY) {
		return;
	}

	if (diff > 0) {
		/* Missed the anchoring ACL connection event */
		proc->state = CS_STATE_DONE;
		cs_plan_request(proc, TICKER_USER_ID_ULL_HIGH);

		return;
	}

	if (diff < 0) {
		return;
	}

	/* Window widening in this ACL connection event.
	 * TODO: As central, the peer window widening is estimated from the
	 * worst case sleep clock accuracy.
	 */
	ww_us = DIV_ROUND_UP((uint64_t)conn->lll.interval * CONN_INT_UNIT_US *
			     (lll_clock_ppm_local_get() + 500U) * (lazy + 1U),
			     USEC_PER_SEC);
	proc->lll.acl_periph = 0U;

#if defined(CONFIG_BT_PERIPHERAL)
	if (conn->lll.role == BT_HCI_ROLE_PERIPHERAL) {
		ww_us = conn->lll.periph.window_widening_prepare_us +
			(conn->lll.periph.window_widening_periodic_us * (lazy + 1U));
		ww_us = MIN(ww_us, conn->lll.periph.window_widening_max_us);
		proc->lll.acl_periph = 1U;
	}
#endif /* CONFIG_BT_PERIPHERAL */

	proc->lll.acl_ww_us = ww_us;
	proc->lll.window_us = EVENT_JITTER_US + EVENT_TICKER_RES_MARGIN_US + (ww_us << 1) +
			      CS_WINDOW_MARGIN_US;
	proc->lll.remainder = remainder;
	proc->lll.start_offset_us = CS_LEAD_US;
#if defined(CONFIG_BT_CTLR_PHY)
	proc->lll.acl_phy = (conn->lll.role == BT_HCI_ROLE_PERIPHERAL) ? conn->lll.phy_rx :
									  conn->lll.phy_tx;
#else /* !CONFIG_BT_CTLR_PHY */
	proc->lll.acl_phy = PHY_1M;
#endif /* !CONFIG_BT_CTLR_PHY */

	ticks_slot = HAL_TICKER_US_TO_TICKS(EVENT_OVERHEAD_START_US + CS_LEAD_US +
					    proc->lll.window_us + proc->lll.duration_us +
					    EVENT_OVERHEAD_END_US);

	proc->ull.ticks_slot = ticks_slot;
	proc->state = CS_STATE_SCHEDULED;

	ret = ticker_start(TICKER_INSTANCE_ID_CTLR, TICKER_USER_ID_ULL_HIGH, proc->ticker_id,
			   ticks_at_expire, HAL_TICKER_US_TO_TICKS(proc->offset_us - CS_LEAD_US),
			   TICKER_NULL_PERIOD, TICKER_NULL_REMAINDER, TICKER_NULL_LAZY,
			   ticks_slot, cs_ticker_cb, proc, cs_ticker_op_one_shot_cb, proc);
	LL_ASSERT((ret == TICKER_STATUS_SUCCESS) || (ret == TICKER_STATUS_BUSY));
}

void ull_cs_procedure_start(uint16_t handle, uint8_t config_id,
			    uint8_t *access_address)
{
	struct ull_cs_proc *proc = NULL;
	struct ll_conn *conn;
	struct ll_conn_cs_data *cs_data;
	struct lll_cs *lll;
	uint32_t offset_us;

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
		proc->procedure_interval = cs_data->schedule.procedure_interval;
		proc->instance_event_counter = cs_data->schedule.conn_event_count;
		offset_us = sys_get_le24(cs_data->schedule.offset);
	} else {
		proc->event_interval = 1U;
		proc->subevents_per_event = 1U;
		proc->subevent_interval = 0U;
		memcpy(proc->subevent_len, cs_data->max_subevent_len,
		       sizeof(proc->subevent_len));
		proc->procedure_interval = cs_data->max_procedure_interval;
		proc->instance_event_counter = ull_conn_event_counter(conn) + 6U;
		offset_us = 0U;
	}

	if (proc->subevents_per_event == 0U) {
		proc->subevents_per_event = 1U;
	}

	if (proc->procedure_interval == 0U) {
		proc->procedure_interval = 1U;
	}

	/* The CS event is started CS_LEAD_US before the first step, after the
	 * ACL connection event.
	 */
	proc->offset_us = MAX(offset_us, CS_LEAD_US + EVENT_OVERHEAD_START_US);
	proc->procedure_interval_us = (uint32_t)proc->procedure_interval *
				      (uint32_t)conn->lll.interval * CONN_INT_UNIT_US;
	proc->acl_event_counter = ull_conn_event_counter(conn);

	/* Populate the LLL radio event context from the negotiated control
	 * procedure configuration. The CS_SYNC PHY is LE 1M.
	 */
	lll = &proc->lll;
	lll->handle = handle;
	lll->config_id = config_id;
	memcpy(lll->access_address, access_address, sizeof(lll->access_address));
	lll->role = (cs_data->config[config_id].role ==
		     BT_HCI_OP_LE_CS_INITIATOR_ROLE) ?
			    LLL_CS_ROLE_INITIATOR : LLL_CS_ROLE_REFLECTOR;
	lll->phy = PHY_1M;
	lll->main_mode = cs_data->config[config_id].main_mode_type;
	lll->sub_mode = cs_data->config[config_id].sub_mode_type;
	lll->main_mode_steps = cs_data->config[config_id].max_main_mode_steps;
	lll->main_mode_repetition = cs_data->config[config_id].main_mode_repetition;
	lll->mode_0_steps = cs_data->config[config_id].mode_0_steps;
	lll->rtt_type = cs_data->config[config_id].rtt_type;
	memcpy(lll->channel_map, cs_data->config[config_id].channel_map,
	       sizeof(lll->channel_map));
	lll->n_ap = 1U;

	proc->min_main_mode_steps = cs_data->config[config_id].min_main_mode_steps;
	proc->max_main_mode_steps = cs_data->config[config_id].max_main_mode_steps;
	cs_proc_chan_sel_init(proc, cs_data->cs_iv, cs_data->cs_in, cs_data->cs_pv,
			      cs_data->step_counter,
			      cs_data->config[config_id].channel_map_repetition);

	lll->subevents_per_event = proc->subevents_per_event;
	lll->procedure_counter = 0U;
	lll->procedure_count_max = proc->procedure_count_max;

	/* The negotiated subevent length is a 24-bit microsecond value and the
	 * subevent interval is expressed in 0.625 ms units.
	 */
	lll->subevent_len_us = sys_get_le24(proc->subevent_len);
	lll->subevent_interval_us = (uint32_t)proc->subevent_interval * 625U;
	if (lll->subevent_interval_us == 0U) {
		lll->subevent_interval_us = lll->subevent_len_us;
	}

	proc->is_test = 0U;
	proc->active = 1U;
	proc->state = CS_STATE_PLAN;
	cs_plan_request(proc, TICKER_USER_ID_ULL_HIGH);
}

void ull_cs_procedure_stop(uint16_t handle)
{
	struct ull_cs_proc *proc;

	proc = cs_proc_get(handle);
	if (proc == NULL) {
		return;
	}

	cs_proc_end(proc);

	(void)ticker_stop(TICKER_INSTANCE_ID_CTLR, TICKER_USER_ID_ULL_HIGH, proc->ticker_id,
			  cs_ticker_op_stop_cb, proc);
}
