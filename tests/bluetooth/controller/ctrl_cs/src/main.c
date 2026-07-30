/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <zephyr/ztest.h>

#include <zephyr/bluetooth/hci.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/slist.h>
#include <zephyr/sys/util.h>
#include "hal/ccm.h"

#include "util/util.h"
#include "util/mem.h"
#include "util/memq.h"
#include "util/dbuf.h"

#include "pdu_df.h"
#include "lll/pdu_vendor.h"
#include "pdu.h"
#include "ll.h"
#include "ll_settings.h"

#include "lll.h"
#include "lll/lll_df_types.h"
#include "lll_conn.h"
#include "lll_conn_iso.h"

#include "ull_tx_queue.h"

#include "isoal.h"
#include "ull_iso_types.h"
#include "ull_cs_types.h"
#include "ull_conn_iso_types.h"
#include "ull_conn_types.h"
#include "ull_llcp.h"
#include "ull_conn_internal.h"
#include "ull_llcp_internal.h"

#include "helper_pdu.h"
#include "helper_util.h"

extern sys_slist_t ut_rx_q;

static struct ll_conn conn;

/* Track calls to ull_cs_procedure_start/stop for verification */
static uint8_t cs_procedure_started;
static uint8_t cs_procedure_stopped;
static uint16_t cs_start_handle;
static uint8_t cs_start_config_id;
static uint16_t cs_stop_handle;

void ull_cs_procedure_start(uint16_t handle, uint8_t config_id, uint8_t *access_address)
{
	cs_procedure_started++;
	cs_start_handle = handle;
	cs_start_config_id = config_id;
}

void ull_cs_procedure_stop(uint16_t handle)
{
	cs_procedure_stopped++;
	cs_stop_handle = handle;
}

/* Local Channel Sounding capabilities used by the reflector to populate the
 * LL_CS_CAPABILITIES_RSP. A fixed, distinctive set of values is provided so
 * tests can verify the encoded response.
 */
static const struct ll_cs_local_capabilities test_local_capabilities = {
	.num_config_supported = 4,
	.max_consecutive_procedures_supported = 0x0102,
	.num_antennas_supported = 2,
	.max_antenna_paths_supported = 3,
	.roles_supported = BT_HCI_OP_LE_CS_INITIATOR_ROLE_MASK |
			   BT_HCI_OP_LE_CS_REFLECTOR_ROLE_MASK,
	.modes_supported = 0x06,
	.rtt_capability = 0x01,
	.rtt_aa_only_n = 1,
	.rtt_sounding_n = 2,
	.rtt_random_payload_n = 3,
	.nadm_sounding_capability = 0x1234,
	.nadm_random_capability = 0x5678,
	.cs_sync_phys_supported = 0x01,
	.subfeatures_supported = BT_HCI_LE_CS_SUBFEATURE_NO_TX_FAE_MASK |
				 BT_HCI_LE_CS_SUBFEATURE_CHSEL_ALG_3C_MASK,
	.t_ip1_times_supported = 0x0011,
	.t_ip2_times_supported = 0x0022,
	.t_fcs_times_supported = 0x0033,
	.t_pm_times_supported = 0x0044,
	.t_sw_time_supported = 10,
	.tx_snr_capability = 0x05,
};

const struct ll_cs_local_capabilities *ull_cs_local_capabilities_get(void)
{
	return &test_local_capabilities;
}

static void cs_setup(void *data)
{
	test_setup(&conn);
	cs_procedure_started = 0;
	cs_procedure_stopped = 0;
	cs_start_handle = 0xFFFF;
	cs_start_config_id = 0xFF;
	cs_stop_handle = 0xFFFF;
}

/*
 * Helper to verify a Channel Sounding notification from the host RX queue.
 */
static void ut_rx_cs_ntf(uint8_t exp_type, uint8_t exp_status, uint8_t exp_config_id,
			  uint8_t exp_action, uint8_t exp_state)
{
	struct node_rx_pdu *ntf;
	struct node_rx_csound *cs;

	ntf = (struct node_rx_pdu *)sys_slist_get(&ut_rx_q);
	zassert_not_null(ntf, "Expected CS notification but queue is empty");

	zassert_equal(ntf->hdr.type, exp_type,
		      "Notification type mismatch: got %u, expected %u",
		      ntf->hdr.type, exp_type);

	cs = (struct node_rx_csound *)ntf->pdu;
	zassert_equal(cs->status, exp_status,
		      "CS ntf status mismatch: got 0x%02x, expected 0x%02x",
		      cs->status, exp_status);
	zassert_equal(cs->config_id, exp_config_id,
		      "CS ntf config_id mismatch: got %u, expected %u",
		      cs->config_id, exp_config_id);
	zassert_equal(cs->action, exp_action,
		      "CS ntf action mismatch: got %u, expected %u",
		      cs->action, exp_action);
	zassert_equal(cs->state, exp_state,
		      "CS ntf state mismatch: got %u, expected %u",
		      cs->state, exp_state);
}

/*
 * Section 5.1.23 - Channel Sounding Security Start procedure
 *
 * This procedure is local-only (no over-the-air exchange in the current
 * implementation). The controller completes the security enablement locally
 * and notifies the host.
 *
 * +-----+                     +-------+              +-----+
 * | UT  |                     | LL_A  |              | LT  |
 * +-----+                     +-------+              +-----+
 *    |                            |                     |
 *    | Start                      |                     |
 *    | CS Security Enable         |                     |
 *    |--------------------------->|                     |
 *    |                            |                     |
 *    |   CS Security Enable       |                     |
 *    |   Complete (notification)  |                     |
 *    |<---------------------------|                     |
 *    |                            |                     |
 */
ZTEST(cs_central, test_cs_security_enable_central_loc)
{
	uint8_t err;

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_CENTRAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Initiate CS Security Enable */
	err = ull_cp_cs_security_enable(&conn);
	zassert_equal(err, BT_HCI_ERR_SUCCESS);

	/* Prepare */
	event_prepare(&conn);

	/* Done */
	event_done(&conn);

	/* Verify: host notification for security enable complete */
	ut_rx_cs_ntf(NODE_RX_TYPE_CS_SECURITY_ENABLE_COMPLETE,
		     BT_HCI_ERR_SUCCESS, 0U, 0U, 0U);

	/* No further notifications */
	ut_rx_q_is_empty();

	/* No TX PDUs should be sent */
	lt_rx_q_is_empty(&conn);

	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(),
		      "Free CTX buffers %d", llcp_ctx_buffers_free());
}

ZTEST(cs_periph, test_cs_security_enable_periph_loc)
{
	uint8_t err;

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_PERIPHERAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Initiate CS Security Enable */
	err = ull_cp_cs_security_enable(&conn);
	zassert_equal(err, BT_HCI_ERR_SUCCESS);

	/* Prepare */
	event_prepare(&conn);

	/* Done */
	event_done(&conn);

	/* Verify: host notification for security enable complete */
	ut_rx_cs_ntf(NODE_RX_TYPE_CS_SECURITY_ENABLE_COMPLETE,
		     BT_HCI_ERR_SUCCESS, 0U, 0U, 0U);

	/* No further notifications */
	ut_rx_q_is_empty();

	/* No TX PDUs should be sent */
	lt_rx_q_is_empty(&conn);

	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(),
		      "Free CTX buffers %d", llcp_ctx_buffers_free());
}

/*
 * Section 5.1.24 - Channel Sounding Capabilities Exchange procedure
 *
 * Local-only completion (no over-the-air exchange in the current
 * implementation). The controller reads the remote supported capabilities
 * from cached data and notifies the host.
 *
 * +-----+                     +-------+              +-----+
 * | UT  |                     | LL_A  |              | LT  |
 * +-----+                     +-------+              +-----+
 *    |                            |                     |
 *    | Start                      |                     |
 *    | CS Read Remote             |                     |
 *    | Supported Capabilities     |                     |
 *    |--------------------------->|                     |
 *    |                            |                     |
 *    |   CS Read Remote Supported |                     |
 *    |   Capabilities Complete    |                     |
 *    |<---------------------------|                     |
 *    |                            |                     |
 */
ZTEST(cs_central, test_cs_capabilities_exchange_central_loc)
{
	uint8_t err;

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_CENTRAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Initiate CS Read Remote Supported Capabilities */
	err = ull_cp_cs_read_remote_supported_capabilities(&conn);
	zassert_equal(err, BT_HCI_ERR_SUCCESS);

	/* Prepare */
	event_prepare(&conn);

	/* Done */
	event_done(&conn);

	/* Verify: host notification for capabilities read complete */
	ut_rx_cs_ntf(NODE_RX_TYPE_CS_READ_REMOTE_SUPPORTED_CAPABILITIES_COMPLETE,
		     BT_HCI_ERR_SUCCESS, 0U, 0U, 0U);

	/* No further notifications */
	ut_rx_q_is_empty();

	/* No TX PDUs should be sent (local-only) */
	lt_rx_q_is_empty(&conn);

	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(),
		      "Free CTX buffers %d", llcp_ctx_buffers_free());
}

ZTEST(cs_periph, test_cs_capabilities_exchange_periph_loc)
{
	uint8_t err;

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_PERIPHERAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Initiate CS Read Remote Supported Capabilities */
	err = ull_cp_cs_read_remote_supported_capabilities(&conn);
	zassert_equal(err, BT_HCI_ERR_SUCCESS);

	/* Prepare */
	event_prepare(&conn);

	/* Done */
	event_done(&conn);

	/* Verify: host notification for capabilities read complete */
	ut_rx_cs_ntf(NODE_RX_TYPE_CS_READ_REMOTE_SUPPORTED_CAPABILITIES_COMPLETE,
		     BT_HCI_ERR_SUCCESS, 0U, 0U, 0U);

	/* No further notifications */
	ut_rx_q_is_empty();

	/* No TX PDUs should be sent (local-only) */
	lt_rx_q_is_empty(&conn);

	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(),
		      "Free CTX buffers %d", llcp_ctx_buffers_free());
}

/*
 * Section 5.1.25 - Channel Sounding Configuration procedure
 *
 * Local-only completion. The controller creates a configuration locally
 * and notifies the host.
 *
 * +-----+                     +-------+              +-----+
 * | UT  |                     | LL_A  |              | LT  |
 * +-----+                     +-------+              +-----+
 *    |                            |                     |
 *    | Start                      |                     |
 *    | CS Create Config           |                     |
 *    | (config_id=1, action=0)    |                     |
 *    |--------------------------->|                     |
 *    |                            |                     |
 *    |   CS Config Complete       |                     |
 *    |   (config_id=1, action=0)  |                     |
 *    |<---------------------------|                     |
 *    |                            |                     |
 */
ZTEST(cs_central, test_cs_config_central_loc)
{
	uint8_t err;
	const uint8_t config_id = 1U;
	const uint8_t action = 0U;

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_CENTRAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Initiate CS Create Config */
	err = ull_cp_cs_create_config(&conn, config_id, action);
	zassert_equal(err, BT_HCI_ERR_SUCCESS);

	/* Prepare */
	event_prepare(&conn);

	/* Done */
	event_done(&conn);

	/* Verify: host notification for config complete */
	ut_rx_cs_ntf(NODE_RX_TYPE_CS_CONFIG_COMPLETE,
		     BT_HCI_ERR_SUCCESS, config_id, action, 0U);

	/* No further notifications */
	ut_rx_q_is_empty();

	/* No TX PDUs should be sent (local-only) */
	lt_rx_q_is_empty(&conn);

	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(),
		      "Free CTX buffers %d", llcp_ctx_buffers_free());
}

ZTEST(cs_periph, test_cs_config_periph_loc)
{
	uint8_t err;
	const uint8_t config_id = 2U;
	const uint8_t action = 1U;

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_PERIPHERAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Initiate CS Create Config */
	err = ull_cp_cs_create_config(&conn, config_id, action);
	zassert_equal(err, BT_HCI_ERR_SUCCESS);

	/* Prepare */
	event_prepare(&conn);

	/* Done */
	event_done(&conn);

	/* Verify: host notification for config complete with correct params */
	ut_rx_cs_ntf(NODE_RX_TYPE_CS_CONFIG_COMPLETE,
		     BT_HCI_ERR_SUCCESS, config_id, action, 0U);

	/* No further notifications */
	ut_rx_q_is_empty();

	/* No TX PDUs should be sent (local-only) */
	lt_rx_q_is_empty(&conn);

	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(),
		      "Free CTX buffers %d", llcp_ctx_buffers_free());
}

/*
 * Test multiple configuration IDs to validate parameter handling
 */
ZTEST(cs_central, test_cs_config_central_loc_multiple_configs)
{
	uint8_t err;

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_CENTRAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Create first config */
	err = ull_cp_cs_create_config(&conn, 0U, 0U);
	zassert_equal(err, BT_HCI_ERR_SUCCESS);

	/* Prepare/Done for first */
	event_prepare(&conn);
	event_done(&conn);

	ut_rx_cs_ntf(NODE_RX_TYPE_CS_CONFIG_COMPLETE,
		     BT_HCI_ERR_SUCCESS, 0U, 0U, 0U);
	ut_rx_q_is_empty();

	/* Create second config */
	err = ull_cp_cs_create_config(&conn, 3U, 1U);
	zassert_equal(err, BT_HCI_ERR_SUCCESS);

	/* Prepare/Done for second */
	event_prepare(&conn);
	event_done(&conn);

	ut_rx_cs_ntf(NODE_RX_TYPE_CS_CONFIG_COMPLETE,
		     BT_HCI_ERR_SUCCESS, 3U, 1U, 0U);
	ut_rx_q_is_empty();

	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(),
		      "Free CTX buffers %d", llcp_ctx_buffers_free());
}

/*
 * Section 5.1.25 - Channel Sounding Configuration procedure (over-the-air)
 *
 * When the configuration is created with create_context set to
 * LOCAL_AND_REMOTE, the local device exchanges the configuration with the
 * peer using LL_CS_CONFIG_REQ / LL_CS_CONFIG_RSP before notifying the host.
 *
 * +-----+                     +-------+              +-----+
 * | UT  |                     | LL_A  |              | LT  |
 * +-----+                     +-------+              +-----+
 *    |                            |                     |
 *    | Start                      |                     |
 *    | CS Create Config           |                     |
 *    | (config_id=1, action=0)    |                     |
 *    |--------------------------->|                     |
 *    |                            | LL_CS_CONFIG_REQ    |
 *    |                            |-------------------->|
 *    |                            |                     |
 *    |                            |     LL_CS_CONFIG_RSP|
 *    |                            |<--------------------|
 *    |   CS Config Complete       |                     |
 *    |   (config_id=1, action=0)  |                     |
 *    |<---------------------------|                     |
 *    |                            |                     |
 */
ZTEST(cs_central, test_cs_config_central_loc_remote)
{
	uint8_t err;
	struct node_tx *tx;
	const uint8_t config_id = 1U;
	const uint8_t action = 0U;

	struct pdu_data_llctrl_cs_config_req local_cs_config_req = {
		.config_id = config_id,
		.action = action,
	};

	struct pdu_data_llctrl_cs_config_rsp remote_cs_config_rsp = {
		.config_id = config_id,
	};

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_CENTRAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Request exchange of the config with the remote
	 * (create_context == LOCAL_AND_REMOTE).
	 */
	conn.llcp.cs.config[config_id].create_context = 1U;

	/* Initiate CS Create Config */
	err = ull_cp_cs_create_config(&conn, config_id, action);
	zassert_equal(err, BT_HCI_ERR_SUCCESS);

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should have one LL Control PDU: LL_CS_CONFIG_REQ */
	lt_rx(LL_CS_CONFIG_REQ, &conn, &tx, &local_cs_config_req);
	lt_rx_q_is_empty(&conn);

	/* Rx: LL_CS_CONFIG_RSP from remote */
	lt_tx(LL_CS_CONFIG_RSP, &conn, &remote_cs_config_rsp);

	/* Done */
	event_done(&conn);

	/* Release tx node */
	ull_cp_release_tx(&conn, tx);

	/* Verify: host notification for config complete */
	ut_rx_cs_ntf(NODE_RX_TYPE_CS_CONFIG_COMPLETE,
		     BT_HCI_ERR_SUCCESS, config_id, action, 0U);

	/* No further notifications */
	ut_rx_q_is_empty();

	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(),
		      "Free CTX buffers %d", llcp_ctx_buffers_free());
}

ZTEST(cs_periph, test_cs_config_periph_loc_remote)
{
	uint8_t err;
	struct node_tx *tx;
	const uint8_t config_id = 2U;
	const uint8_t action = 1U;

	struct pdu_data_llctrl_cs_config_req local_cs_config_req = {
		.config_id = config_id,
		.action = action,
	};

	struct pdu_data_llctrl_cs_config_rsp remote_cs_config_rsp = {
		.config_id = config_id,
	};

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_PERIPHERAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Request exchange of the config with the remote
	 * (create_context == LOCAL_AND_REMOTE).
	 */
	conn.llcp.cs.config[config_id].create_context = 1U;

	/* Initiate CS Create Config */
	err = ull_cp_cs_create_config(&conn, config_id, action);
	zassert_equal(err, BT_HCI_ERR_SUCCESS);

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should have one LL Control PDU: LL_CS_CONFIG_REQ */
	lt_rx(LL_CS_CONFIG_REQ, &conn, &tx, &local_cs_config_req);
	lt_rx_q_is_empty(&conn);

	/* Rx: LL_CS_CONFIG_RSP from remote */
	lt_tx(LL_CS_CONFIG_RSP, &conn, &remote_cs_config_rsp);

	/* Done */
	event_done(&conn);

	/* Release tx node */
	ull_cp_release_tx(&conn, tx);

	/* Verify: host notification for config complete */
	ut_rx_cs_ntf(NODE_RX_TYPE_CS_CONFIG_COMPLETE,
		     BT_HCI_ERR_SUCCESS, config_id, action, 0U);

	/* No further notifications */
	ut_rx_q_is_empty();

	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(),
		      "Free CTX buffers %d", llcp_ctx_buffers_free());
}

/*
 * Section 5.1.25 - Channel Sounding Configuration procedure (remote)
 *
 * The remote device sends LL_CS_CONFIG_REQ and the local device stores the
 * configuration, responds with LL_CS_CONFIG_RSP and notifies the host.
 *
 * +-----+                     +-------+              +-----+
 * | UT  |                     | LL_A  |              | LT  |
 * +-----+                     +-------+              +-----+
 *    |                            |                     |
 *    |                            |    LL_CS_CONFIG_REQ |
 *    |                            |<--------------------|
 *    |                            |                     |
 *    |                            | LL_CS_CONFIG_RSP    |
 *    |                            |-------------------->|
 *    |   CS Config Complete       |                     |
 *    |<---------------------------|                     |
 *    |                            |                     |
 */
ZTEST(cs_central, test_cs_config_central_rem)
{
	struct node_tx *tx;
	const uint8_t config_id = 2U;
	const uint8_t action = 1U;

	struct pdu_data_llctrl_cs_config_req remote_cs_config_req = {
		.config_id = config_id,
		.action = action,
		.channel_map_repetition = 1U,
		.main_mode = 2U,
		.sub_mode = 1U,
		.main_mode_min_steps = 3U,
		.main_mode_max_steps = 5U,
		.main_mode_repetition = 1U,
		.mode_0_steps = 2U,
		.cs_sync_phy = 1U,
		.rtt_type = 2U,
		.role = BT_HCI_OP_LE_CS_INITIATOR_ROLE,
	};

	struct pdu_data_llctrl_cs_config_rsp local_cs_config_rsp = {
		.config_id = config_id,
	};

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_CENTRAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Prepare */
	event_prepare(&conn);

	/* Rx: LL_CS_CONFIG_REQ from remote */
	lt_tx(LL_CS_CONFIG_REQ, &conn, &remote_cs_config_req);

	/* Done */
	event_done(&conn);

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should have one LL Control PDU: LL_CS_CONFIG_RSP */
	lt_rx(LL_CS_CONFIG_RSP, &conn, &tx, &local_cs_config_rsp);
	lt_rx_q_is_empty(&conn);

	/* TX Ack */
	event_tx_ack(&conn, tx);

	/* Done */
	event_done(&conn);

	/* Release tx node */
	ull_cp_release_tx(&conn, tx);

	/* Verify: host notification for config complete */
	ut_rx_cs_ntf(NODE_RX_TYPE_CS_CONFIG_COMPLETE,
		     BT_HCI_ERR_SUCCESS, config_id, action, 0U);

	/* No further notifications */
	ut_rx_q_is_empty();

	/* Verify the configuration was stored on the reflector */
	zassert_equal(conn.llcp.cs.config[config_id].main_mode_type, 2U,
		      "main_mode_type mismatch");
	zassert_equal(conn.llcp.cs.config[config_id].sub_mode_type, 1U,
		      "sub_mode_type mismatch");
	zassert_equal(conn.llcp.cs.config[config_id].mode_0_steps, 2U,
		      "mode_0_steps mismatch");
	zassert_equal(conn.llcp.cs.config[config_id].rtt_type, 2U,
		      "rtt_type mismatch");
	/* The Initiator sends its own role; the Reflector stores the
	 * complementary role.
	 */
	zassert_equal(conn.llcp.cs.config[config_id].role,
		      BT_HCI_OP_LE_CS_REFLECTOR_ROLE, "role not flipped");

	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(),
		      "Free CTX buffers %d", llcp_ctx_buffers_free());
}

ZTEST(cs_periph, test_cs_config_periph_rem)
{
	struct node_tx *tx;
	const uint8_t config_id = 0U;
	const uint8_t action = 0U;

	struct pdu_data_llctrl_cs_config_req remote_cs_config_req = {
		.config_id = config_id,
		.action = action,
		.main_mode = 1U,
		.sub_mode = 0U,
		.role = BT_HCI_OP_LE_CS_INITIATOR_ROLE,
	};

	struct pdu_data_llctrl_cs_config_rsp local_cs_config_rsp = {
		.config_id = config_id,
	};

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_PERIPHERAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Prepare */
	event_prepare(&conn);

	/* Rx: LL_CS_CONFIG_REQ from remote */
	lt_tx(LL_CS_CONFIG_REQ, &conn, &remote_cs_config_req);

	/* Done */
	event_done(&conn);

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should have one LL Control PDU: LL_CS_CONFIG_RSP */
	lt_rx(LL_CS_CONFIG_RSP, &conn, &tx, &local_cs_config_rsp);
	lt_rx_q_is_empty(&conn);

	/* TX Ack */
	event_tx_ack(&conn, tx);

	/* Done */
	event_done(&conn);

	/* Release tx node */
	ull_cp_release_tx(&conn, tx);

	/* Verify: host notification for config complete */
	ut_rx_cs_ntf(NODE_RX_TYPE_CS_CONFIG_COMPLETE,
		     BT_HCI_ERR_SUCCESS, config_id, action, 0U);

	/* No further notifications */
	ut_rx_q_is_empty();

	/* Verify the configuration was stored on the reflector */
	zassert_equal(conn.llcp.cs.config[config_id].main_mode_type, 1U,
		      "main_mode_type mismatch");
	zassert_equal(conn.llcp.cs.config[config_id].role,
		      BT_HCI_OP_LE_CS_REFLECTOR_ROLE, "role not flipped");

	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(),
		      "Free CTX buffers %d", llcp_ctx_buffers_free());
}

/*
 * Section 5.1.26 - Channel Sounding Start procedure
 *
 * The initiator sends LL_CS_REQ and the reflector responds with LL_CS_RSP.
 * Upon completion, both sides start the CS measurement procedure.
 *
 * +-----+                     +-------+              +-----+
 * | UT  |                     | LL_A  |              | LT  |
 * +-----+                     +-------+              +-----+
 *    |                            |                     |
 *    | Start                      |                     |
 *    | CS Procedure Enable        |                     |
 *    | (config_id=1, enable=1)    |                     |
 *    |--------------------------->|                     |
 *    |                            |                     |
 *    |                            | LL_CS_REQ           |
 *    |                            | (config_id=1)       |
 *    |                            |-------------------->|
 *    |                            |                     |
 *    |                            |          LL_CS_RSP  |
 *    |                            |<--------------------|
 *    |                            |                     |
 *    |   CS Procedure Enable      |                     |
 *    |   Complete (state=1)       |                     |
 *    |<---------------------------|                     |
 *    |                            |                     |
 */
ZTEST(cs_central, test_cs_start_central_loc)
{
	uint8_t err;
	struct node_tx *tx;
	const uint8_t config_id = 1U;

	struct pdu_data_llctrl_cs_req local_cs_req = {
		.config_id = config_id,
		.offset_min = {0, 0, 0},
		.offset_max = {0, 0, 0},
		.conn_event_count = 0U,
	};

	struct pdu_data_llctrl_cs_rsp remote_cs_rsp = {
		.conn_event_count = 0U,
	};

	struct pdu_data_llctrl_cs_ind local_cs_ind = {0};

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_CENTRAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Initiate CS Procedure Enable (start) */
	err = ull_cp_cs_procedure_enable(&conn, config_id, 1U);
	zassert_equal(err, BT_HCI_ERR_SUCCESS);

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should have one LL Control PDU: LL_CS_REQ */
	lt_rx(LL_CS_REQ, &conn, &tx, &local_cs_req);
	lt_rx_q_is_empty(&conn);

	/* Rx: LL_CS_RSP from remote */
	lt_tx(LL_CS_RSP, &conn, &remote_cs_rsp);

	/* Done */
	event_done(&conn);

	/* Release tx node */
	ull_cp_release_tx(&conn, tx);

	/* Prepare - LL_CS_IND should be sent */
	event_prepare(&conn);

	/* Tx Queue should have one LL Control PDU: LL_CS_IND */
	lt_rx(LL_CS_IND, &conn, &tx, &local_cs_ind);
	lt_rx_q_is_empty(&conn);

	/* Done */
	event_done(&conn);

	/* Release tx node */
	ull_cp_release_tx(&conn, tx);

	/* Verify: host notification for procedure enable complete */
	ut_rx_cs_ntf(NODE_RX_TYPE_CS_PROCEDURE_ENABLE_COMPLETE,
		     BT_HCI_ERR_SUCCESS, config_id, 0U, 1U);

	/* No further notifications */
	ut_rx_q_is_empty();

	/* Verify CS procedure was started */
	zassert_equal(cs_procedure_started, 1U, "CS procedure not started");
	zassert_equal(cs_start_config_id, config_id,
		      "CS start config_id mismatch");

	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(),
		      "Free CTX buffers %d", llcp_ctx_buffers_free());
}

/* Helper to read a little-endian 24-bit field. */
static uint32_t cs_test_get_u24(const uint8_t buf[3])
{
	return (uint32_t)buf[0] | ((uint32_t)buf[1] << 8) |
	       ((uint32_t)buf[2] << 16);
}

/*
 * Channel Sounding scheduling parameter negotiation (Initiator side).
 *
 * Verifies that the Initiator encodes the negotiated scheduling parameters
 * derived from the host-provided procedure parameters into the LL_CS_REQ, and
 * that the parameters accepted by the Reflector in the LL_CS_RSP (including the
 * selected offset) are stored and confirmed in the LL_CS_IND.
 */
ZTEST(cs_central, test_cs_start_central_loc_sched_params)
{
	uint8_t err;
	struct node_tx *tx;
	struct pdu_data *pdu;
	const uint8_t config_id = 1U;

	struct pdu_data_llctrl_cs_req local_cs_req = {
		.config_id = config_id,
	};

	/* Reflector accepts a narrower offset window [1000, 2000] us. */
	struct pdu_data_llctrl_cs_rsp remote_cs_rsp = {
		.config_id = config_id,
		.conn_event_count = 0U,
		.offset_min = {0xE8, 0x03, 0x00}, /* 1000 us */
		.offset_max = {0xD0, 0x07, 0x00}, /* 2000 us */
		.event_interval = 1U,
		.subevents_per_event = 1U,
		.subevent_interval = 0U,
		.subevent_len = {0x10, 0x00, 0x00},
		.phy = 1U,
	};

	struct pdu_data_llctrl_cs_ind local_cs_ind = {0};

	/* Host-provided procedure parameters that drive the proposal. */
	conn.llcp.cs.config_id = config_id;
	conn.llcp.cs.max_procedure_len = 0x1234U;
	conn.llcp.cs.min_procedure_interval = 2U;
	conn.llcp.cs.max_procedure_interval = 10U;
	conn.llcp.cs.max_procedure_count = 5U;
	conn.llcp.cs.max_subevent_len[0] = 0x10U;
	conn.llcp.cs.tone_antenna_config_selection = 3U;
	conn.llcp.cs.preferred_peer_antenna = 2U;
	conn.llcp.cs.phy = 1U;

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_CENTRAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Initiate CS Procedure Enable (start) */
	err = ull_cp_cs_procedure_enable(&conn, config_id, 1U);
	zassert_equal(err, BT_HCI_ERR_SUCCESS);

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should have one LL Control PDU: LL_CS_REQ */
	lt_rx(LL_CS_REQ, &conn, &tx, &local_cs_req);
	lt_rx_q_is_empty(&conn);

	/* Verify the negotiated scheduling parameters proposed in LL_CS_REQ */
	pdu = (struct pdu_data *)tx->pdu;
	zassert_equal(pdu->llctrl.cs_req.config_id, config_id);
	zassert_equal(cs_test_get_u24(pdu->llctrl.cs_req.offset_min), 500U,
		      "offset_min mismatch");
	zassert_equal(cs_test_get_u24(pdu->llctrl.cs_req.offset_max), 4000U,
		      "offset_max mismatch");
	zassert_equal(sys_le16_to_cpu(pdu->llctrl.cs_req.max_procedure_len),
		      0x1234U, "max_procedure_len mismatch");
	zassert_equal(sys_le16_to_cpu(pdu->llctrl.cs_req.event_interval), 1U,
		      "event_interval mismatch");
	zassert_equal(pdu->llctrl.cs_req.subevents_per_event, 1U,
		      "subevents_per_event mismatch");
	zassert_equal(cs_test_get_u24(pdu->llctrl.cs_req.subevent_len), 0x10U,
		      "subevent_len mismatch");
	zassert_equal(sys_le16_to_cpu(pdu->llctrl.cs_req.procedure_interval),
		      10U, "procedure_interval mismatch");
	zassert_equal(sys_le16_to_cpu(pdu->llctrl.cs_req.procedure_count), 5U,
		      "procedure_count mismatch");
	zassert_equal(pdu->llctrl.cs_req.aci, 3U, "aci mismatch");
	zassert_equal(pdu->llctrl.cs_req.preferred_peer_ant, 2U,
		      "preferred_peer_ant mismatch");
	zassert_equal(pdu->llctrl.cs_req.phy, 1U, "phy mismatch");

	/* Rx: LL_CS_RSP from remote (Reflector) */
	lt_tx(LL_CS_RSP, &conn, &remote_cs_rsp);

	/* Done */
	event_done(&conn);

	/* Release tx node */
	ull_cp_release_tx(&conn, tx);

	/* Prepare - LL_CS_IND should be sent */
	event_prepare(&conn);

	/* Tx Queue should have one LL Control PDU: LL_CS_IND */
	lt_rx(LL_CS_IND, &conn, &tx, &local_cs_ind);
	lt_rx_q_is_empty(&conn);

	/* Verify the confirmed parameters and selected offset in LL_CS_IND */
	pdu = (struct pdu_data *)tx->pdu;
	zassert_equal(pdu->llctrl.cs_ind.config_id, config_id);
	zassert_equal(cs_test_get_u24(pdu->llctrl.cs_ind.offset), 1000U,
		      "selected offset mismatch");
	zassert_equal(sys_le16_to_cpu(pdu->llctrl.cs_ind.event_interval), 1U,
		      "event_interval mismatch");
	zassert_equal(pdu->llctrl.cs_ind.subevents_per_event, 1U,
		      "subevents_per_event mismatch");
	zassert_equal(pdu->llctrl.cs_ind.phy, 1U, "phy mismatch");

	/* Done */
	event_done(&conn);

	/* Release tx node */
	ull_cp_release_tx(&conn, tx);

	/* Verify: host notification for procedure enable complete */
	ut_rx_cs_ntf(NODE_RX_TYPE_CS_PROCEDURE_ENABLE_COMPLETE,
		     BT_HCI_ERR_SUCCESS, config_id, 0U, 1U);

	/* No further notifications */
	ut_rx_q_is_empty();

	/* Verify the negotiated schedule was stored for the procedure start */
	zassert_equal(conn.llcp.cs.schedule.valid, 1U, "schedule not valid");
	zassert_equal(cs_test_get_u24(conn.llcp.cs.schedule.offset), 1000U,
		      "stored offset mismatch");
	zassert_equal(conn.llcp.cs.schedule.procedure_count, 5U,
		      "stored procedure_count mismatch");

	/* Verify CS procedure was started */
	zassert_equal(cs_procedure_started, 1U, "CS procedure not started");
	zassert_equal(cs_start_config_id, config_id,
		      "CS start config_id mismatch");

	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(),
		      "Free CTX buffers %d", llcp_ctx_buffers_free());
}

ZTEST(cs_periph, test_cs_start_periph_loc)
{
	uint8_t err;
	struct node_tx *tx;
	const uint8_t config_id = 2U;

	struct pdu_data_llctrl_cs_req local_cs_req = {
		.config_id = config_id,
		.offset_min = {0, 0, 0},
		.offset_max = {0, 0, 0},
		.conn_event_count = 0U,
	};

	struct pdu_data_llctrl_cs_rsp remote_cs_rsp = {
		.conn_event_count = 0U,
	};

	struct pdu_data_llctrl_cs_ind local_cs_ind = {0};

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_PERIPHERAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Initiate CS Procedure Enable (start) */
	err = ull_cp_cs_procedure_enable(&conn, config_id, 1U);
	zassert_equal(err, BT_HCI_ERR_SUCCESS);

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should have one LL Control PDU: LL_CS_REQ */
	lt_rx(LL_CS_REQ, &conn, &tx, &local_cs_req);
	lt_rx_q_is_empty(&conn);

	/* Rx: LL_CS_RSP from remote */
	lt_tx(LL_CS_RSP, &conn, &remote_cs_rsp);

	/* Done */
	event_done(&conn);

	/* Release tx node */
	ull_cp_release_tx(&conn, tx);

	/* Prepare - LL_CS_IND should be sent */
	event_prepare(&conn);

	/* Tx Queue should have one LL Control PDU: LL_CS_IND */
	lt_rx(LL_CS_IND, &conn, &tx, &local_cs_ind);
	lt_rx_q_is_empty(&conn);

	/* Done */
	event_done(&conn);

	/* Release tx node */
	ull_cp_release_tx(&conn, tx);

	/* Verify: host notification for procedure enable complete */
	ut_rx_cs_ntf(NODE_RX_TYPE_CS_PROCEDURE_ENABLE_COMPLETE,
		     BT_HCI_ERR_SUCCESS, config_id, 0U, 1U);

	/* No further notifications */
	ut_rx_q_is_empty();

	/* Verify CS procedure was started */
	zassert_equal(cs_procedure_started, 1U, "CS procedure not started");
	zassert_equal(cs_start_config_id, config_id,
		      "CS start config_id mismatch");

	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(),
		      "Free CTX buffers %d", llcp_ctx_buffers_free());
}

/*
 * Section 5.1.26 - Channel Sounding Start procedure (Remote initiated)
 *
 * The remote initiator sends LL_CS_REQ and the local device responds with
 * LL_CS_RSP. Upon TX ACK, the local device notifies the host and starts
 * the CS measurement procedure.
 *
 * +-----+                     +-------+              +-----+
 * | UT  |                     | LL_A  |              | LT  |
 * +-----+                     +-------+              +-----+
 *    |                            |                     |
 *    |                            |          LL_CS_REQ  |
 *    |                            |  (config_id=1)      |
 *    |                            |<--------------------|
 *    |                            |                     |
 *    |                            | LL_CS_RSP           |
 *    |                            |-------------------->|
 *    |                            |                     |
 *    |   CS Procedure Enable      |                     |
 *    |   Complete (state=1)       |                     |
 *    |<---------------------------|                     |
 *    |                            |                     |
 */
ZTEST(cs_central, test_cs_start_central_rem)
{
	struct node_tx *tx;
	const uint8_t config_id = 1U;

	struct pdu_data_llctrl_cs_req remote_cs_req = {
		.config_id = config_id,
		.offset_min = {0, 0, 0},
		.offset_max = {0, 0, 0},
		.conn_event_count = 0U,
	};

	struct pdu_data_llctrl_cs_rsp local_cs_rsp = {
		.conn_event_count = 0U,
	};

	struct pdu_data_llctrl_cs_ind remote_cs_ind = {0};

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_CENTRAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Prepare */
	event_prepare(&conn);

	/* Rx: LL_CS_REQ from remote */
	lt_tx(LL_CS_REQ, &conn, &remote_cs_req);

	/* Done */
	event_done(&conn);

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should have one LL Control PDU: LL_CS_RSP */
	lt_rx(LL_CS_RSP, &conn, &tx, &local_cs_rsp);
	lt_rx_q_is_empty(&conn);

	/* TX Ack */
	event_tx_ack(&conn, tx);

	/* Done */
	event_done(&conn);

	/* Release tx node */
	ull_cp_release_tx(&conn, tx);

	/* Prepare - receive LL_CS_IND from remote (initiator) */
	event_prepare(&conn);

	/* Rx: LL_CS_IND from remote */
	lt_tx(LL_CS_IND, &conn, &remote_cs_ind);

	/* Done */
	event_done(&conn);

	/* Verify: host notification for procedure enable complete */
	ut_rx_cs_ntf(NODE_RX_TYPE_CS_PROCEDURE_ENABLE_COMPLETE,
		     BT_HCI_ERR_SUCCESS, config_id, 0U, 1U);

	/* No further notifications */
	ut_rx_q_is_empty();

	/* Verify CS procedure was started */
	zassert_equal(cs_procedure_started, 1U, "CS procedure not started");
	zassert_equal(cs_start_config_id, config_id,
		      "CS start config_id mismatch");

	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(),
		      "Free CTX buffers %d", llcp_ctx_buffers_free());
}

ZTEST(cs_periph, test_cs_start_periph_rem)
{
	struct node_tx *tx;
	const uint8_t config_id = 3U;

	struct pdu_data_llctrl_cs_req remote_cs_req = {
		.config_id = config_id,
		.offset_min = {0, 0, 0},
		.offset_max = {0, 0, 0},
		.conn_event_count = 0U,
	};

	struct pdu_data_llctrl_cs_rsp local_cs_rsp = {
		.conn_event_count = 0U,
	};

	struct pdu_data_llctrl_cs_ind remote_cs_ind = {0};

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_PERIPHERAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Prepare */
	event_prepare(&conn);

	/* Rx: LL_CS_REQ from remote */
	lt_tx(LL_CS_REQ, &conn, &remote_cs_req);

	/* Done */
	event_done(&conn);

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should have one LL Control PDU: LL_CS_RSP */
	lt_rx(LL_CS_RSP, &conn, &tx, &local_cs_rsp);
	lt_rx_q_is_empty(&conn);

	/* TX Ack */
	event_tx_ack(&conn, tx);

	/* Done */
	event_done(&conn);

	/* Release tx node */
	ull_cp_release_tx(&conn, tx);

	/* Prepare - receive LL_CS_IND from remote (initiator) */
	event_prepare(&conn);

	/* Rx: LL_CS_IND from remote */
	lt_tx(LL_CS_IND, &conn, &remote_cs_ind);

	/* Done */
	event_done(&conn);

	/* Verify: host notification for procedure enable complete */
	ut_rx_cs_ntf(NODE_RX_TYPE_CS_PROCEDURE_ENABLE_COMPLETE,
		     BT_HCI_ERR_SUCCESS, config_id, 0U, 1U);

	/* No further notifications */
	ut_rx_q_is_empty();

	/* Verify CS procedure was started */
	zassert_equal(cs_procedure_started, 1U, "CS procedure not started");
	zassert_equal(cs_start_config_id, config_id,
		      "CS start config_id mismatch");

	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(),
		      "Free CTX buffers %d", llcp_ctx_buffers_free());
}

/*
 * Section 5.1.27 - Channel Sounding Procedure Repeat Termination
 *
 * The initiator sends LL_CS_TERMINATE_REQ and the peer acknowledges with
 * LL_CS_TERMINATE_RSP. Upon receiving the response, the local device
 * notifies the host and stops the CS procedure.
 *
 * +-----+                     +-------+              +-----+
 * | UT  |                     | LL_A  |              | LT  |
 * +-----+                     +-------+              +-----+
 *    |                            |                     |
 *    | Start                      |                     |
 *    | CS Procedure Enable        |                     |
 *    | (config_id=1, enable=0)    |                     |
 *    |--------------------------->|                     |
 *    |                            |                     |
 *    |                            | LL_CS_TERMINATE_REQ |
 *    |                            | (config_id=1,       |
 *    |                            |  error=0x00)        |
 *    |                            |-------------------->|
 *    |                            |                     |
 *    |                            | LL_CS_TERMINATE_RSP |
 *    |                            |<--------------------|
 *    |                            |                     |
 *    |   CS Procedure Enable      |                     |
 *    |   Complete (state=0)       |                     |
 *    |<---------------------------|                     |
 *    |                            |                     |
 */
ZTEST(cs_central, test_cs_terminate_central_loc)
{
	uint8_t err;
	struct node_tx *tx;
	const uint8_t config_id = 1U;

	struct pdu_data_llctrl_cs_terminate_req local_cs_terminate_req = {
		.config_id = config_id,
		.error_code = BT_HCI_ERR_SUCCESS,
	};

	struct pdu_data_llctrl_cs_terminate_rsp remote_cs_terminate_rsp = {
		.config_id = config_id,
		.error_code = BT_HCI_ERR_SUCCESS,
	};

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_CENTRAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Initiate CS Procedure Terminate (disable) */
	err = ull_cp_cs_procedure_enable(&conn, config_id, 0U);
	zassert_equal(err, BT_HCI_ERR_SUCCESS);

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should have one LL Control PDU: LL_CS_TERMINATE_REQ */
	lt_rx(LL_CS_TERMINATE_REQ, &conn, &tx, &local_cs_terminate_req);
	lt_rx_q_is_empty(&conn);

	/* Rx: LL_CS_TERMINATE_RSP from remote */
	lt_tx(LL_CS_TERMINATE_RSP, &conn, &remote_cs_terminate_rsp);

	/* Done */
	event_done(&conn);

	/* Release tx node */
	ull_cp_release_tx(&conn, tx);

	/* Verify: host notification for procedure enable complete (state=0) */
	ut_rx_cs_ntf(NODE_RX_TYPE_CS_PROCEDURE_ENABLE_COMPLETE,
		     BT_HCI_ERR_SUCCESS, config_id, 0U, 0U);

	/* No further notifications */
	ut_rx_q_is_empty();

	/* Verify CS procedure was stopped */
	zassert_equal(cs_procedure_stopped, 1U, "CS procedure not stopped");

	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(),
		      "Free CTX buffers %d", llcp_ctx_buffers_free());
}

ZTEST(cs_periph, test_cs_terminate_periph_loc)
{
	uint8_t err;
	struct node_tx *tx;
	const uint8_t config_id = 2U;

	struct pdu_data_llctrl_cs_terminate_req local_cs_terminate_req = {
		.config_id = config_id,
		.error_code = BT_HCI_ERR_SUCCESS,
	};

	struct pdu_data_llctrl_cs_terminate_rsp remote_cs_terminate_rsp = {
		.config_id = config_id,
		.error_code = BT_HCI_ERR_SUCCESS,
	};

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_PERIPHERAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Initiate CS Procedure Terminate (disable) */
	err = ull_cp_cs_procedure_enable(&conn, config_id, 0U);
	zassert_equal(err, BT_HCI_ERR_SUCCESS);

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should have one LL Control PDU: LL_CS_TERMINATE_REQ */
	lt_rx(LL_CS_TERMINATE_REQ, &conn, &tx, &local_cs_terminate_req);
	lt_rx_q_is_empty(&conn);

	/* Rx: LL_CS_TERMINATE_RSP from remote */
	lt_tx(LL_CS_TERMINATE_RSP, &conn, &remote_cs_terminate_rsp);

	/* Done */
	event_done(&conn);

	/* Release tx node */
	ull_cp_release_tx(&conn, tx);

	/* Verify: host notification for procedure enable complete (state=0) */
	ut_rx_cs_ntf(NODE_RX_TYPE_CS_PROCEDURE_ENABLE_COMPLETE,
		     BT_HCI_ERR_SUCCESS, config_id, 0U, 0U);

	/* No further notifications */
	ut_rx_q_is_empty();

	/* Verify CS procedure was stopped */
	zassert_equal(cs_procedure_stopped, 1U, "CS procedure not stopped");

	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(),
		      "Free CTX buffers %d", llcp_ctx_buffers_free());
}

/*
 * Section 5.1.27 - Channel Sounding Procedure Repeat Termination (Remote)
 *
 * The remote device sends LL_CS_TERMINATE_REQ. The local device responds
 * with LL_CS_TERMINATE_RSP, notifies the host and stops the CS procedure.
 *
 * +-----+                     +-------+              +-----+
 * | UT  |                     | LL_A  |              | LT  |
 * +-----+                     +-------+              +-----+
 *    |                            |                     |
 *    |                            | LL_CS_TERMINATE_REQ |
 *    |                            | (config_id=1,       |
 *    |                            |  error=0x00)        |
 *    |                            |<--------------------|
 *    |                            |                     |
 *    |                            | LL_CS_TERMINATE_RSP |
 *    |                            |-------------------->|
 *    |                            |                     |
 *    |   CS Procedure Enable      |                     |
 *    |   Complete (state=0)       |                     |
 *    |<---------------------------|                     |
 *    |                            |                     |
 */
ZTEST(cs_central, test_cs_terminate_central_rem)
{
	struct node_tx *tx;
	const uint8_t config_id = 1U;

	struct pdu_data_llctrl_cs_terminate_req remote_cs_terminate_req = {
		.config_id = config_id,
		.error_code = BT_HCI_ERR_SUCCESS,
	};

	struct pdu_data_llctrl_cs_terminate_rsp local_cs_terminate_rsp = {
		.config_id = config_id,
		.error_code = BT_HCI_ERR_SUCCESS,
	};

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_CENTRAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Prepare */
	event_prepare(&conn);

	/* Rx: LL_CS_TERMINATE_REQ from remote */
	lt_tx(LL_CS_TERMINATE_REQ, &conn, &remote_cs_terminate_req);

	/* Done */
	event_done(&conn);

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should have one LL Control PDU: LL_CS_TERMINATE_RSP */
	lt_rx(LL_CS_TERMINATE_RSP, &conn, &tx, &local_cs_terminate_rsp);
	lt_rx_q_is_empty(&conn);

	/* TX Ack */
	event_tx_ack(&conn, tx);

	/* Done */
	event_done(&conn);

	/* Release tx node */
	ull_cp_release_tx(&conn, tx);

	/* Verify: host notification for procedure enable complete (state=0) */
	ut_rx_cs_ntf(NODE_RX_TYPE_CS_PROCEDURE_ENABLE_COMPLETE,
		     BT_HCI_ERR_SUCCESS, config_id, 0U, 0U);

	/* No further notifications */
	ut_rx_q_is_empty();

	/* Verify CS procedure was stopped */
	zassert_equal(cs_procedure_stopped, 1U, "CS procedure not stopped");

	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(),
		      "Free CTX buffers %d", llcp_ctx_buffers_free());
}

ZTEST(cs_periph, test_cs_terminate_periph_rem)
{
	struct node_tx *tx;
	const uint8_t config_id = 2U;

	struct pdu_data_llctrl_cs_terminate_req remote_cs_terminate_req = {
		.config_id = config_id,
		.error_code = BT_HCI_ERR_REMOTE_USER_TERM_CONN,
	};

	struct pdu_data_llctrl_cs_terminate_rsp local_cs_terminate_rsp = {
		.config_id = config_id,
		.error_code = BT_HCI_ERR_REMOTE_USER_TERM_CONN,
	};

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_PERIPHERAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Prepare */
	event_prepare(&conn);

	/* Rx: LL_CS_TERMINATE_REQ from remote */
	lt_tx(LL_CS_TERMINATE_REQ, &conn, &remote_cs_terminate_req);

	/* Done */
	event_done(&conn);

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should have one LL Control PDU: LL_CS_TERMINATE_RSP */
	lt_rx(LL_CS_TERMINATE_RSP, &conn, &tx, &local_cs_terminate_rsp);
	lt_rx_q_is_empty(&conn);

	/* TX Ack */
	event_tx_ack(&conn, tx);

	/* Done */
	event_done(&conn);

	/* Release tx node */
	ull_cp_release_tx(&conn, tx);

	/* Verify: host notification for procedure enable complete (state=0) */
	ut_rx_cs_ntf(NODE_RX_TYPE_CS_PROCEDURE_ENABLE_COMPLETE,
		     BT_HCI_ERR_SUCCESS, config_id, 0U, 0U);

	/* No further notifications */
	ut_rx_q_is_empty();

	/* Verify CS procedure was stopped */
	zassert_equal(cs_procedure_stopped, 1U, "CS procedure not stopped");

	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(),
		      "Free CTX buffers %d", llcp_ctx_buffers_free());
}

/*
 * Section 5.1.28 - Channel Sounding Channel Map Update procedure
 *
 * This is handled indirectly through the configuration mechanism.
 * The channel map is part of the CS configuration stored in ll_conn_cs_data.
 * Testing channel map updates is part of the configuration procedure.
 * (No separate LL_CS_CHANNEL_MAP_IND exchange is currently implemented.)
 */

/*
 * Section 5.1.29 - Channel Sounding Mode-0 FAE Table Request procedure
 *
 * The initiator sends LL_CS_FAE_REQ and the reflector responds with
 * LL_CS_FAE_RSP containing the Mode-0 FAE table.
 *
 * +-----+                     +-------+              +-----+
 * | UT  |                     | LL_A  |              | LT  |
 * +-----+                     +-------+              +-----+
 *    |                            |                     |
 *    | Start                      |                     |
 *    | CS Read Remote             |                     |
 *    | FAE Table                  |                     |
 *    |--------------------------->|                     |
 *    |                            |                     |
 *    |                            | LL_CS_FAE_REQ       |
 *    |                            |-------------------->|
 *    |                            |                     |
 *    |                            |      LL_CS_FAE_RSP  |
 *    |                            |  (fae_table[72])    |
 *    |                            |<--------------------|
 *    |                            |                     |
 *    |   CS Read Remote FAE       |                     |
 *    |   Table Complete           |                     |
 *    |<---------------------------|                     |
 *    |                            |                     |
 */
ZTEST(cs_central, test_cs_fae_table_central_loc)
{
	uint8_t err;
	struct node_tx *tx;

	struct pdu_data_llctrl_cs_fae_req local_cs_fae_req = {};

	struct pdu_data_llctrl_cs_fae_rsp remote_cs_fae_rsp;

	/* Fill FAE table with test pattern */
	memset(remote_cs_fae_rsp.ch_fae, 0, sizeof(remote_cs_fae_rsp.ch_fae));
	remote_cs_fae_rsp.ch_fae[0] = 5;
	remote_cs_fae_rsp.ch_fae[1] = -3;
	remote_cs_fae_rsp.ch_fae[71] = 10;

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_CENTRAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Initiate CS Read Remote FAE Table */
	err = ull_cp_cs_read_remote_fae_table(&conn);
	zassert_equal(err, BT_HCI_ERR_SUCCESS);

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should have one LL Control PDU: LL_CS_FAE_REQ */
	lt_rx(LL_CS_FAE_REQ, &conn, &tx, &local_cs_fae_req);
	lt_rx_q_is_empty(&conn);

	/* Rx: LL_CS_FAE_RSP from remote */
	lt_tx(LL_CS_FAE_RSP, &conn, &remote_cs_fae_rsp);

	/* Done */
	event_done(&conn);

	/* Release tx node */
	ull_cp_release_tx(&conn, tx);

	/* Verify: host notification for FAE table read complete */
	ut_rx_cs_ntf(NODE_RX_TYPE_CS_READ_REMOTE_FAE_TABLE_COMPLETE,
		     BT_HCI_ERR_SUCCESS, 0U, 0U, 0U);

	/* No further notifications */
	ut_rx_q_is_empty();

	/* Verify FAE table was stored in connection data */
	zassert_equal(conn.llcp.cs.remote_fae_available, 1U,
		      "Remote FAE table not marked as available");
	zassert_mem_equal(conn.llcp.cs.remote_fae_table,
			  remote_cs_fae_rsp.ch_fae,
			  sizeof(remote_cs_fae_rsp.ch_fae),
			  "FAE table content mismatch");

	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(),
		      "Free CTX buffers %d", llcp_ctx_buffers_free());
}

ZTEST(cs_periph, test_cs_fae_table_periph_loc)
{
	uint8_t err;
	struct node_tx *tx;

	struct pdu_data_llctrl_cs_fae_req local_cs_fae_req = {};

	struct pdu_data_llctrl_cs_fae_rsp remote_cs_fae_rsp;

	/* Fill FAE table with all zeros (no measurements) */
	memset(remote_cs_fae_rsp.ch_fae, 0, sizeof(remote_cs_fae_rsp.ch_fae));

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_PERIPHERAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Initiate CS Read Remote FAE Table */
	err = ull_cp_cs_read_remote_fae_table(&conn);
	zassert_equal(err, BT_HCI_ERR_SUCCESS);

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should have one LL Control PDU: LL_CS_FAE_REQ */
	lt_rx(LL_CS_FAE_REQ, &conn, &tx, &local_cs_fae_req);
	lt_rx_q_is_empty(&conn);

	/* Rx: LL_CS_FAE_RSP from remote */
	lt_tx(LL_CS_FAE_RSP, &conn, &remote_cs_fae_rsp);

	/* Done */
	event_done(&conn);

	/* Release tx node */
	ull_cp_release_tx(&conn, tx);

	/* Verify: host notification for FAE table read complete */
	ut_rx_cs_ntf(NODE_RX_TYPE_CS_READ_REMOTE_FAE_TABLE_COMPLETE,
		     BT_HCI_ERR_SUCCESS, 0U, 0U, 0U);

	/* No further notifications */
	ut_rx_q_is_empty();

	/* Verify FAE table was stored */
	zassert_equal(conn.llcp.cs.remote_fae_available, 1U,
		      "Remote FAE table not marked as available");

	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(),
		      "Free CTX buffers %d", llcp_ctx_buffers_free());
}

/*
 * Section 5.1.29 - Channel Sounding Mode-0 FAE Table Request (Remote)
 *
 * The remote device sends LL_CS_FAE_REQ and the local device responds
 * with LL_CS_FAE_RSP containing its local Mode-0 FAE table.
 *
 * +-----+                     +-------+              +-----+
 * | UT  |                     | LL_A  |              | LT  |
 * +-----+                     +-------+              +-----+
 *    |                            |                     |
 *    |                            |      LL_CS_FAE_REQ  |
 *    |                            |<--------------------|
 *    |                            |                     |
 *    |                            | LL_CS_FAE_RSP       |
 *    |                            | (fae_table[72]=0)   |
 *    |                            |-------------------->|
 *    |                            |                     |
 */
ZTEST(cs_central, test_cs_fae_table_central_rem)
{
	struct node_tx *tx;

	struct pdu_data_llctrl_cs_fae_req remote_cs_fae_req = {};

	struct pdu_data_llctrl_cs_fae_rsp local_cs_fae_rsp;

	/* Expected: all-zero table (no measurements supported yet) */
	memset(local_cs_fae_rsp.ch_fae, 0, sizeof(local_cs_fae_rsp.ch_fae));

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_CENTRAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Prepare */
	event_prepare(&conn);

	/* Rx: LL_CS_FAE_REQ from remote */
	lt_tx(LL_CS_FAE_REQ, &conn, &remote_cs_fae_req);

	/* Done */
	event_done(&conn);

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should have one LL Control PDU: LL_CS_FAE_RSP */
	lt_rx(LL_CS_FAE_RSP, &conn, &tx, &local_cs_fae_rsp);
	lt_rx_q_is_empty(&conn);

	/* TX Ack */
	event_tx_ack(&conn, tx);

	/* Done */
	event_done(&conn);

	/* Release tx node */
	ull_cp_release_tx(&conn, tx);

	/* No host notification expected for reflector side of FAE exchange */
	ut_rx_q_is_empty();

	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(),
		      "Free CTX buffers %d", llcp_ctx_buffers_free());
}

ZTEST(cs_periph, test_cs_fae_table_periph_rem)
{
	struct node_tx *tx;

	struct pdu_data_llctrl_cs_fae_req remote_cs_fae_req = {};

	struct pdu_data_llctrl_cs_fae_rsp local_cs_fae_rsp;

	/* Expected: all-zero table (no measurements supported yet) */
	memset(local_cs_fae_rsp.ch_fae, 0, sizeof(local_cs_fae_rsp.ch_fae));

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_PERIPHERAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Prepare */
	event_prepare(&conn);

	/* Rx: LL_CS_FAE_REQ from remote */
	lt_tx(LL_CS_FAE_REQ, &conn, &remote_cs_fae_req);

	/* Done */
	event_done(&conn);

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should have one LL Control PDU: LL_CS_FAE_RSP */
	lt_rx(LL_CS_FAE_RSP, &conn, &tx, &local_cs_fae_rsp);
	lt_rx_q_is_empty(&conn);

	/* TX Ack */
	event_tx_ack(&conn, tx);

	/* Done */
	event_done(&conn);

	/* Release tx node */
	ull_cp_release_tx(&conn, tx);

	/* No host notification expected for reflector side of FAE exchange */
	ut_rx_q_is_empty();

	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(),
		      "Free CTX buffers %d", llcp_ctx_buffers_free());
}

/*
 * Section 5.1.28 - Channel Sounding Capabilities Exchange (Remote)
 *
 * The remote device sends LL_CS_CAPABILITIES_REQ and the local device
 * responds with LL_CS_CAPABILITIES_RSP populated from its local
 * capabilities. The remote capabilities advertised in the request are
 * cached. No host notification is generated on the responder side.
 *
 * +-----+                     +-------+              +-----+
 * | UT  |                     | LL_A  |              | LT  |
 * +-----+                     +-------+              +-----+
 *    |                            | LL_CS_CAPABILITIES_REQ |
 *    |                            |<-----------------------|
 *    |                            |                        |
 *    |                            | LL_CS_CAPABILITIES_RSP |
 *    |                            |----------------------->|
 *    |                            |                        |
 */
static void cs_capabilities_rsp_from_local(struct pdu_data_llctrl_cs_capabilities_rsp *p)
{
	const struct ll_cs_local_capabilities *cap = ull_cs_local_capabilities_get();

	memset(p, 0, sizeof(*p));
	p->mode_types = cap->modes_supported;
	p->rtt_capability = cap->rtt_capability;
	p->rtt_aa_only_n = cap->rtt_aa_only_n;
	p->rtt_sounding_n = cap->rtt_sounding_n;
	p->rtt_random_sequence_n = cap->rtt_random_payload_n;
	p->nadm_sounding_capability = sys_cpu_to_le16(cap->nadm_sounding_capability);
	p->nadm_random_capability = sys_cpu_to_le16(cap->nadm_random_capability);
	p->cs_sync_phy_capability = cap->cs_sync_phys_supported;
	p->num_ant = cap->num_antennas_supported;
	p->max_ant_path = cap->max_antenna_paths_supported;
	p->role = cap->roles_supported;
	p->no_fae =
		(cap->subfeatures_supported & BT_HCI_LE_CS_SUBFEATURE_NO_TX_FAE_MASK) ? 1U : 0U;
	p->channel_selection_3c =
		(cap->subfeatures_supported & BT_HCI_LE_CS_SUBFEATURE_CHSEL_ALG_3C_MASK) ? 1U : 0U;
	p->sounding_pct_estimate =
		(cap->subfeatures_supported &
		 BT_HCI_LE_CS_SUBFEATURE_PBR_FROM_RTT_SOUNDING_SEQ_MASK) ? 1U : 0U;
	p->num_configs = cap->num_config_supported;
	p->max_procedures_supported = sys_cpu_to_le16(cap->max_consecutive_procedures_supported);
	p->t_sw = cap->t_sw_time_supported;
	p->t_ip1_capability = sys_cpu_to_le16(cap->t_ip1_times_supported);
	p->t_ip2_capability = sys_cpu_to_le16(cap->t_ip2_times_supported);
	p->t_fcs_capability = sys_cpu_to_le16(cap->t_fcs_times_supported);
	p->t_pm_capability = sys_cpu_to_le16(cap->t_pm_times_supported);
	p->tx_snr_capability = cap->tx_snr_capability;
}

static void run_cs_capabilities_rem(uint8_t role)
{
	struct node_tx *tx;

	struct pdu_data_llctrl_cs_capabilities_req remote_cs_capabilities_req = {
		.mode_types = 0x02,
		.rtt_capability = 0x01,
		.num_ant = 1,
		.max_ant_path = 2,
		.role = BT_HCI_OP_LE_CS_INITIATOR_ROLE_MASK,
		.no_fae = 1,
		.num_configs = 2,
		.max_procedures_supported = sys_cpu_to_le16(0x0003),
		.t_sw = 10,
		.t_ip1_capability = sys_cpu_to_le16(0x0010),
	};

	struct pdu_data_llctrl_cs_capabilities_rsp local_cs_capabilities_rsp;

	cs_capabilities_rsp_from_local(&local_cs_capabilities_rsp);

	/* Role */
	test_set_role(&conn, role);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Prepare */
	event_prepare(&conn);

	/* Rx: LL_CS_CAPABILITIES_REQ from remote */
	lt_tx(LL_CS_CAPABILITIES_REQ, &conn, &remote_cs_capabilities_req);

	/* Done */
	event_done(&conn);

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should have one LL Control PDU: LL_CS_CAPABILITIES_RSP */
	lt_rx(LL_CS_CAPABILITIES_RSP, &conn, &tx, &local_cs_capabilities_rsp);
	lt_rx_q_is_empty(&conn);

	/* TX Ack */
	event_tx_ack(&conn, tx);

	/* Done */
	event_done(&conn);

	/* Release tx node */
	ull_cp_release_tx(&conn, tx);

	/* No host notification expected for the responder side */
	ut_rx_q_is_empty();

	/* Verify the remote capabilities advertised in the request were cached */
	zassert_equal(conn.llcp.cs.remote_capabilities_available, 1U,
		      "Remote capabilities not marked as available");
	zassert_equal(conn.llcp.cs.remote_capabilities.modes_supported, 0x02,
		      "Remote modes_supported mismatch");
	zassert_equal(conn.llcp.cs.remote_capabilities.num_antennas_supported, 1,
		      "Remote num_antennas_supported mismatch");
	zassert_equal(conn.llcp.cs.remote_capabilities.max_antenna_paths_supported, 2,
		      "Remote max_antenna_paths_supported mismatch");
	zassert_equal(conn.llcp.cs.remote_capabilities.num_config_supported, 2,
		      "Remote num_config_supported mismatch");
	zassert_equal(conn.llcp.cs.remote_capabilities.subfeatures_supported,
		      BT_HCI_LE_CS_SUBFEATURE_NO_TX_FAE_MASK,
		      "Remote subfeatures_supported mismatch");

	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(),
		      "Free CTX buffers %d", llcp_ctx_buffers_free());
}

ZTEST(cs_central, test_cs_capabilities_central_rem)
{
	run_cs_capabilities_rem(BT_HCI_ROLE_CENTRAL);
}

ZTEST(cs_periph, test_cs_capabilities_periph_rem)
{
	run_cs_capabilities_rem(BT_HCI_ROLE_PERIPHERAL);
}

/*
 * Section 5.1.30 - Channel Sounding Security Start (Remote)
 *
 * The remote device sends LL_CS_SEC_REQ and the local device responds with
 * LL_CS_SEC_RSP. Security material generation is not implemented yet, so the
 * response vectors are all-zero. No host notification is generated on the
 * responder side.
 *
 * +-----+                     +-------+              +-----+
 * | UT  |                     | LL_A  |              | LT  |
 * +-----+                     +-------+              +-----+
 *    |                            |      LL_CS_SEC_REQ  |
 *    |                            |<-------------------|
 *    |                            |                    |
 *    |                            | LL_CS_SEC_RSP      |
 *    |                            |------------------->|
 *    |                            |                    |
 */
static void run_cs_security_rem(uint8_t role)
{
	struct node_tx *tx;

	struct pdu_data_llctrl_cs_sec_req remote_cs_sec_req = {
		.cs_iv_c = {1, 2, 3, 4, 5, 6, 7, 8},
		.cs_in_c = {9, 10, 11, 12},
		.cs_pv_c = {13, 14, 15, 16, 17, 18, 19, 20},
	};

	struct pdu_data_llctrl_cs_sec_rsp local_cs_sec_rsp = {};

	/* Role */
	test_set_role(&conn, role);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Prepare */
	event_prepare(&conn);

	/* Rx: LL_CS_SEC_REQ from remote */
	lt_tx(LL_CS_SEC_REQ, &conn, &remote_cs_sec_req);

	/* Done */
	event_done(&conn);

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should have one LL Control PDU: LL_CS_SEC_RSP */
	lt_rx(LL_CS_SEC_RSP, &conn, &tx, &local_cs_sec_rsp);
	lt_rx_q_is_empty(&conn);

	/* TX Ack */
	event_tx_ack(&conn, tx);

	/* Done */
	event_done(&conn);

	/* Release tx node */
	ull_cp_release_tx(&conn, tx);

	/* No host notification expected for the responder side */
	ut_rx_q_is_empty();

	/* Verify security was marked as enabled */
	zassert_equal(conn.llcp.cs.security_enabled, 1U, "Security not marked as enabled");

	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(),
		      "Free CTX buffers %d", llcp_ctx_buffers_free());
}

ZTEST(cs_central, test_cs_security_central_rem)
{
	run_cs_security_rem(BT_HCI_ROLE_CENTRAL);
}

ZTEST(cs_periph, test_cs_security_periph_rem)
{
	run_cs_security_rem(BT_HCI_ROLE_PERIPHERAL);
}

/*
 * Invalid behavior tests
 */

/*
 * Test: unexpected PDU received during CS FAE procedure (local)
 *
 * +-----+                     +-------+              +-----+
 * | UT  |                     | LL_A  |              | LT  |
 * +-----+                     +-------+              +-----+
 *    |                            |                     |
 *    | Start                      |                     |
 *    | CS Read Remote FAE Table   |                     |
 *    |--------------------------->|                     |
 *    |                            |                     |
 *    |                            | LL_CS_FAE_REQ       |
 *    |                            |-------------------->|
 *    |                            |                     |
 *    |                            | LL_<INVALID>_RSP    |
 *    |                            |<--------------------|
 *    |                            |                     |
 *  ~~~~~~~~~~~~~~~~~ TERMINATE CONNECTION ~~~~~~~~~~~~~~
 *    |                            |                     |
 */
ZTEST(cs_central, test_cs_fae_table_central_loc_invalid_rsp)
{
	uint8_t err;
	struct node_tx *tx;

	struct pdu_data_llctrl_cs_fae_req local_cs_fae_req = {};

	struct pdu_data_llctrl_reject_ext_ind reject_ext_ind = {
		.reject_opcode = PDU_DATA_LLCTRL_TYPE_CS_FAE_REQ,
		.error_code = BT_HCI_ERR_LL_PROC_COLLISION
	};

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_CENTRAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Initiate CS Read Remote FAE Table */
	err = ull_cp_cs_read_remote_fae_table(&conn);
	zassert_equal(err, BT_HCI_ERR_SUCCESS);

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should have one LL Control PDU: LL_CS_FAE_REQ */
	lt_rx(LL_CS_FAE_REQ, &conn, &tx, &local_cs_fae_req);
	lt_rx_q_is_empty(&conn);

	/* Rx: unexpected LL_REJECT_EXT_IND */
	lt_tx(LL_REJECT_EXT_IND, &conn, &reject_ext_ind);

	/* Done */
	event_done(&conn);

	/* Release tx node */
	ull_cp_release_tx(&conn, tx);

	/* Termination 'triggered' due to unexpected PDU */
	zassert_equal(conn.llcp_terminate.reason_final, BT_HCI_ERR_LMP_PDU_NOT_ALLOWED,
		      "Terminate reason %d", conn.llcp_terminate.reason_final);

	/* No host notification expected */
	ut_rx_q_is_empty();

	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(),
		      "Free CTX buffers %d", llcp_ctx_buffers_free());
}

/*
 * Test: LL_CS_REQ rejected by remote with LL_REJECT_EXT_IND (local)
 *
 * Per spec section 6.37, the Peripheral may reject the CS procedure
 * by responding with LL_REJECT_EXT_IND. The controller notifies the
 * host with an error status and does not start CS.
 *
 * +-----+                     +-------+              +-----+
 * | UT  |                     | LL_A  |              | LT  |
 * +-----+                     +-------+              +-----+
 *    |                            |                     |
 *    | Start                      |                     |
 *    | CS Procedure Enable        |                     |
 *    |--------------------------->|                     |
 *    |                            |                     |
 *    |                            | LL_CS_REQ           |
 *    |                            |-------------------->|
 *    |                            |                     |
 *    |                            | LL_REJECT_EXT_IND   |
 *    |                            |<--------------------|
 *    |                            |                     |
 *    |   CS Procedure Enable      |                     |
 *    |   Complete (error)         |                     |
 *    |<---------------------------|                     |
 *    |                            |                     |
 */
ZTEST(cs_central, test_cs_start_central_loc_rejected)
{
	uint8_t err;
	struct node_tx *tx;
	const uint8_t config_id = 1U;

	struct pdu_data_llctrl_cs_req local_cs_req = {
		.config_id = config_id,
		.offset_min = {0, 0, 0},
		.offset_max = {0, 0, 0},
		.conn_event_count = 0U,
	};

	struct pdu_data_llctrl_reject_ext_ind reject_ext_ind = {
		.reject_opcode = PDU_DATA_LLCTRL_TYPE_CS_REQ,
		.error_code = BT_HCI_ERR_UNSUPP_REMOTE_FEATURE
	};

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_CENTRAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Initiate CS Procedure Enable (start) */
	err = ull_cp_cs_procedure_enable(&conn, config_id, 1U);
	zassert_equal(err, BT_HCI_ERR_SUCCESS);

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should have one LL Control PDU: LL_CS_REQ */
	lt_rx(LL_CS_REQ, &conn, &tx, &local_cs_req);
	lt_rx_q_is_empty(&conn);

	/* Rx: LL_REJECT_EXT_IND from remote */
	lt_tx(LL_REJECT_EXT_IND, &conn, &reject_ext_ind);

	/* Done */
	event_done(&conn);

	/* Release tx node */
	ull_cp_release_tx(&conn, tx);

	/* Verify: host notification for procedure enable complete with error */
	ut_rx_cs_ntf(NODE_RX_TYPE_CS_PROCEDURE_ENABLE_COMPLETE,
		     BT_HCI_ERR_UNSUPP_REMOTE_FEATURE,
		     config_id, 0U, 0U);

	/* No further notifications */
	ut_rx_q_is_empty();

	/* CS procedure should NOT have been started */
	zassert_equal(cs_procedure_started, 0U,
		      "CS procedure should not have started on rejection");

	/* Connection should NOT be terminated */
	zassert_equal(conn.llcp_terminate.reason_final, 0U,
		      "Connection should not be terminated on rejection");

	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(),
		      "Free CTX buffers %d", llcp_ctx_buffers_free());
}

/*
 * Test: procedure allocation failure (no ctx buffers)
 */
ZTEST(cs_central, test_cs_procedure_no_ctx_buffers)
{
	uint8_t err;
	struct proc_ctx *ctx;

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_CENTRAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Exhaust all context buffers */
	unsigned int count = 0;

	do {
		ctx = llcp_create_local_procedure(PROC_CS);
		if (ctx) {
			count++;
		}
	} while (ctx != NULL);

	/* Try to start a CS procedure - should fail */
	err = ull_cp_cs_read_remote_fae_table(&conn);
	zassert_equal(err, BT_HCI_ERR_CMD_DISALLOWED,
		      "Expected CMD_DISALLOWED when no ctx buffers available");

	err = ull_cp_cs_security_enable(&conn);
	zassert_equal(err, BT_HCI_ERR_CMD_DISALLOWED,
		      "Expected CMD_DISALLOWED when no ctx buffers available");

	err = ull_cp_cs_procedure_enable(&conn, 0U, 1U);
	zassert_equal(err, BT_HCI_ERR_CMD_DISALLOWED,
		      "Expected CMD_DISALLOWED when no ctx buffers available");

	err = ull_cp_cs_create_config(&conn, 0U, 0U);
	zassert_equal(err, BT_HCI_ERR_CMD_DISALLOWED,
		      "Expected CMD_DISALLOWED when no ctx buffers available");

	err = ull_cp_cs_read_remote_supported_capabilities(&conn);
	zassert_equal(err, BT_HCI_ERR_CMD_DISALLOWED,
		      "Expected CMD_DISALLOWED when no ctx buffers available");
}

/*
 * Test: Full start-then-terminate sequence
 *
 * +-----+                     +-------+              +-----+
 * | UT  |                     | LL_A  |              | LT  |
 * +-----+                     +-------+              +-----+
 *    |                            |                     |
 *    | CS Procedure Enable (1)    |                     |
 *    |--------------------------->|                     |
 *    |                            | LL_CS_REQ           |
 *    |                            |-------------------->|
 *    |                            |          LL_CS_RSP  |
 *    |                            |<--------------------|
 *    |   CS Enable Complete (1)   |                     |
 *    |<---------------------------|                     |
 *    |                            |                     |
 *    | CS Procedure Enable (0)    |                     |
 *    |--------------------------->|                     |
 *    |                            | LL_CS_TERMINATE_REQ |
 *    |                            |-------------------->|
 *    |   CS Enable Complete (0)   |                     |
 *    |<---------------------------|                     |
 *    |                            |                     |
 */
ZTEST(cs_central, test_cs_start_then_terminate_central_loc)
{
	uint8_t err;
	struct node_tx *tx;
	const uint8_t config_id = 1U;

	struct pdu_data_llctrl_cs_req local_cs_req = {
		.config_id = config_id,
		.offset_min = {0, 0, 0},
		.offset_max = {0, 0, 0},
		.conn_event_count = 0U,
	};

	struct pdu_data_llctrl_cs_rsp remote_cs_rsp = {
		.conn_event_count = 0U,
	};

	struct pdu_data_llctrl_cs_terminate_req local_cs_terminate_req = {
		.config_id = config_id,
		.error_code = BT_HCI_ERR_SUCCESS,
	};

	struct pdu_data_llctrl_cs_terminate_rsp remote_cs_terminate_rsp = {
		.config_id = config_id,
		.error_code = BT_HCI_ERR_SUCCESS,
	};

	struct pdu_data_llctrl_cs_ind local_cs_ind = {0};

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_CENTRAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* --- Phase 1: Start CS Procedure --- */

	err = ull_cp_cs_procedure_enable(&conn, config_id, 1U);
	zassert_equal(err, BT_HCI_ERR_SUCCESS);

	event_prepare(&conn);

	lt_rx(LL_CS_REQ, &conn, &tx, &local_cs_req);
	lt_rx_q_is_empty(&conn);

	lt_tx(LL_CS_RSP, &conn, &remote_cs_rsp);

	event_done(&conn);

	ull_cp_release_tx(&conn, tx);

	/* LL_CS_IND should be sent */
	event_prepare(&conn);

	lt_rx(LL_CS_IND, &conn, &tx, &local_cs_ind);
	lt_rx_q_is_empty(&conn);

	event_done(&conn);

	ull_cp_release_tx(&conn, tx);

	ut_rx_cs_ntf(NODE_RX_TYPE_CS_PROCEDURE_ENABLE_COMPLETE,
		     BT_HCI_ERR_SUCCESS, config_id, 0U, 1U);
	ut_rx_q_is_empty();

	zassert_equal(cs_procedure_started, 1U, "CS procedure not started");

	/* --- Phase 2: Terminate CS Procedure --- */

	err = ull_cp_cs_procedure_enable(&conn, config_id, 0U);
	zassert_equal(err, BT_HCI_ERR_SUCCESS);

	event_prepare(&conn);

	lt_rx(LL_CS_TERMINATE_REQ, &conn, &tx, &local_cs_terminate_req);
	lt_rx_q_is_empty(&conn);

	lt_tx(LL_CS_TERMINATE_RSP, &conn, &remote_cs_terminate_rsp);

	event_done(&conn);

	ull_cp_release_tx(&conn, tx);

	ut_rx_cs_ntf(NODE_RX_TYPE_CS_PROCEDURE_ENABLE_COMPLETE,
		     BT_HCI_ERR_SUCCESS, config_id, 0U, 0U);
	ut_rx_q_is_empty();

	zassert_equal(cs_procedure_stopped, 1U, "CS procedure not stopped");

	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(),
		      "Free CTX buffers %d", llcp_ctx_buffers_free());
}

/*
 * Test: multiple FAE table requests validate FAE table content is
 * updated each time
 */
ZTEST(cs_central, test_cs_fae_table_central_loc_update)
{
	uint8_t err;
	struct node_tx *tx;

	struct pdu_data_llctrl_cs_fae_req local_cs_fae_req = {};
	struct pdu_data_llctrl_cs_fae_rsp remote_cs_fae_rsp;

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_CENTRAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* --- First FAE request with pattern A --- */
	memset(remote_cs_fae_rsp.ch_fae, 1, sizeof(remote_cs_fae_rsp.ch_fae));

	err = ull_cp_cs_read_remote_fae_table(&conn);
	zassert_equal(err, BT_HCI_ERR_SUCCESS);

	event_prepare(&conn);
	lt_rx(LL_CS_FAE_REQ, &conn, &tx, &local_cs_fae_req);
	lt_rx_q_is_empty(&conn);
	lt_tx(LL_CS_FAE_RSP, &conn, &remote_cs_fae_rsp);
	event_done(&conn);
	ull_cp_release_tx(&conn, tx);

	ut_rx_cs_ntf(NODE_RX_TYPE_CS_READ_REMOTE_FAE_TABLE_COMPLETE,
		     BT_HCI_ERR_SUCCESS, 0U, 0U, 0U);
	ut_rx_q_is_empty();

	/* Verify first table stored */
	zassert_equal(conn.llcp.cs.remote_fae_table[0], 1,
		      "First FAE table not stored correctly");

	/* --- Second FAE request with pattern B --- */
	memset(remote_cs_fae_rsp.ch_fae, 7, sizeof(remote_cs_fae_rsp.ch_fae));

	err = ull_cp_cs_read_remote_fae_table(&conn);
	zassert_equal(err, BT_HCI_ERR_SUCCESS);

	event_prepare(&conn);
	lt_rx(LL_CS_FAE_REQ, &conn, &tx, &local_cs_fae_req);
	lt_rx_q_is_empty(&conn);
	lt_tx(LL_CS_FAE_RSP, &conn, &remote_cs_fae_rsp);
	event_done(&conn);
	ull_cp_release_tx(&conn, tx);

	ut_rx_cs_ntf(NODE_RX_TYPE_CS_READ_REMOTE_FAE_TABLE_COMPLETE,
		     BT_HCI_ERR_SUCCESS, 0U, 0U, 0U);
	ut_rx_q_is_empty();

	/* Verify second table overwrites first */
	zassert_equal(conn.llcp.cs.remote_fae_table[0], 7,
		      "Second FAE table not stored correctly");
	zassert_equal(conn.llcp.cs.remote_fae_table[71], 7,
		      "Second FAE table not stored correctly");

	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(),
		      "Free CTX buffers %d", llcp_ctx_buffers_free());
}

/*
 * Section 6.38 - Channel Sounding configuration removal during an active
 * CS procedure.
 *
 * When a config is removed while a CS procedure using that config is active,
 * the controller terminates the procedure before removing the config.
 *
 * +-----+                     +-------+              +-----+
 * | UT  |                     | LL_A  |              | LT  |
 * +-----+                     +-------+              +-----+
 *    |                            |                     |
 *    | CS Procedure Enable (1)    |                     |
 *    |--------------------------->|                     |
 *    |                            | LL_CS_REQ           |
 *    |                            |-------------------->|
 *    |                            |          LL_CS_RSP  |
 *    |                            |<--------------------|
 *    |                            | LL_CS_IND           |
 *    |                            |-------------------->|
 *    |   CS Enable Complete (1)   |                     |
 *    |<---------------------------|                     |
 *    |                            |                     |
 *    | CS Remove Config (1)       |                     |
 *    |--------------------------->|                     |
 *    |                            | LL_CS_TERMINATE_REQ |
 *    |                            |-------------------->|
 *    |   CS Enable Complete (0)   |                     |
 *    |<---------------------------|                     |
 *    |   CS Config Complete       |                     |
 *    |   (action=removed)         |                     |
 *    |<---------------------------|                     |
 *    |                            |                     |
 */
ZTEST(cs_central, test_cs_config_removal_during_active_cs)
{
	uint8_t err;
	struct node_tx *tx;
	const uint8_t config_id = 1U;

	struct pdu_data_llctrl_cs_req local_cs_req = {
		.config_id = config_id,
		.offset_min = {0, 0, 0},
		.offset_max = {0, 0, 0},
		.conn_event_count = 0U,
	};

	struct pdu_data_llctrl_cs_rsp remote_cs_rsp = {
		.conn_event_count = 0U,
	};

	struct pdu_data_llctrl_cs_ind local_cs_ind = {0};

	struct pdu_data_llctrl_cs_terminate_req local_cs_terminate_req = {
		.config_id = config_id,
		.error_code = BT_HCI_ERR_SUCCESS,
	};

	struct pdu_data_llctrl_cs_terminate_rsp remote_cs_terminate_rsp = {
		.config_id = config_id,
		.error_code = BT_HCI_ERR_SUCCESS,
	};

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_CENTRAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Setup: mark config as valid and set procedure_enable state */
	conn.llcp.cs.config_id = config_id;
	conn.llcp.cs.procedure_enable = 1U;
	conn.llcp.cs.num_config = 1U;

	/* --- Phase 1: Start CS Procedure --- */

	err = ull_cp_cs_procedure_enable(&conn, config_id, 1U);
	zassert_equal(err, BT_HCI_ERR_SUCCESS);

	event_prepare(&conn);

	lt_rx(LL_CS_REQ, &conn, &tx, &local_cs_req);
	lt_rx_q_is_empty(&conn);

	lt_tx(LL_CS_RSP, &conn, &remote_cs_rsp);

	event_done(&conn);

	ull_cp_release_tx(&conn, tx);

	/* LL_CS_IND */
	event_prepare(&conn);
	lt_rx(LL_CS_IND, &conn, &tx, &local_cs_ind);
	lt_rx_q_is_empty(&conn);
	event_done(&conn);
	ull_cp_release_tx(&conn, tx);

	ut_rx_cs_ntf(NODE_RX_TYPE_CS_PROCEDURE_ENABLE_COMPLETE,
		     BT_HCI_ERR_SUCCESS, config_id, 0U, 1U);
	ut_rx_q_is_empty();

	zassert_equal(cs_procedure_started, 1U, "CS procedure not started");

	/* --- Phase 2: Remove config while CS is active --- */
	/* This should trigger a terminate followed by config removal */

	err = ull_cp_cs_procedure_enable(&conn, config_id, 0U);
	zassert_equal(err, BT_HCI_ERR_SUCCESS);

	event_prepare(&conn);

	lt_rx(LL_CS_TERMINATE_REQ, &conn, &tx, &local_cs_terminate_req);
	lt_rx_q_is_empty(&conn);

	lt_tx(LL_CS_TERMINATE_RSP, &conn, &remote_cs_terminate_rsp);

	event_done(&conn);

	ull_cp_release_tx(&conn, tx);

	/* Verify: procedure terminate notification */
	ut_rx_cs_ntf(NODE_RX_TYPE_CS_PROCEDURE_ENABLE_COMPLETE,
		     BT_HCI_ERR_SUCCESS, config_id, 0U, 0U);

	/* Verify CS procedure was stopped */
	zassert_equal(cs_procedure_stopped, 1U, "CS procedure not stopped");

	/* Now the config removal notification */
	err = ull_cp_cs_create_config(&conn, config_id,
				      BT_HCI_LE_CS_CONFIG_ACTION_REMOVED);
	zassert_equal(err, BT_HCI_ERR_SUCCESS);

	event_prepare(&conn);
	event_done(&conn);

	ut_rx_cs_ntf(NODE_RX_TYPE_CS_CONFIG_COMPLETE,
		     BT_HCI_ERR_SUCCESS, config_id,
		     BT_HCI_LE_CS_CONFIG_ACTION_REMOVED, 0U);
	ut_rx_q_is_empty();

	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(),
		      "Free CTX buffers %d", llcp_ctx_buffers_free());
}

ZTEST_SUITE(cs_central, NULL, NULL, cs_setup, NULL, NULL);
ZTEST_SUITE(cs_periph, NULL, NULL, cs_setup, NULL, NULL);
