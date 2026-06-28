/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>

#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include <zephyr/bluetooth/hci_types.h>

#include "hal/ccm.h"

#include "util/util.h"
#include "util/mem.h"
#include "util/memq.h"
#include "util/dbuf.h"

#include "pdu_df.h"
#include "lll/pdu_vendor.h"
#include "pdu.h"

#include "ll.h"
#include "ll_feat.h"

#include "lll.h"
#include "lll/lll_df_types.h"
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
#include "ull_cs_internal.h"
#include "ull_llcp_internal.h"

#include "ull_llcp.h"

#include "hal/debug.h"

#if defined(CONFIG_BT_CTLR_CHANNEL_SOUNDING)

/* LLCP Local Procedure FSM states */
enum {
	LP_CS_STATE_IDLE = LLCP_STATE_IDLE,
	LP_CS_STATE_WAIT_TX_REQ,
	LP_CS_STATE_WAIT_RX_RSP,
	LP_CS_STATE_WAIT_TX_IND,
	LP_CS_STATE_WAIT_NTF_AVAIL,
};

/* LLCP Local Procedure FSM events */
enum {
	/* Procedure run */
	LP_CS_EVT_RUN,

	/* Response received */
	LP_CS_EVT_RSP,

	/* Rejection received */
	LP_CS_EVT_REJECT,
};

/* LLCP Remote Procedure FSM states */
enum {
	RP_CS_STATE_IDLE = LLCP_STATE_IDLE,
	RP_CS_STATE_WAIT_RX_REQ,
	RP_CS_STATE_WAIT_TX_RSP,
	RP_CS_STATE_WAIT_TX_ACK_RSP,
	RP_CS_STATE_WAIT_RX_IND,
	RP_CS_STATE_WAIT_NTF_AVAIL,
};

/* LLCP Remote Procedure FSM events */
enum {
	/* Procedure run */
	RP_CS_EVT_RUN,

	/* Request received */
	RP_CS_EVT_REQ,

	/* Indication received */
	RP_CS_EVT_IND,
};

/*
 * Scheduling parameter negotiation helpers
 *
 * The 3-octet (24-bit) fields carried in the CS PDUs (offsets and subevent
 * lengths) are encoded little-endian on the air.
 */

static uint32_t cs_get_u24(const uint8_t buf[3])
{
	return (uint32_t)buf[0] | ((uint32_t)buf[1] << 8) |
	       ((uint32_t)buf[2] << 16);
}

static void cs_put_u24(uint8_t buf[3], uint32_t val)
{
	buf[0] = (uint8_t)(val & 0xFFU);
	buf[1] = (uint8_t)((val >> 8) & 0xFFU);
	buf[2] = (uint8_t)((val >> 16) & 0xFFU);
}

/* Default offset window (in microseconds) proposed by the Initiator from the
 * anchoring connection event to the first CS subevent.
 */
#define CS_OFFSET_MIN_US 500U
#define CS_OFFSET_MAX_US 4000U

/* Initiator: build the proposed scheduling parameter set for LL_CS_REQ from
 * the host-provided procedure parameters and the selected configuration.
 */
static void cs_schedule_init_proposal(struct ll_conn *conn, struct proc_ctx *ctx)
{
	struct ll_conn_cs_data *cs_data = &conn->llcp.cs;
	uint16_t procedure_interval;
	uint32_t subevent_len;

	/* Clamp the requested subevent length to the configured maximum. */
	subevent_len = cs_get_u24(cs_data->max_subevent_len);
	if (subevent_len == 0U) {
		subevent_len = cs_get_u24(cs_data->min_subevent_len);
	}

	/* Select a procedure interval within the host-requested range. */
	procedure_interval = cs_data->max_procedure_interval;
	if (procedure_interval == 0U) {
		procedure_interval = cs_data->min_procedure_interval;
	}
	if ((cs_data->min_procedure_interval != 0U) &&
	    (procedure_interval < cs_data->min_procedure_interval)) {
		procedure_interval = cs_data->min_procedure_interval;
	}

	cs_data->schedule.config_id = ctx->data.cs.config_id;
	cs_data->schedule.conn_event_count = conn->lll.event_counter;
	cs_put_u24(cs_data->schedule.offset_min, CS_OFFSET_MIN_US);
	cs_put_u24(cs_data->schedule.offset_max, CS_OFFSET_MAX_US);
	cs_put_u24(cs_data->schedule.offset, CS_OFFSET_MIN_US);
	cs_data->schedule.max_procedure_len = cs_data->max_procedure_len;
	/* A single CS event per connection event, carrying a single subevent. */
	cs_data->schedule.event_interval = 1U;
	cs_data->schedule.subevents_per_event = 1U;
	cs_data->schedule.subevent_interval = 0U;
	cs_put_u24(cs_data->schedule.subevent_len, subevent_len);
	cs_data->schedule.procedure_interval = procedure_interval;
	cs_data->schedule.procedure_count = cs_data->max_procedure_count;
	cs_data->schedule.aci = cs_data->tone_antenna_config_selection;
	cs_data->schedule.preferred_peer_ant = cs_data->preferred_peer_antenna;
	cs_data->schedule.phy = cs_data->phy;
	cs_data->schedule.pwr_delta = (int8_t)cs_data->tx_power_delta;
	cs_data->schedule.tx_snr_i = cs_data->snr_control_initiator & 0x0FU;
	cs_data->schedule.tx_snr_r = cs_data->snr_control_reflector & 0x0FU;
	cs_data->schedule.valid = 1U;
}

/* Reflector: store the scheduling parameters proposed in a received
 * LL_CS_REQ.
 */
static void cs_schedule_store_req(struct ll_conn *conn,
				  const struct pdu_data_llctrl_cs_req *p)
{
	struct ll_conn_cs_data *cs_data = &conn->llcp.cs;

	cs_data->schedule.config_id = p->config_id;
	cs_data->schedule.conn_event_count = sys_le16_to_cpu(p->conn_event_count);
	memcpy(cs_data->schedule.offset_min, p->offset_min,
	       sizeof(cs_data->schedule.offset_min));
	memcpy(cs_data->schedule.offset_max, p->offset_max,
	       sizeof(cs_data->schedule.offset_max));
	cs_data->schedule.max_procedure_len = sys_le16_to_cpu(p->max_procedure_len);
	cs_data->schedule.event_interval = sys_le16_to_cpu(p->event_interval);
	cs_data->schedule.subevents_per_event = p->subevents_per_event;
	cs_data->schedule.subevent_interval = sys_le16_to_cpu(p->subevent_interval);
	memcpy(cs_data->schedule.subevent_len, p->subevent_len,
	       sizeof(cs_data->schedule.subevent_len));
	cs_data->schedule.procedure_interval = sys_le16_to_cpu(p->procedure_interval);
	cs_data->schedule.procedure_count = sys_le16_to_cpu(p->procedure_count);
	cs_data->schedule.aci = p->aci;
	cs_data->schedule.preferred_peer_ant = p->preferred_peer_ant;
	cs_data->schedule.phy = p->phy;
	cs_data->schedule.pwr_delta = p->pwr_delta;
	cs_data->schedule.tx_snr_i = p->tx_snr_i;
	cs_data->schedule.tx_snr_r = p->tx_snr_r;
	cs_data->schedule.valid = 1U;
}

/* Initiator: store the scheduling parameters accepted by the Reflector in a
 * received LL_CS_RSP and select the final offset for LL_CS_IND.
 */
static void cs_schedule_store_rsp(struct ll_conn *conn,
				  const struct pdu_data_llctrl_cs_rsp *p)
{
	struct ll_conn_cs_data *cs_data = &conn->llcp.cs;
	uint32_t offset_min;
	uint32_t offset_max;

	cs_data->schedule.conn_event_count = sys_le16_to_cpu(p->conn_event_count);
	memcpy(cs_data->schedule.offset_min, p->offset_min,
	       sizeof(cs_data->schedule.offset_min));
	memcpy(cs_data->schedule.offset_max, p->offset_max,
	       sizeof(cs_data->schedule.offset_max));
	cs_data->schedule.event_interval = sys_le16_to_cpu(p->event_interval);
	cs_data->schedule.subevents_per_event = p->subevents_per_event;
	cs_data->schedule.subevent_interval = sys_le16_to_cpu(p->subevent_interval);
	memcpy(cs_data->schedule.subevent_len, p->subevent_len,
	       sizeof(cs_data->schedule.subevent_len));
	cs_data->schedule.aci = p->aci;
	cs_data->schedule.phy = p->phy;
	cs_data->schedule.pwr_delta = p->pwr_delta;

	/* Pick the final offset as the start of the Reflector's accepted
	 * window.
	 */
	offset_min = cs_get_u24(p->offset_min);
	offset_max = cs_get_u24(p->offset_max);
	if (offset_max < offset_min) {
		offset_max = offset_min;
	}
	cs_put_u24(cs_data->schedule.offset, offset_min);
	cs_data->schedule.valid = 1U;
}

/* Reflector: store the final scheduling parameters confirmed in a received
 * LL_CS_IND.
 */
static void cs_schedule_store_ind(struct ll_conn *conn,
				  const struct pdu_data_llctrl_cs_ind *p)
{
	struct ll_conn_cs_data *cs_data = &conn->llcp.cs;

	cs_data->schedule.conn_event_count = sys_le16_to_cpu(p->conn_event_count);
	memcpy(cs_data->schedule.offset, p->offset,
	       sizeof(cs_data->schedule.offset));
	cs_data->schedule.event_interval = sys_le16_to_cpu(p->event_interval);
	cs_data->schedule.subevents_per_event = p->subevents_per_event;
	cs_data->schedule.subevent_interval = sys_le16_to_cpu(p->subevent_interval);
	memcpy(cs_data->schedule.subevent_len, p->subevent_len,
	       sizeof(cs_data->schedule.subevent_len));
	cs_data->schedule.aci = p->aci;
	cs_data->schedule.phy = p->phy;
	cs_data->schedule.pwr_delta = p->pwr_delta;
	cs_data->schedule.valid = 1U;
}

/*
 * PDU encode/decode helpers
 */

static void cs_pdu_encode_fae_req(struct pdu_data *pdu)
{
	pdu->ll_id = PDU_DATA_LLID_CTRL;
	pdu->len = PDU_DATA_LLCTRL_LEN(cs_fae_req);
	pdu->llctrl.opcode = PDU_DATA_LLCTRL_TYPE_CS_FAE_REQ;
}

static void cs_pdu_encode_fae_rsp(struct pdu_data *pdu)
{
	struct pdu_data_llctrl_cs_fae_rsp *p = &pdu->llctrl.cs_fae_rsp;

	pdu->ll_id = PDU_DATA_LLID_CTRL;
	pdu->len = PDU_DATA_LLCTRL_LEN(cs_fae_rsp);
	pdu->llctrl.opcode = PDU_DATA_LLCTRL_TYPE_CS_FAE_RSP;

	/* No mode-0 FAE measurements are supported yet, report all-zero table */
	memset(p->ch_fae, 0, sizeof(p->ch_fae));
}

static void cs_pdu_encode_req(struct ll_conn *conn, struct proc_ctx *ctx, struct pdu_data *pdu)
{
	struct pdu_data_llctrl_cs_req *p = &pdu->llctrl.cs_req;
	struct ll_conn_cs_data *cs_data = &conn->llcp.cs;

	/* Build the Initiator's proposed scheduling parameter set. */
	cs_schedule_init_proposal(conn, ctx);

	pdu->ll_id = PDU_DATA_LLID_CTRL;
	pdu->len = PDU_DATA_LLCTRL_LEN(cs_req);
	pdu->llctrl.opcode = PDU_DATA_LLCTRL_TYPE_CS_REQ;
	p->config_id = ctx->data.cs.config_id;
	p->rfu0 = 0U;
	p->conn_event_count = sys_cpu_to_le16(cs_data->schedule.conn_event_count);
	memcpy(p->offset_min, cs_data->schedule.offset_min, sizeof(p->offset_min));
	memcpy(p->offset_max, cs_data->schedule.offset_max, sizeof(p->offset_max));
	p->max_procedure_len = sys_cpu_to_le16(cs_data->schedule.max_procedure_len);
	p->event_interval = sys_cpu_to_le16(cs_data->schedule.event_interval);
	p->subevents_per_event = cs_data->schedule.subevents_per_event;
	p->subevent_interval = sys_cpu_to_le16(cs_data->schedule.subevent_interval);
	memcpy(p->subevent_len, cs_data->schedule.subevent_len, sizeof(p->subevent_len));
	p->procedure_interval = sys_cpu_to_le16(cs_data->schedule.procedure_interval);
	p->procedure_count = sys_cpu_to_le16(cs_data->schedule.procedure_count);
	p->aci = cs_data->schedule.aci;
	p->preferred_peer_ant = cs_data->schedule.preferred_peer_ant;
	p->phy = cs_data->schedule.phy;
	p->pwr_delta = cs_data->schedule.pwr_delta;
	p->tx_snr_i = cs_data->schedule.tx_snr_i;
	p->tx_snr_r = cs_data->schedule.tx_snr_r;
}

static void cs_pdu_encode_rsp(struct ll_conn *conn, struct pdu_data *pdu)
{
	struct pdu_data_llctrl_cs_rsp *p = &pdu->llctrl.cs_rsp;
	struct ll_conn_cs_data *cs_data = &conn->llcp.cs;

	pdu->ll_id = PDU_DATA_LLID_CTRL;
	pdu->len = PDU_DATA_LLCTRL_LEN(cs_rsp);
	pdu->llctrl.opcode = PDU_DATA_LLCTRL_TYPE_CS_RSP;

	/* The Reflector accepts the Initiator's proposed scheduling
	 * parameters and echoes the selected values. The connection event
	 * count is anchored to the Reflector's current event counter.
	 */
	cs_data->schedule.conn_event_count = conn->lll.event_counter;

	p->config_id = cs_data->schedule.config_id;
	p->rfu0 = 0U;
	p->conn_event_count = sys_cpu_to_le16(cs_data->schedule.conn_event_count);
	memcpy(p->offset_min, cs_data->schedule.offset_min, sizeof(p->offset_min));
	memcpy(p->offset_max, cs_data->schedule.offset_max, sizeof(p->offset_max));
	p->event_interval = sys_cpu_to_le16(cs_data->schedule.event_interval);
	p->subevents_per_event = cs_data->schedule.subevents_per_event;
	p->subevent_interval = sys_cpu_to_le16(cs_data->schedule.subevent_interval);
	memcpy(p->subevent_len, cs_data->schedule.subevent_len, sizeof(p->subevent_len));
	p->aci = cs_data->schedule.aci;
	p->phy = cs_data->schedule.phy;
	p->pwr_delta = cs_data->schedule.pwr_delta;
	p->rfu1 = 0U;
}

static void cs_pdu_encode_ind(struct ll_conn *conn, struct proc_ctx *ctx, struct pdu_data *pdu)
{
	struct pdu_data_llctrl_cs_ind *p = &pdu->llctrl.cs_ind;
	struct ll_conn_cs_data *cs_data = &conn->llcp.cs;

	pdu->ll_id = PDU_DATA_LLID_CTRL;
	pdu->len = PDU_DATA_LLCTRL_LEN(cs_ind);
	pdu->llctrl.opcode = PDU_DATA_LLCTRL_TYPE_CS_IND;

	/* The Initiator confirms the negotiated scheduling parameters and the
	 * single chosen offset selected from the Reflector's accepted window.
	 */
	p->config_id = ctx->data.cs.config_id;
	p->rfu0 = 0U;
	p->conn_event_count = sys_cpu_to_le16(cs_data->schedule.conn_event_count);
	memcpy(p->offset, cs_data->schedule.offset, sizeof(p->offset));
	p->event_interval = sys_cpu_to_le16(cs_data->schedule.event_interval);
	p->subevents_per_event = cs_data->schedule.subevents_per_event;
	p->subevent_interval = sys_cpu_to_le16(cs_data->schedule.subevent_interval);
	memcpy(p->subevent_len, cs_data->schedule.subevent_len, sizeof(p->subevent_len));
	p->aci = cs_data->schedule.aci;
	p->phy = cs_data->schedule.phy;
	p->pwr_delta = cs_data->schedule.pwr_delta;
	p->rfu1 = 0U;
}

static void cs_pdu_encode_terminate_req(struct proc_ctx *ctx, struct pdu_data *pdu)
{
	struct pdu_data_llctrl_cs_terminate_req *p = &pdu->llctrl.cs_terminate_req;

	pdu->ll_id = PDU_DATA_LLID_CTRL;
	pdu->len = PDU_DATA_LLCTRL_LEN(cs_terminate_req);
	pdu->llctrl.opcode = PDU_DATA_LLCTRL_TYPE_CS_TERMINATE_REQ;
	p->config_id = ctx->data.cs.config_id;
	p->rfu = 0U;
	p->proc_count = 0U;
	p->error_code = ctx->data.cs.error;
}

static void cs_pdu_encode_terminate_rsp(struct proc_ctx *ctx, struct pdu_data *pdu)
{
	struct pdu_data_llctrl_cs_terminate_rsp *p = &pdu->llctrl.cs_terminate_rsp;

	pdu->ll_id = PDU_DATA_LLID_CTRL;
	pdu->len = PDU_DATA_LLCTRL_LEN(cs_terminate_rsp);
	pdu->llctrl.opcode = PDU_DATA_LLCTRL_TYPE_CS_TERMINATE_RSP;
	p->config_id = ctx->data.cs.config_id;
	p->rfu = 0U;
	p->proc_count = 0U;
	p->error_code = ctx->data.cs.error;
}

/*
 * Notification helpers
 */

static void cs_ntf(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t type,
		   uint8_t status, uint8_t config_id, uint8_t action, uint8_t state)
{
	struct node_rx_pdu *ntf;
	struct node_rx_csound *p;

	ARG_UNUSED(ctx);

	/* Always allocate a dedicated notification node and enqueue it
	 * immediately. This keeps the ordering of multiple notifications
	 * generated by a single procedure deterministic.
	 */
	ntf = llcp_ntf_alloc();
	LL_ASSERT_DBG(ntf);

	ntf->hdr.type = type;
	ntf->hdr.handle = conn->lll.handle;

	p = (struct node_rx_csound *)ntf->pdu;
	p->status = status;
	p->config_id = config_id;
	p->action = action;
	p->state = state;

	ll_rx_put_sched(ntf->hdr.link, ntf);
}

/*
 * LLCP Local Procedure FSM
 */

static void lp_cs_tx(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t opcode)
{
	struct node_tx *tx;
	struct pdu_data *pdu;

	tx = llcp_tx_alloc(conn, ctx);
	LL_ASSERT_DBG(tx);

	pdu = (struct pdu_data *)tx->pdu;

	switch (opcode) {
	case PDU_DATA_LLCTRL_TYPE_CS_FAE_REQ:
		cs_pdu_encode_fae_req(pdu);
		break;
	case PDU_DATA_LLCTRL_TYPE_CS_REQ:
		cs_pdu_encode_req(conn, ctx, pdu);
		break;
	case PDU_DATA_LLCTRL_TYPE_CS_IND:
		cs_pdu_encode_ind(conn, ctx, pdu);
		break;
	case PDU_DATA_LLCTRL_TYPE_CS_TERMINATE_REQ:
		cs_pdu_encode_terminate_req(ctx, pdu);
		break;
	default:
		LL_ASSERT_DBG(0);
		break;
	}

	ctx->tx_opcode = pdu->llctrl.opcode;

	llcp_tx_enqueue(conn, tx);
}

static void lp_cs_complete(struct ll_conn *conn, struct proc_ctx *ctx)
{
	llcp_lr_complete(conn);
	ctx->state = LP_CS_STATE_IDLE;
}

static void lp_cs_send_req(struct ll_conn *conn, struct proc_ctx *ctx)
{
	if (llcp_lr_ispaused(conn) || !llcp_tx_alloc_peek(conn, ctx)) {
		ctx->state = LP_CS_STATE_WAIT_TX_REQ;
		return;
	}

	switch (ctx->data.cs.op) {
	case LLCP_CS_OP_READ_REMOTE_FAE:
		lp_cs_tx(conn, ctx, PDU_DATA_LLCTRL_TYPE_CS_FAE_REQ);
		ctx->rx_opcode = PDU_DATA_LLCTRL_TYPE_CS_FAE_RSP;
		break;
	case LLCP_CS_OP_PROCEDURE_ENABLE:
		if (ctx->data.cs.state) {
			lp_cs_tx(conn, ctx, PDU_DATA_LLCTRL_TYPE_CS_REQ);
			ctx->rx_opcode = PDU_DATA_LLCTRL_TYPE_CS_RSP;
		} else {
			/* LL_CS_TERMINATE_REQ is acknowledged by the peer with
			 * LL_CS_TERMINATE_RSP (BT Core Spec v6.3, Vol 6,
			 * Part B, Section 5.1.27).  Wait for the response
			 * before completing the procedure.
			 */
			lp_cs_tx(conn, ctx, PDU_DATA_LLCTRL_TYPE_CS_TERMINATE_REQ);
			ctx->rx_opcode = PDU_DATA_LLCTRL_TYPE_CS_TERMINATE_RSP;
		}
		break;
	default:
		LL_ASSERT_DBG(0);
		break;
	}

	ctx->state = LP_CS_STATE_WAIT_RX_RSP;
}

static void lp_cs_local_complete(struct ll_conn *conn, struct proc_ctx *ctx)
{
	/* Local-only completions: no over-the-air exchange */
	switch (ctx->data.cs.op) {
	case LLCP_CS_OP_READ_REMOTE_CAP:
		cs_ntf(conn, ctx,
		       NODE_RX_TYPE_CS_READ_REMOTE_SUPPORTED_CAPABILITIES_COMPLETE,
		       BT_HCI_ERR_SUCCESS, 0U, 0U, 0U);
		break;
	case LLCP_CS_OP_CREATE_CONFIG:
		cs_ntf(conn, ctx, NODE_RX_TYPE_CS_CONFIG_COMPLETE,
		       BT_HCI_ERR_SUCCESS, ctx->data.cs.config_id,
		       ctx->data.cs.action, 0U);
		break;
	case LLCP_CS_OP_SECURITY_ENABLE:
		cs_ntf(conn, ctx, NODE_RX_TYPE_CS_SECURITY_ENABLE_COMPLETE,
		       BT_HCI_ERR_SUCCESS, 0U, 0U, 0U);
		break;
	default:
		LL_ASSERT_DBG(0);
		break;
	}

	lp_cs_complete(conn, ctx);
}

static void lp_cs_st_idle(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt, void *param)
{
	switch (evt) {
	case LP_CS_EVT_RUN:
		switch (ctx->data.cs.op) {
		case LLCP_CS_OP_READ_REMOTE_CAP:
		case LLCP_CS_OP_CREATE_CONFIG:
		case LLCP_CS_OP_SECURITY_ENABLE:
			if (!llcp_ntf_alloc_is_available()) {
				/* Wait for a notification node to become available */
				ctx->state = LP_CS_STATE_WAIT_NTF_AVAIL;
				break;
			}
			lp_cs_local_complete(conn, ctx);
			break;
		default:
			lp_cs_send_req(conn, ctx);
			break;
		}
		break;
	default:
		break;
	}
}

static void lp_cs_st_wait_tx_req(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt,
				 void *param)
{
	switch (evt) {
	case LP_CS_EVT_RUN:
		lp_cs_send_req(conn, ctx);
		break;
	default:
		break;
	}
}

static void lp_cs_rx_decode(struct ll_conn *conn, struct proc_ctx *ctx, struct pdu_data *pdu)
{
	struct ll_conn_cs_data *cs_data = &conn->llcp.cs;

	switch (pdu->llctrl.opcode) {
	case PDU_DATA_LLCTRL_TYPE_CS_FAE_RSP:
		memcpy(cs_data->remote_fae_table, pdu->llctrl.cs_fae_rsp.ch_fae,
		       sizeof(cs_data->remote_fae_table));
		cs_data->remote_fae_available = 1U;
		break;
	case PDU_DATA_LLCTRL_TYPE_CS_RSP:
		/* Initiator: store the scheduling parameters accepted by the
		 * Reflector and select the final offset for LL_CS_IND.
		 */
		cs_schedule_store_rsp(conn, &pdu->llctrl.cs_rsp);
		break;
	default:
		break;
	}
}

static void lp_cs_send_ind(struct ll_conn *conn, struct proc_ctx *ctx)
{
	if (llcp_lr_ispaused(conn) || !llcp_tx_alloc_peek(conn, ctx)) {
		ctx->state = LP_CS_STATE_WAIT_TX_IND;
		return;
	}

	lp_cs_tx(conn, ctx, PDU_DATA_LLCTRL_TYPE_CS_IND);

	/* LL_CS_IND is a one-way PDU; complete the procedure after
	 * transmission and notify the host.
	 */
	if (!llcp_ntf_alloc_is_available()) {
		ctx->state = LP_CS_STATE_WAIT_NTF_AVAIL;
		return;
	}

	cs_ntf(conn, ctx, NODE_RX_TYPE_CS_PROCEDURE_ENABLE_COMPLETE,
	       BT_HCI_ERR_SUCCESS, ctx->data.cs.config_id, 0U,
	       ctx->data.cs.state);
	ull_cs_procedure_start(conn->lll.handle, ctx->data.cs.config_id,
			       conn->lll.access_addr);
	lp_cs_complete(conn, ctx);
}

static void lp_cs_complete_rsp(struct ll_conn *conn, struct proc_ctx *ctx)
{
	switch (ctx->data.cs.op) {
	case LLCP_CS_OP_READ_REMOTE_FAE:
		cs_ntf(conn, ctx, NODE_RX_TYPE_CS_READ_REMOTE_FAE_TABLE_COMPLETE,
		       BT_HCI_ERR_SUCCESS, 0U, 0U, 0U);
		lp_cs_complete(conn, ctx);
		break;
	case LLCP_CS_OP_PROCEDURE_ENABLE:
		if (ctx->data.cs.state) {
			/* After receiving LL_CS_RSP, send LL_CS_IND per spec
			 * section 6.35/6.36 before starting the CS procedure.
			 */
			lp_cs_send_ind(conn, ctx);
			return;
		}
		cs_ntf(conn, ctx, NODE_RX_TYPE_CS_PROCEDURE_ENABLE_COMPLETE,
		       BT_HCI_ERR_SUCCESS, ctx->data.cs.config_id, 0U,
		       ctx->data.cs.state);
		ull_cs_procedure_stop(conn->lll.handle);
		lp_cs_complete(conn, ctx);
		break;
	default:
		LL_ASSERT_DBG(0);
		break;
	}
}

static void lp_cs_st_wait_rx_rsp(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt,
				 void *param)
{
	switch (evt) {
	case LP_CS_EVT_RSP:
		lp_cs_rx_decode(conn, ctx, (struct pdu_data *)param);

		if (!llcp_ntf_alloc_is_available()) {
			ctx->state = LP_CS_STATE_WAIT_NTF_AVAIL;
			break;
		}

		lp_cs_complete_rsp(conn, ctx);
		break;
	case LP_CS_EVT_REJECT:
		if (ctx->data.cs.op == LLCP_CS_OP_PROCEDURE_ENABLE) {
			/* LL_REJECT_EXT_IND received in response to LL_CS_REQ.
			 * Per spec section 6.37, notify host of rejection and
			 * complete the procedure without starting CS.
			 */
			ctx->data.cs.error = ctx->reject_ext_ind.error_code;
			ctx->data.cs.state = 0U;
			if (!llcp_ntf_alloc_is_available()) {
				ctx->state = LP_CS_STATE_WAIT_NTF_AVAIL;
				break;
			}
			cs_ntf(conn, ctx,
			       NODE_RX_TYPE_CS_PROCEDURE_ENABLE_COMPLETE,
			       ctx->data.cs.error,
			       ctx->data.cs.config_id, 0U, 0U);
			lp_cs_complete(conn, ctx);
		} else {
			/* Rejection not expected for other CS operations;
			 * terminate connection.
			 */
			conn->llcp_terminate.reason_final =
				BT_HCI_ERR_LMP_PDU_NOT_ALLOWED;
			llcp_lr_complete(conn);
			ctx->state = LP_CS_STATE_IDLE;
		}
		break;
	default:
		break;
	}
}

static void lp_cs_st_wait_tx_ind(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt,
				 void *param)
{
	switch (evt) {
	case LP_CS_EVT_RUN:
		lp_cs_send_ind(conn, ctx);
		break;
	default:
		break;
	}
}

static void lp_cs_st_wait_ntf_avail(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt,
				    void *param)
{
	switch (evt) {
	case LP_CS_EVT_RUN:
		if (!llcp_ntf_alloc_is_available()) {
			break;
		}

		switch (ctx->data.cs.op) {
		case LLCP_CS_OP_READ_REMOTE_CAP:
		case LLCP_CS_OP_CREATE_CONFIG:
		case LLCP_CS_OP_SECURITY_ENABLE:
			lp_cs_local_complete(conn, ctx);
			break;
		case LLCP_CS_OP_PROCEDURE_ENABLE:
			if (ctx->tx_opcode == PDU_DATA_LLCTRL_TYPE_CS_IND) {
				/* LL_CS_IND already sent, complete now */
				cs_ntf(conn, ctx,
				       NODE_RX_TYPE_CS_PROCEDURE_ENABLE_COMPLETE,
				       BT_HCI_ERR_SUCCESS,
				       ctx->data.cs.config_id, 0U,
				       ctx->data.cs.state);
				ull_cs_procedure_start(conn->lll.handle,
						       ctx->data.cs.config_id,
						       conn->lll.access_addr);
				lp_cs_complete(conn, ctx);
			} else if (ctx->tx_opcode ==
				   PDU_DATA_LLCTRL_TYPE_CS_TERMINATE_REQ) {
				/* Terminate path: IND not involved */
				cs_ntf(conn, ctx,
				       NODE_RX_TYPE_CS_PROCEDURE_ENABLE_COMPLETE,
				       BT_HCI_ERR_SUCCESS,
				       ctx->data.cs.config_id, 0U, 0U);
				ull_cs_procedure_stop(conn->lll.handle);
				lp_cs_complete(conn, ctx);
			} else if (ctx->data.cs.error) {
				/* Rejected by remote, complete with error */
				cs_ntf(conn, ctx,
				       NODE_RX_TYPE_CS_PROCEDURE_ENABLE_COMPLETE,
				       ctx->data.cs.error,
				       ctx->data.cs.config_id, 0U, 0U);
				lp_cs_complete(conn, ctx);
			} else {
				/* LL_CS_RSP received, proceed to send IND */
				lp_cs_complete_rsp(conn, ctx);
			}
			break;
		default:
			lp_cs_complete_rsp(conn, ctx);
			break;
		}
		break;
	default:
		break;
	}
}

static void lp_cs_execute_fsm(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt, void *param)
{
	switch (ctx->state) {
	case LP_CS_STATE_IDLE:
		lp_cs_st_idle(conn, ctx, evt, param);
		break;
	case LP_CS_STATE_WAIT_TX_REQ:
		lp_cs_st_wait_tx_req(conn, ctx, evt, param);
		break;
	case LP_CS_STATE_WAIT_RX_RSP:
		lp_cs_st_wait_rx_rsp(conn, ctx, evt, param);
		break;
	case LP_CS_STATE_WAIT_TX_IND:
		lp_cs_st_wait_tx_ind(conn, ctx, evt, param);
		break;
	case LP_CS_STATE_WAIT_NTF_AVAIL:
		lp_cs_st_wait_ntf_avail(conn, ctx, evt, param);
		break;
	default:
		LL_ASSERT_DBG(0);
		break;
	}
}

void llcp_lp_cs_init_proc(struct proc_ctx *ctx)
{
	ctx->state = LP_CS_STATE_IDLE;
}

void llcp_lp_cs_run(struct ll_conn *conn, struct proc_ctx *ctx, void *param)
{
	lp_cs_execute_fsm(conn, ctx, LP_CS_EVT_RUN, param);
}

void llcp_lp_cs_rx(struct ll_conn *conn, struct proc_ctx *ctx, struct node_rx_pdu *rx)
{
	struct pdu_data *pdu = (struct pdu_data *)rx->pdu;

	switch (pdu->llctrl.opcode) {
	case PDU_DATA_LLCTRL_TYPE_CS_FAE_RSP:
	case PDU_DATA_LLCTRL_TYPE_CS_RSP:
	case PDU_DATA_LLCTRL_TYPE_CS_TERMINATE_RSP:
		lp_cs_execute_fsm(conn, ctx, LP_CS_EVT_RSP, pdu);
		break;
	case PDU_DATA_LLCTRL_TYPE_REJECT_EXT_IND:
		ctx->reject_ext_ind.reject_opcode =
			pdu->llctrl.reject_ext_ind.reject_opcode;
		ctx->reject_ext_ind.error_code =
			pdu->llctrl.reject_ext_ind.error_code;
		lp_cs_execute_fsm(conn, ctx, LP_CS_EVT_REJECT, pdu);
		break;
	default:
		/* Unexpected PDU, terminate connection */
		conn->llcp_terminate.reason_final = BT_HCI_ERR_LMP_PDU_NOT_ALLOWED;
		llcp_lr_complete(conn);
		ctx->state = LP_CS_STATE_IDLE;
		break;
	}
}

void llcp_lp_cs_tx_ack(struct ll_conn *conn, struct proc_ctx *ctx, struct node_tx *tx)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(ctx);
	ARG_UNUSED(tx);
}

/*
 * LLCP Remote Procedure FSM
 */

static void rp_cs_tx(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t opcode)
{
	struct node_tx *tx;
	struct pdu_data *pdu;

	tx = llcp_tx_alloc(conn, ctx);
	LL_ASSERT_DBG(tx);

	pdu = (struct pdu_data *)tx->pdu;

	switch (opcode) {
	case PDU_DATA_LLCTRL_TYPE_CS_FAE_RSP:
		cs_pdu_encode_fae_rsp(pdu);
		break;
	case PDU_DATA_LLCTRL_TYPE_CS_RSP:
		cs_pdu_encode_rsp(conn, pdu);
		break;
	case PDU_DATA_LLCTRL_TYPE_CS_TERMINATE_RSP:
		cs_pdu_encode_terminate_rsp(ctx, pdu);
		break;
	default:
		LL_ASSERT_DBG(0);
		break;
	}

	ctx->tx_opcode = pdu->llctrl.opcode;
	ctx->node_ref.tx_ack = tx;

	llcp_tx_enqueue(conn, tx);
}

static void rp_cs_complete(struct ll_conn *conn, struct proc_ctx *ctx)
{
	llcp_rr_complete(conn);
	ctx->state = RP_CS_STATE_IDLE;
}

static void rp_cs_send_rsp(struct ll_conn *conn, struct proc_ctx *ctx)
{
	if (llcp_rr_ispaused(conn) || !llcp_tx_alloc_peek(conn, ctx)) {
		ctx->state = RP_CS_STATE_WAIT_TX_RSP;
		return;
	}

	switch (ctx->data.cs.op) {
	case LLCP_CS_OP_READ_REMOTE_FAE:
		rp_cs_tx(conn, ctx, PDU_DATA_LLCTRL_TYPE_CS_FAE_RSP);
		ctx->state = RP_CS_STATE_WAIT_TX_ACK_RSP;
		break;
	case LLCP_CS_OP_PROCEDURE_ENABLE:
		if (ctx->data.cs.state) {
			rp_cs_tx(conn, ctx, PDU_DATA_LLCTRL_TYPE_CS_RSP);
			ctx->state = RP_CS_STATE_WAIT_TX_ACK_RSP;
		} else {
			/* Acknowledge LL_CS_TERMINATE_REQ by transmitting
			 * LL_CS_TERMINATE_RSP (BT Core Spec v6.3, Vol 6,
			 * Part B, Section 5.1.27).  The host is notified and
			 * the CS procedure is stopped once the response is
			 * acknowledged.
			 */
			rp_cs_tx(conn, ctx, PDU_DATA_LLCTRL_TYPE_CS_TERMINATE_RSP);
			ctx->state = RP_CS_STATE_WAIT_TX_ACK_RSP;
		}
		break;
	default:
		LL_ASSERT_DBG(0);
		break;
	}
}

static void rp_cs_complete_ack(struct ll_conn *conn, struct proc_ctx *ctx)
{
	switch (ctx->data.cs.op) {
	case LLCP_CS_OP_READ_REMOTE_FAE:
		/* Reflector side does not generate a host notification for the
		 * FAE table exchange.
		 */
		rp_cs_complete(conn, ctx);
		break;
	case LLCP_CS_OP_PROCEDURE_ENABLE:
		if (ctx->data.cs.state) {
			/* Per spec section 6.35/6.36, after sending LL_CS_RSP
			 * the reflector waits for LL_CS_IND from the initiator
			 * before starting the CS procedure.
			 */
			ctx->rx_opcode = PDU_DATA_LLCTRL_TYPE_CS_IND;
			ctx->state = RP_CS_STATE_WAIT_RX_IND;
		} else {
			/* LL_CS_TERMINATE_RSP has been acknowledged.  Notify
			 * the host and stop the CS procedure.
			 */
			cs_ntf(conn, ctx,
			       NODE_RX_TYPE_CS_PROCEDURE_ENABLE_COMPLETE,
			       BT_HCI_ERR_SUCCESS, ctx->data.cs.config_id,
			       0U, 0U);
			ull_cs_procedure_stop(conn->lll.handle);
			rp_cs_complete(conn, ctx);
		}
		break;
	default:
		LL_ASSERT_DBG(0);
		break;
	}
}

static void rp_cs_complete_ind(struct ll_conn *conn, struct proc_ctx *ctx)
{
	if (!llcp_ntf_alloc_is_available()) {
		ctx->state = RP_CS_STATE_WAIT_NTF_AVAIL;
		return;
	}

	cs_ntf(conn, ctx, NODE_RX_TYPE_CS_PROCEDURE_ENABLE_COMPLETE,
	       BT_HCI_ERR_SUCCESS, ctx->data.cs.config_id, 0U,
	       ctx->data.cs.state);
	ull_cs_procedure_start(conn->lll.handle, ctx->data.cs.config_id,
			       conn->lll.access_addr);
	rp_cs_complete(conn, ctx);
}

static void rp_cs_rx_decode(struct ll_conn *conn, struct proc_ctx *ctx, struct pdu_data *pdu)
{
	switch (pdu->llctrl.opcode) {
	case PDU_DATA_LLCTRL_TYPE_CS_FAE_REQ:
		ctx->data.cs.op = LLCP_CS_OP_READ_REMOTE_FAE;
		break;
	case PDU_DATA_LLCTRL_TYPE_CS_REQ:
		ctx->data.cs.op = LLCP_CS_OP_PROCEDURE_ENABLE;
		ctx->data.cs.config_id = pdu->llctrl.cs_req.config_id;
		ctx->data.cs.state = 1U;
		/* Reflector: store the scheduling parameters proposed by the
		 * Initiator so they can be selected/echoed in LL_CS_RSP.
		 */
		cs_schedule_store_req(conn, &pdu->llctrl.cs_req);
		break;
	case PDU_DATA_LLCTRL_TYPE_CS_TERMINATE_REQ:
		ctx->data.cs.op = LLCP_CS_OP_PROCEDURE_ENABLE;
		ctx->data.cs.config_id = pdu->llctrl.cs_terminate_req.config_id;
		ctx->data.cs.error = pdu->llctrl.cs_terminate_req.error_code;
		ctx->data.cs.state = 0U;
		break;
	default:
		LL_ASSERT_DBG(0);
		break;
	}
}

static void rp_cs_st_idle(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt, void *param)
{
	switch (evt) {
	case RP_CS_EVT_REQ:
		rp_cs_rx_decode(conn, ctx, (struct pdu_data *)param);
		rp_cs_send_rsp(conn, ctx);
		break;
	default:
		break;
	}
}

static void rp_cs_st_wait_tx_rsp(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt,
				 void *param)
{
	switch (evt) {
	case RP_CS_EVT_RUN:
		rp_cs_send_rsp(conn, ctx);
		break;
	default:
		break;
	}
}

static bool rp_cs_needs_ntf(struct proc_ctx *ctx, uint8_t *count)
{
	if (ctx->data.cs.op == LLCP_CS_OP_PROCEDURE_ENABLE &&
	    !ctx->data.cs.state) {
		/* Only the terminate path needs a notification at ACK time.
		 * The start path defers notification until LL_CS_IND is
		 * received.
		 */
		*count = 1U;
		return true;
	}

	*count = 0U;
	return false;
}

static void rp_cs_st_wait_tx_ack_rsp(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt,
				     void *param)
{
	uint8_t ntf_count;

	switch (evt) {
	case RP_CS_EVT_RUN:
		if (rp_cs_needs_ntf(ctx, &ntf_count) &&
		    !llcp_ntf_alloc_num_available(ntf_count)) {
			ctx->state = RP_CS_STATE_WAIT_NTF_AVAIL;
			break;
		}

		rp_cs_complete_ack(conn, ctx);
		break;
	default:
		break;
	}
}

static void rp_cs_st_wait_rx_ind(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt,
				 void *param)
{
	switch (evt) {
	case RP_CS_EVT_IND:
		/* LL_CS_IND received from initiator. Store the final
		 * negotiated scheduling parameters and start the CS procedure.
		 */
		cs_schedule_store_ind(conn, &((struct pdu_data *)param)->llctrl.cs_ind);
		rp_cs_complete_ind(conn, ctx);
		break;
	default:
		break;
	}
}

static void rp_cs_st_wait_ntf_avail(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt,
				    void *param)
{
	uint8_t ntf_count;

	switch (evt) {
	case RP_CS_EVT_RUN:
		(void)rp_cs_needs_ntf(ctx, &ntf_count);
		if (llcp_ntf_alloc_num_available(ntf_count)) {
			if (ctx->data.cs.op == LLCP_CS_OP_PROCEDURE_ENABLE &&
			    !ctx->data.cs.state) {
				/* Terminate case: complete without TX */
				cs_ntf(conn, ctx,
				       NODE_RX_TYPE_CS_PROCEDURE_ENABLE_COMPLETE,
				       BT_HCI_ERR_SUCCESS,
				       ctx->data.cs.config_id, 0U, 0U);
				ull_cs_procedure_stop(conn->lll.handle);
				rp_cs_complete(conn, ctx);
			} else if (ctx->data.cs.op == LLCP_CS_OP_PROCEDURE_ENABLE &&
				   ctx->data.cs.state) {
				/* IND completion waiting for ntf */
				rp_cs_complete_ind(conn, ctx);
			} else {
				rp_cs_complete_ack(conn, ctx);
			}
		}
		break;
	default:
		break;
	}
}

static void rp_cs_execute_fsm(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt, void *param)
{
	switch (ctx->state) {
	case RP_CS_STATE_IDLE:
		rp_cs_st_idle(conn, ctx, evt, param);
		break;
	case RP_CS_STATE_WAIT_TX_RSP:
		rp_cs_st_wait_tx_rsp(conn, ctx, evt, param);
		break;
	case RP_CS_STATE_WAIT_TX_ACK_RSP:
		rp_cs_st_wait_tx_ack_rsp(conn, ctx, evt, param);
		break;
	case RP_CS_STATE_WAIT_RX_IND:
		rp_cs_st_wait_rx_ind(conn, ctx, evt, param);
		break;
	case RP_CS_STATE_WAIT_NTF_AVAIL:
		rp_cs_st_wait_ntf_avail(conn, ctx, evt, param);
		break;
	default:
		LL_ASSERT_DBG(0);
		break;
	}
}

void llcp_rp_cs_init_proc(struct proc_ctx *ctx)
{
	ctx->state = RP_CS_STATE_IDLE;
}

void llcp_rp_cs_run(struct ll_conn *conn, struct proc_ctx *ctx, void *param)
{
	rp_cs_execute_fsm(conn, ctx, RP_CS_EVT_RUN, param);
}

void llcp_rp_cs_rx(struct ll_conn *conn, struct proc_ctx *ctx, struct node_rx_pdu *rx)
{
	struct pdu_data *pdu = (struct pdu_data *)rx->pdu;

	switch (pdu->llctrl.opcode) {
	case PDU_DATA_LLCTRL_TYPE_CS_FAE_REQ:
	case PDU_DATA_LLCTRL_TYPE_CS_REQ:
	case PDU_DATA_LLCTRL_TYPE_CS_TERMINATE_REQ:
		rp_cs_execute_fsm(conn, ctx, RP_CS_EVT_REQ, pdu);
		break;
	case PDU_DATA_LLCTRL_TYPE_CS_IND:
		rp_cs_execute_fsm(conn, ctx, RP_CS_EVT_IND, pdu);
		break;
	default:
		/* Unexpected PDU, terminate connection */
		conn->llcp_terminate.reason_final = BT_HCI_ERR_LMP_PDU_NOT_ALLOWED;
		llcp_rr_complete(conn);
		ctx->state = RP_CS_STATE_IDLE;
		break;
	}
}

void llcp_rp_cs_tx_ack(struct ll_conn *conn, struct proc_ctx *ctx, struct node_tx *tx)
{
	ARG_UNUSED(tx);

	if (ctx->state == RP_CS_STATE_WAIT_TX_ACK_RSP) {
		rp_cs_execute_fsm(conn, ctx, RP_CS_EVT_RUN, NULL);
	}
}

#endif /* CONFIG_BT_CTLR_CHANNEL_SOUNDING */
