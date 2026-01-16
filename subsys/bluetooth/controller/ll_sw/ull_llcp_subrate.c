/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>

#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/slist.h>
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
#include "ll_settings.h"

#include "lll.h"
#include "lll/lll_df_types.h"
#include "lll_conn.h"

#include "ull_tx_queue.h"

#include "ull_conn_types.h"
#include "ull_conn_internal.h"
#include "ull_internal.h"
#include "ull_llcp.h"
#include "ull_llcp_features.h"
#include "ull_llcp_internal.h"

#include <soc.h>
#include "hal/debug.h"

#if defined(CONFIG_BT_CTLR_SUBRATING)

#define BT_DBG_ENABLED IS_ENABLED(CONFIG_BT_DEBUG_HCI_DRIVER)
#define LOG_MODULE_NAME bt_ctlr_ull_llcp_subrate
#include "common/log.h"
#include <soc.h>
#include "hal/debug.h"

/* Subrate parameter limits according to BT Core Spec v6.2 */
#define SUBRATE_FACTOR_MIN 0x0001
#define SUBRATE_FACTOR_MAX 0x01F4
#define MAX_LATENCY_MAX 0x01F3
#define CONTINUATION_NUMBER_MAX 0x01F3
#define SUPERVISION_TIMEOUT_MIN 0x000A
#define SUPERVISION_TIMEOUT_MAX 0x0C80

/* LLCP Local Procedure Subrate Update FSM states */
enum {
	LP_SR_STATE_IDLE = LLCP_STATE_IDLE,
	LP_SR_STATE_WAIT_TX_SUBRATE_REQ,
	LP_SR_STATE_WAIT_RX_SUBRATE_IND,
	LP_SR_STATE_WAIT_INSTANT,
	LP_SR_STATE_WAIT_NTF,
};

/* LLCP Local Procedure Subrate Update FSM events */
enum {
	/* Procedure run */
	LP_SR_EVT_RUN,

	/* Indication received */
	LP_SR_EVT_SUBRATE_IND,
};

/* LLCP Remote Procedure Subrate Update FSM states */
enum {
	RP_SR_STATE_IDLE = LLCP_STATE_IDLE,
	RP_SR_STATE_WAIT_RX_SUBRATE_REQ,
	RP_SR_STATE_WAIT_TX_SUBRATE_IND,
	RP_SR_STATE_WAIT_INSTANT,
	RP_SR_STATE_WAIT_NTF,
};

/* LLCP Remote Procedure Subrate Update FSM events */
enum {
	/* Procedure run */
	RP_SR_EVT_RUN,

	/* Request received */
	RP_SR_EVT_SUBRATE_REQ,
};

/*
 * LLCP Local Procedure Subrate Update FSM
 */

static void lp_sr_tx(struct ll_conn *conn, struct proc_ctx *ctx)
{
	struct node_tx *tx;
	struct pdu_data *pdu;

	/* Allocate tx node */
	tx = llcp_tx_alloc(conn, ctx);
	LL_ASSERT(tx);

	pdu = (struct pdu_data *)tx->pdu;

	/* Encode LL_SUBRATE_REQ Control PDU */
	llcp_pdu_encode_subrate_req(ctx, pdu);

	ctx->tx_opcode = pdu->llctrl.opcode;

	/* Enqueue LL Control PDU towards LLL */
	llcp_tx_enqueue(conn, tx);
}

static void lp_sr_complete(struct ll_conn *conn, struct proc_ctx *ctx)
{
	llcp_lr_complete(conn);
	ctx->state = LP_SR_STATE_IDLE;
}

static void lp_sr_send_subrate_req(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt,
				   void *param)
{
	if (llcp_lr_ispaused(conn) || !llcp_tx_alloc_peek(conn, ctx)) {
		ctx->state = LP_SR_STATE_WAIT_TX_SUBRATE_REQ;
	} else {
		lp_sr_tx(conn, ctx);
		ctx->rx_opcode = PDU_DATA_LLCTRL_TYPE_SUBRATE_IND;
		ctx->state = LP_SR_STATE_WAIT_RX_SUBRATE_IND;
	}
}

static void lp_sr_st_wait_tx_subrate_req(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt,
					void *param)
{
	switch (evt) {
	case LP_SR_EVT_RUN:
		lp_sr_send_subrate_req(conn, ctx, evt, param);
		break;
	default:
		/* Ignore other events */
		break;
	}
}

static void lp_sr_st_wait_rx_subrate_ind(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt,
					void *param)
{
	switch (evt) {
	case LP_SR_EVT_SUBRATE_IND:
		llcp_pdu_decode_subrate_ind(ctx, (struct pdu_data *)param);

		/* Apply subrate parameters */
		conn->lll.event_counter = ctx->data.sr.subrate_base_event;
		conn->subrate_factor = ctx->data.sr.subrate_factor;
		conn->subrate_base_event = ctx->data.sr.subrate_base_event;
		conn->continuation_number = ctx->data.sr.continuation_number;
		conn->lll.latency = ctx->data.sr.latency;
		conn->supervision_timeout = ctx->data.sr.supervision_timeout;

		ctx->data.sr.error = BT_HCI_ERR_SUCCESS;
		ctx->state = LP_SR_STATE_WAIT_NTF;
		break;
	default:
		/* Ignore other events */
		break;
	}
}

static void lp_sr_st_wait_ntf(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt,
			     void *param)
{
	switch (evt) {
	case LP_SR_EVT_RUN:
		/* Generate subrate change complete event */
		ctx->node_ref.rx = llcp_ntf_alloc();
		if (ctx->node_ref.rx) {
			struct node_rx_pdu *ntf = ctx->node_ref.rx;
			struct pdu_data *pdu = (struct pdu_data *)ntf->pdu;

			ntf->hdr.type = NODE_RX_TYPE_SUBRATE_CHANGE;
			ntf->hdr.handle = conn->lll.handle;

			/* Encode subrate change complete */
			pdu->llctrl.opcode = PDU_DATA_LLCTRL_TYPE_SUBRATE_IND;
			
			/* Notification will be picked up by HCI */
			llcp_ntf_set_pending(conn);
			
			lp_sr_complete(conn, ctx);
		}
		break;
	default:
		/* Ignore other events */
		break;
	}
}

static void lp_sr_execute_fsm(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt,
			     void *param)
{
	switch (ctx->state) {
	case LP_SR_STATE_IDLE:
		/* No action needed */
		break;
	case LP_SR_STATE_WAIT_TX_SUBRATE_REQ:
		lp_sr_st_wait_tx_subrate_req(conn, ctx, evt, param);
		break;
	case LP_SR_STATE_WAIT_RX_SUBRATE_IND:
		lp_sr_st_wait_rx_subrate_ind(conn, ctx, evt, param);
		break;
	case LP_SR_STATE_WAIT_NTF:
		lp_sr_st_wait_ntf(conn, ctx, evt, param);
		break;
	default:
		/* Unknown state */
		LL_ASSERT(0);
	}
}

void llcp_lp_sr_rx(struct ll_conn *conn, struct proc_ctx *ctx, struct node_rx_pdu *rx)
{
	struct pdu_data *pdu = (struct pdu_data *)rx->pdu;

	switch (pdu->llctrl.opcode) {
	case PDU_DATA_LLCTRL_TYPE_SUBRATE_IND:
		lp_sr_execute_fsm(conn, ctx, LP_SR_EVT_SUBRATE_IND, pdu);
		break;
	default:
		/* Unknown opcode */
		break;
	}
}

void llcp_lp_sr_init_proc(struct proc_ctx *ctx)
{
	ctx->state = LP_SR_STATE_WAIT_TX_SUBRATE_REQ;
}

void llcp_lp_sr_run(struct ll_conn *conn, struct proc_ctx *ctx, void *param)
{
	lp_sr_execute_fsm(conn, ctx, LP_SR_EVT_RUN, param);
}

/*
 * LLCP Remote Procedure Subrate Update FSM
 */

static void rp_sr_tx(struct ll_conn *conn, struct proc_ctx *ctx)
{
	struct node_tx *tx;
	struct pdu_data *pdu;

	/* Allocate tx node */
	tx = llcp_tx_alloc(conn, ctx);
	LL_ASSERT(tx);

	pdu = (struct pdu_data *)tx->pdu;

	/* Encode LL_SUBRATE_IND Control PDU */
	llcp_pdu_encode_subrate_ind(ctx, pdu);

	ctx->tx_opcode = pdu->llctrl.opcode;

	/* Enqueue LL Control PDU towards LLL */
	llcp_tx_enqueue(conn, tx);
}

static void rp_sr_complete(struct ll_conn *conn, struct proc_ctx *ctx)
{
	llcp_rr_complete(conn);
	ctx->state = RP_SR_STATE_IDLE;
}

static void rp_sr_send_subrate_ind(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt,
				  void *param)
{
	if (llcp_rr_ispaused(conn) || !llcp_tx_alloc_peek(conn, ctx)) {
		ctx->state = RP_SR_STATE_WAIT_TX_SUBRATE_IND;
	} else {
		/* Calculate acceptable subrate parameters */
		uint16_t subrate_factor;
		
		/* Use the minimum acceptable subrate factor within requested range */
		subrate_factor = ctx->data.sr.subrate_factor_min;
		if (subrate_factor < SUBRATE_FACTOR_MIN) {
			subrate_factor = SUBRATE_FACTOR_MIN;
		}
		if (subrate_factor > ctx->data.sr.subrate_factor_max) {
			subrate_factor = ctx->data.sr.subrate_factor_max;
		}

		ctx->data.sr.subrate_factor = subrate_factor;
		ctx->data.sr.subrate_base_event = conn->lll.event_counter + 6;
		ctx->data.sr.latency = MIN(ctx->data.sr.max_latency,
					  (500 / subrate_factor) - 1);

		/* Use requested values for continuation and timeout */
		/* continuation_number and supervision_timeout already in ctx */

		rp_sr_tx(conn, ctx);
		ctx->state = RP_SR_STATE_WAIT_NTF;
	}
}

static void rp_sr_st_wait_rx_subrate_req(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt,
					void *param)
{
	switch (evt) {
	case RP_SR_EVT_SUBRATE_REQ:
		llcp_pdu_decode_subrate_req(ctx, (struct pdu_data *)param);
		rp_sr_send_subrate_ind(conn, ctx, evt, param);
		break;
	default:
		/* Ignore other events */
		break;
	}
}

static void rp_sr_st_wait_tx_subrate_ind(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt,
					void *param)
{
	switch (evt) {
	case RP_SR_EVT_RUN:
		rp_sr_send_subrate_ind(conn, ctx, evt, param);
		break;
	default:
		/* Ignore other events */
		break;
	}
}

static void rp_sr_st_wait_ntf(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt,
			     void *param)
{
	switch (evt) {
	case RP_SR_EVT_RUN:
		/* Apply subrate parameters */
		conn->lll.event_counter = ctx->data.sr.subrate_base_event;
		conn->subrate_factor = ctx->data.sr.subrate_factor;
		conn->subrate_base_event = ctx->data.sr.subrate_base_event;
		conn->continuation_number = ctx->data.sr.continuation_number;
		conn->lll.latency = ctx->data.sr.latency;
		conn->supervision_timeout = ctx->data.sr.supervision_timeout;

		/* Generate subrate change complete event */
		ctx->node_ref.rx = llcp_ntf_alloc();
		if (ctx->node_ref.rx) {
			struct node_rx_pdu *ntf = ctx->node_ref.rx;
			struct pdu_data *pdu = (struct pdu_data *)ntf->pdu;

			ntf->hdr.type = NODE_RX_TYPE_SUBRATE_CHANGE;
			ntf->hdr.handle = conn->lll.handle;

			/* Encode subrate change complete */
			pdu->llctrl.opcode = PDU_DATA_LLCTRL_TYPE_SUBRATE_IND;
			
			/* Notification will be picked up by HCI */
			llcp_ntf_set_pending(conn);
			
			rp_sr_complete(conn, ctx);
		}
		break;
	default:
		/* Ignore other events */
		break;
	}
}

static void rp_sr_execute_fsm(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt,
			     void *param)
{
	switch (ctx->state) {
	case RP_SR_STATE_IDLE:
		/* No action needed */
		break;
	case RP_SR_STATE_WAIT_RX_SUBRATE_REQ:
		rp_sr_st_wait_rx_subrate_req(conn, ctx, evt, param);
		break;
	case RP_SR_STATE_WAIT_TX_SUBRATE_IND:
		rp_sr_st_wait_tx_subrate_ind(conn, ctx, evt, param);
		break;
	case RP_SR_STATE_WAIT_NTF:
		rp_sr_st_wait_ntf(conn, ctx, evt, param);
		break;
	default:
		/* Unknown state */
		LL_ASSERT(0);
	}
}

void llcp_rp_sr_rx(struct ll_conn *conn, struct proc_ctx *ctx, struct node_rx_pdu *rx)
{
	struct pdu_data *pdu = (struct pdu_data *)rx->pdu;

	switch (pdu->llctrl.opcode) {
	case PDU_DATA_LLCTRL_TYPE_SUBRATE_REQ:
		rp_sr_execute_fsm(conn, ctx, RP_SR_EVT_SUBRATE_REQ, pdu);
		break;
	default:
		/* Unknown opcode */
		break;
	}
}

void llcp_rp_sr_init_proc(struct proc_ctx *ctx)
{
	ctx->state = RP_SR_STATE_WAIT_RX_SUBRATE_REQ;
}

void llcp_rp_sr_run(struct ll_conn *conn, struct proc_ctx *ctx, void *param)
{
	rp_sr_execute_fsm(conn, ctx, RP_SR_EVT_RUN, param);
}

#endif /* CONFIG_BT_CTLR_SUBRATING */
