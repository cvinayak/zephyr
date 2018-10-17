/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stddef.h>
#include <zephyr.h>

#include "hal/ccm.h"
#include "hal/ticker.h"

#include "util/mem.h"
#include "util/memq.h"
#include "util/mfifo.h"
#include "util/mayfly.h"

#include "ticker/ticker.h"

#include "pdu.h"
#include "lll.h"
#include "lll_conn.h"
#include "ull_conn_types.h"
#include "ull_internal.h"
#include "ull_slave_internal.h"
#include "ull_master_internal.h"

#include "common/log.h"
#include <soc.h>
#include "hal/debug.h"

static int _init_reset(void);
static struct ll_conn *is_connected_get(u16_t handle);
static void ticker_op_update_cb(u32_t status, void *param);
static void terminate_ind_rx_enqueue(struct lll_conn *lll, u8_t reason);
static void conn_cleanup(struct lll_conn *lll);
static void ctrl_tx_enqueue(struct ll_conn *conn, struct node_tx *node_tx);
static void ctrl_tx_sec_enqueue(struct ll_conn *conn, struct node_tx *node_tx);
static inline void ctrl_tx_ack(struct lll_conn *lll, struct pdu_data *pdu);
static inline u8_t ctrl_rx(u16_t handle, struct pdu_data *pdu);

static struct ll_conn _conn[CONFIG_BT_MAX_CONN];
static void *_conn_free;

#define CONN_TX_BUF_SIZE (MROUND(offsetof(struct node_tx, pdu) + \
				 offsetof(struct pdu_data, lldata) + \
				 (CONFIG_BT_CTLR_TX_BUFFER_SIZE)) * \
			  (CONFIG_BT_CTLR_TX_BUFFERS))

#define CONN_TX_CTRL_BUFFERS 2
#define CONN_TX_CTRL_BUF_SIZE (MROUND(offsetof(struct node_tx, pdu) + \
				      offsetof(struct pdu_data, llctrl) + \
				      sizeof(struct pdu_data_llctrl)) * \
			       CONN_TX_CTRL_BUFFERS)

static MFIFO_DEFINE(conn_tx, sizeof(struct lll_tx),
		    CONFIG_BT_CTLR_TX_BUFFERS);

static struct {
	void *free;
	u8_t pool[CONN_TX_BUF_SIZE * CONFIG_BT_CTLR_TX_BUFFERS];
} mem_conn_tx;

static struct {
	void *free;
	u8_t pool[CONN_TX_CTRL_BUF_SIZE * CONN_TX_CTRL_BUFFERS];
} mem_conn_tx_ctrl;

static struct {
	void *free;
	u8_t pool[sizeof(memq_link_t) *
		  (CONFIG_BT_CTLR_TX_BUFFERS + CONN_TX_CTRL_BUFFERS)];
} mem_link_tx;

struct ll_conn *ll_conn_acquire(void)
{
	return mem_acquire(&_conn_free);
}

void ll_conn_release(struct ll_conn *conn)
{
	mem_release(conn, &_conn_free);
}

u16_t ll_conn_handle_get(struct ll_conn *conn)
{
	return mem_index_get(conn, _conn, sizeof(struct ll_conn));
}

struct ll_conn *ll_conn_get(u16_t handle)
{
	return mem_get(_conn, sizeof(struct ll_conn), handle);
}

void *ll_tx_mem_acquire(void)
{
	return mem_acquire(&mem_conn_tx.free);;
}

void ll_tx_mem_release(void *node_tx)
{
	mem_release(node_tx, &mem_conn_tx.free);
}

u8_t ll_tx_mem_enqueue(u16_t handle, void *node_tx)
{
	struct lll_tx *tx;
	struct ll_conn *conn;
	u8_t idx;

	conn = is_connected_get(handle);
	if (!conn) {
		return -EINVAL;
	}

	idx = MFIFO_ENQUEUE_GET(conn_tx, (void **) &tx);
	if (!tx) {
		return -ENOBUFS;
	}

	tx->handle = handle;
	tx->node = node_tx;

	MFIFO_ENQUEUE(conn_tx, idx);

	return 0;
}

u8_t ll_terminate_ind_send(u16_t handle, u8_t reason)
{
	struct ll_conn *conn;

	conn = is_connected_get(handle);
	if (!conn) {
		return -EINVAL;
	}

	conn->llcp_terminate.reason_own = reason;

	conn->llcp_terminate.req++;

	return 0;
}

u8_t ll_version_ind_send(u16_t handle)
{
	return 0;
}

u8_t ll_feature_req_send(u16_t handle)
{
	return 0;
}

u8_t ll_chm_get(u16_t handle, u8_t *chm)
{
	return 0;
}

u8_t ll_conn_update(u16_t handle, u8_t cmd, u8_t status, u16_t interval,
		     u16_t latency, u16_t timeout)
{
	return 0;
}

#if defined(CONFIG_BT_CTLR_CONN_RSSI)
u32_t ll_rssi_get(u16_t handle, u8_t *rssi)
{
	struct ll_conn *conn;

	conn = is_connected_get(handle);
	if (!conn) {
		return -EINVAL;
	}

	*rssi = conn->lll.rssi_latest;

	return 0;
}
#endif /* CONFIG_BT_CTLR_CONN_RSSI */

int ull_conn_init(void)
{
	int err;

	err = _init_reset();
	if (err) {
		return err;
	}

	return 0;
}

int ull_conn_reset(void)
{
	int err;

	/* Re-initialize the Tx mfifo */
	MFIFO_INIT(conn_tx);

	err = _init_reset();
	if (err) {
		return err;
	}

	return 0;
}

void ull_conn_setup(memq_link_t *link, struct node_rx_hdr *rx)
{
	struct node_rx_ftr *ftr;
	struct lll_conn *lll;

	ftr = (void *)((u8_t *)((struct node_rx_pdu *)rx)->pdu +
		       (offsetof(struct pdu_adv, connect_ind) +
		       sizeof(struct pdu_adv_connect_ind)));

	lll = *((struct lll_conn **)((u8_t *)ftr->param +
				     sizeof(struct lll_hdr)));
	switch (lll->role) {
#if defined(CONFIG_BT_CENTRAL)
	case 0:
		ull_master_setup(link, rx, ftr, lll);
		break;
#endif /* CONFIG_BT_CENTRAL */

#if defined(CONFIG_BT_PERIPHERAL)
	case 1:
		ull_slave_setup(link, rx, ftr, lll);
		break;
#endif /* CONFIG_BT_PERIPHERAL */

	default:
		LL_ASSERT(0);
		break;
	}
}

int ull_conn_rx(struct node_rx_pdu *rx)
{
	struct pdu_data *pdu;
	int nack = 0;

	pdu = (void *)rx->pdu;
	switch (pdu->ll_id) {
	case PDU_DATA_LLID_CTRL:
		nack = ctrl_rx(rx->hdr.handle, pdu);
		break;

	case PDU_DATA_LLID_DATA_CONTINUE:
	case PDU_DATA_LLID_DATA_START:
		/* enqueue data packet, as-is */
	case PDU_DATA_LLID_RESV:
	default:
		/* Invalid LL id, drop it. */
		break;
	}

	return nack;
}

void ull_conn_llcp(struct ll_conn *conn)
{
	/* Terminate Procedure Request */
	if (conn->llcp_terminate.ack != conn->llcp_terminate.req) {
		struct node_tx *node_tx;

		node_tx = mem_acquire(&mem_conn_tx_ctrl.free);
		if (node_tx) {
			struct pdu_data *pdu_ctrl_tx = (void *)node_tx->pdu;

			/* Terminate Procedure acked */
			conn->llcp_terminate.ack = conn->llcp_terminate.req;

			/* place the terminate ind packet in tx queue */
			pdu_ctrl_tx->ll_id = PDU_DATA_LLID_CTRL;
			pdu_ctrl_tx->len = offsetof(struct pdu_data_llctrl,
						    terminate_ind) +
				sizeof(struct pdu_data_llctrl_terminate_ind);
			pdu_ctrl_tx->llctrl.opcode =
				PDU_DATA_LLCTRL_TYPE_TERMINATE_IND;
			pdu_ctrl_tx->llctrl.terminate_ind.error_code =
				conn->llcp_terminate.reason_own;

			ctrl_tx_enqueue(conn, node_tx);
		}

		if (!conn->procedure_expire) {
			/* Terminate Procedure timeout is started, will
			 * replace any other timeout running
			 */
			conn->procedure_expire = conn->supervision_reload;

			/* NOTE: if supervision timeout equals connection
			 * interval, dont timeout in current event.
			 */
			if (conn->procedure_expire <= 1) {
				conn->procedure_expire++;
			}
		}
	}
}

void ull_conn_done(struct node_rx_event_done *done)
{
	struct lll_conn *lll = (void *)HDR_ULL2LLL(done->param);
	struct ll_conn *conn = (void *)HDR_LLL2EVT(lll);
	u32_t ticks_drift_minus;
	u32_t ticks_drift_plus;
	u16_t latency_event;
	u16_t elapsed_event;
	u8_t reason_peer;
	u16_t trx_cnt;
	u16_t lazy;
	u8_t force;

	/* Master transmitted ack for the received terminate ind or
	 * Slave received terminate ind.
	 */
	reason_peer = conn->llcp_terminate.reason_peer;
	if (reason_peer && (lll->role || lll->master.terminate_ack)) {
		terminate_ind_rx_enqueue(lll, reason_peer);
		conn_cleanup(lll);

		return;
	}

	ticks_drift_plus = 0;
	ticks_drift_minus = 0;
	latency_event = lll->latency_event;
	elapsed_event = latency_event + 1;

	trx_cnt = done->extra.trx_cnt;
	if (trx_cnt) {
		if (IS_ENABLED(CONFIG_BT_PERIPHERAL) && lll->role) {
			ull_slave_done(done, &ticks_drift_plus,
				       &ticks_drift_minus);
		} else if (reason_peer) {
			lll->master.terminate_ack = 1;
		}

		/* Reset connection failed to establish countdown */
		conn->connect_expire = 0;

		/* Reset supervision countdown */
		conn->supervision_expire = 0;
	}

	/* check connection failed to establish */
	else if (conn->connect_expire) {
		if (conn->connect_expire > elapsed_event) {
			conn->connect_expire -= elapsed_event;
		} else {
			terminate_ind_rx_enqueue(lll, 0x3e);

			conn_cleanup(lll);

			return;
		}
	}

	/* if anchor point not sync-ed, start supervision timeout, and break
	 * latency if any.
	 */
	else {
		/* Start supervision timeout, if not started already */
		if (!conn->supervision_expire) {
			conn->supervision_expire = conn->supervision_reload;
		}
	}

	/* check supervision timeout */
	force = 0;
	if (conn->supervision_expire) {
		if (conn->supervision_expire > elapsed_event) {
			conn->supervision_expire -= elapsed_event;

			/* break latency */
			lll->latency_event = 0;

			/* Force both master and slave when close to
			 * supervision timeout.
			 */
			if (conn->supervision_expire <= 6) {
				force = 1;
			}
			/* use randomness to force slave role when anchor
			 * points are being missed.
			 */
			else if (lll->role) {
				if (latency_event) {
					force = 1;
				} else {
					/* FIXME:*/
					#if 0
					force = lll->slave.force & 0x01;

					/* rotate force bits */
					lll->slave.force >>= 1;
					if (force) {
						lll->slave.force |= BIT(31);
					}
					#endif
				}
			}
		} else {
			terminate_ind_rx_enqueue(lll, 0x08);

			conn_cleanup(lll);

			return;
		}
	}

	/* check procedure timeout */
	if (conn->procedure_expire != 0) {
		if (conn->procedure_expire > elapsed_event) {
			conn->procedure_expire -= elapsed_event;
		} else {
			terminate_ind_rx_enqueue(lll, 0x22);

			conn_cleanup(lll);

			return;
		}
	}

#if defined(CONFIG_BT_CTLR_LE_PING)
	/* check apto */
	if (lll->apto_expire != 0) {
		if (lll->apto_expire > elapsed_event) {
			lll->apto_expire -= elapsed_event;
		} else {
			struct radio_pdu_node_rx *node_rx;

			lll->apto_expire = 0;

			/* Prepare the rx packet structure */
			node_rx = packet_rx_reserve_get(2);
			LL_ASSERT(node_rx);

			node_rx->hdr.handle = lll->handle;
			node_rx->hdr.type = NODE_RX_TYPE_APTO;

			/* enqueue apto event into rx queue */
			packet_rx_enqueue();
		}
	}

	/* check appto */
	if (lll->appto_expire != 0) {
		if (lll->appto_expire > elapsed_event) {
			lll->appto_expire -= elapsed_event;
		} else {
			lll->appto_expire = 0;

			if ((lll->procedure_expire == 0) &&
			    (lll->llcp_req == lll->llcp_ack)) {
				lll->llcp_type = LLCP_PING;
				lll->llcp_ack--;
			}
		}
	}
#endif /* CONFIG_BT_CTLR_LE_PING */

#if defined(CONFIG_BT_CTLR_CONN_RSSI)
	/* generate RSSI event */
	if (lll->rssi_sample_count == 0) {
		struct node_rx_pdu *rx;
		struct pdu_data *pdu_data_rx;

		/* TODO: allocate rx ull-to-thread */
		if (rx) {
			lll->rssi_reported = lll->rssi_latest;
			lll->rssi_sample_count = LLL_CONN_RSSI_SAMPLE_COUNT;

			/* Prepare the rx packet structure */
			rx->hdr.handle = lll->handle;
			rx->hdr.type = NODE_RX_TYPE_RSSI;

			/* prepare connection RSSI structure */
			pdu_data_rx = (void *)rx->pdu;
			pdu_data_rx->rssi = lll->rssi_reported;

			/* enqueue connection RSSI structure into queue */
			/* TODO: */
		}
	}
#endif /* CONFIG_BT_CTLR_CONN_RSSI */

	/* break latency based on ctrl procedure pending */
	if ((conn->llcp_ack != conn->llcp_req) &&
	    ((conn->llcp_type == LLCP_CONN_UPD) ||
	     (conn->llcp_type == LLCP_CHAN_MAP))) {
		lll->latency_event = 0;
	}

	/* check if latency needs update */
	lazy = 0;
	if ((force) || (latency_event != lll->latency_event)) {
		lazy = lll->latency_event + 1;
	}

	/* update conn ticker */
	if ((ticks_drift_plus != 0) || (ticks_drift_minus != 0) ||
	    (lazy != 0) || (force != 0)) {
		u8_t ticker_id = TICKER_ID_CONN_BASE + lll->handle;
		struct ll_conn *conn = lll->hdr.parent;
		u32_t ticker_status;

		/* Call to ticker_update can fail under the race
		 * condition where in the Slave role is being stopped but
		 * at the same time it is preempted by Slave event that
		 * gets into close state. Accept failure when Slave role
		 * is being stopped.
		 */
		ticker_status = ticker_update(TICKER_INSTANCE_ID_CTLR,
					      TICKER_USER_ID_ULL_HIGH,
					      ticker_id,
					      ticks_drift_plus,
					      ticks_drift_minus, 0, 0,
					      lazy, force,
					      ticker_op_update_cb,
					      conn);
		LL_ASSERT((ticker_status == TICKER_STATUS_SUCCESS) ||
			  (ticker_status == TICKER_STATUS_BUSY) ||
			  ((void *)conn == ull_disable_mark_get()));
	}
}

void ull_conn_tx_demux(u8_t count)
{
	do {
		struct ll_conn *conn;
		struct lll_tx *tx;

		tx = MFIFO_DEQUEUE_GET(conn_tx);
		if (!tx) {
			break;
		}

		conn = ll_conn_get(tx->handle);
		if (conn && (conn->lll.handle == tx->handle)) {
			struct node_tx *node_tx_new = tx->node;

			node_tx_new->next = NULL;
			if (!conn->tx_data) {
				conn->tx_data = node_tx_new;
				if (!conn->tx_head) {
					conn->tx_head = node_tx_new;
					conn->tx_data_last = NULL;
				}
			}

			if (conn->tx_data_last) {
				conn->tx_data_last->next = node_tx_new;
			}

			conn->tx_data_last = node_tx_new;
		} else {
			struct node_tx *node_tx = tx->node;
			struct pdu_data *p = (void *)node_tx->pdu;

			p->ll_id = PDU_DATA_LLID_RESV;
			ull_tx_ack_put(tx->handle, node_tx);
		}

		MFIFO_DEQUEUE(conn_tx);
	} while (--count);
}

void ull_conn_tx_lll_enqueue(struct ll_conn *conn, u8_t count)
{
	struct node_tx *node_tx;

	node_tx = conn->tx_head;
	while (node_tx &&
	       (!conn->pause_tx || (node_tx == conn->tx_ctrl)) &&
	       count--) {
		struct node_tx *node_tx_lll;
		memq_link_t *link;

		node_tx_lll = node_tx;

		if (node_tx == conn->tx_ctrl) {
			node_tx = conn->tx_head = conn->tx_head->next;
			if (conn->tx_ctrl == conn->tx_ctrl_last) {
				conn->tx_ctrl = NULL;
				conn->tx_ctrl_last = NULL;
			} else {
				conn->tx_ctrl = node_tx;
			}

			/* point to self to indicate a control PDU mem alloc */
			node_tx_lll->next = node_tx_lll;
		} else {
			if (node_tx == conn->tx_data) {
				conn->tx_data = conn->tx_data->next;
			}
			node_tx = conn->tx_head = conn->tx_head->next;
		}

		link = mem_acquire(&mem_link_tx.free);
		LL_ASSERT(link);

		memq_enqueue(link, node_tx_lll, &conn->lll.memq_tx.tail);
	}
}

void ull_conn_link_tx_release(void *link)
{
	mem_release(link, &mem_link_tx.free);
}

void ull_conn_tx_ack(struct lll_conn *lll, memq_link_t *link,
		     struct node_tx *tx)
{
	struct pdu_data *pdu;

	pdu = (void *)tx->pdu;
	LL_ASSERT(pdu->len);

	if (pdu->ll_id == PDU_DATA_LLID_CTRL) {
		ctrl_tx_ack(lll, pdu);

		/* release mem if points to itself */
		if (link->next == tx) {
			mem_release(tx, &mem_conn_tx_ctrl.free);
			return;
		}
	}

	ull_tx_ack_put(lll->handle, tx);
}

static int _init_reset(void)
{
	/* Initialize conn pool. */
	mem_init(_conn, sizeof(struct ll_conn),
		 sizeof(_conn) / sizeof(struct ll_conn), &_conn_free);

	/* Initialize tx pool. */
	mem_init(mem_conn_tx.pool, CONN_TX_BUF_SIZE, CONFIG_BT_CTLR_TX_BUFFERS,
		 &mem_conn_tx.free);

	/* Initialize tx ctrl pool. */
	mem_init(mem_conn_tx_ctrl.pool, CONN_TX_CTRL_BUF_SIZE,
		 CONN_TX_CTRL_BUFFERS, &mem_conn_tx_ctrl.free);

	/* Initialize tx link pool. */
	mem_init(mem_link_tx.pool, sizeof(memq_link_t),
		 CONFIG_BT_CTLR_TX_BUFFERS, &mem_link_tx.free);

	return 0;
}

static struct ll_conn *is_connected_get(u16_t handle)
{
	struct ll_conn *conn;

	if (handle >= CONFIG_BT_MAX_CONN) {
		return NULL;
	}

	conn = ll_conn_get(handle);
	if (conn->lll.handle != handle) {
		return NULL;
	}

	return conn;
}

static void terminate_ind_rx_enqueue(struct lll_conn *lll, u8_t reason)
{
	struct ll_conn *conn = (void *)HDR_LLL2EVT(lll);
	struct node_rx_pdu *rx;
	memq_link_t *link;

	/* Prepare the rx packet structure */
	rx = (void *)&conn->llcp_terminate.node_rx;
	LL_ASSERT(rx->hdr.link);

	rx->hdr.handle = lll->handle;
	rx->hdr.type = NODE_RX_TYPE_TERMINATE;
	*((u8_t *)rx->pdu) = reason;

	/* Get the link mem reserved in the connection context */
	link = rx->hdr.link;
	rx->hdr.link = NULL;

	ll_rx_put(link, rx);
	ll_rx_sched();
}

static void ticker_op_update_cb(u32_t status, void *param)
{
	LL_ASSERT(status == TICKER_STATUS_SUCCESS ||
		  param == ull_disable_mark_get());
}

static void ticker_op_stop_cb(u32_t status, void *param)
{
	static memq_link_t _link;
	static struct mayfly _mfy = {0, 0, &_link, NULL, lll_conn_tx_flush};

	LL_ASSERT(status == TICKER_STATUS_SUCCESS);

	_mfy.param = param;

	/* Flush pending tx PDUs in LLL (using a mayfly) */
	mayfly_enqueue(TICKER_USER_ID_ULL_LOW, TICKER_USER_ID_LLL, 1, &_mfy);
}

static void conn_cleanup(struct lll_conn *lll)
{
	u8_t ticker_id = TICKER_ID_CONN_BASE + lll->handle;
	u32_t ticker_status;

	/* Enable Ticker Job, we are in a radio event which disabled it if
	 * worker0 and job0 priority where same.
	 */
	mayfly_enable(TICKER_USER_ID_ULL_HIGH, TICKER_USER_ID_ULL_LOW, 1);

	/* Stop Master or Slave role ticker */
	ticker_status = ticker_stop(TICKER_INSTANCE_ID_CTLR,
				    TICKER_USER_ID_ULL_HIGH,
				    ticker_id,
				    ticker_op_stop_cb, (void *)lll);
	LL_ASSERT((ticker_status == TICKER_STATUS_SUCCESS) ||
		  (ticker_status == TICKER_STATUS_BUSY));
}

static void ctrl_tx_data_last_enqueue(struct ll_conn *conn,
				      struct node_tx *node_tx)
{
	node_tx->next = conn->tx_ctrl_last->next;
	conn->tx_ctrl_last->next = node_tx;
	conn->tx_ctrl_last = node_tx;
}

static void ctrl_tx_enqueue(struct ll_conn *conn, struct node_tx *node_tx)
{
	/* check if a packet was tx-ed and not acked by peer */
	if (
	    /* data/ctrl packet is in the head */
	    conn->tx_head &&
	    /* data PDU tx is not paused */
	    !conn->pause_tx) {
		/* data or ctrl may have been transmitted once, but not acked
		 * by peer, hence place this new ctrl after head
		 */

		/* if data transmited once, keep it at head of the tx list,
		 * as we will insert a ctrl after it, hence advance the
		 * data pointer
		 */
		if (conn->tx_head == conn->tx_data) {
			conn->tx_data = conn->tx_data->next;
		}

		/* if no ctrl packet already queued, new ctrl added will be
		 * the ctrl pointer and is inserted after head.
		 */
		if (!conn->tx_ctrl) {
			node_tx->next = conn->tx_head->next;
			conn->tx_head->next = node_tx;
			conn->tx_ctrl = node_tx;
			conn->tx_ctrl_last = node_tx;
		} else {
			ctrl_tx_data_last_enqueue(conn, node_tx);
		}
	} else {
		/* No packet needing ACK. */

		/* If first ctrl packet then add it as head else add it to the
		 * tail of the ctrl packets.
		 */
		if (!conn->tx_ctrl) {
			node_tx->next = conn->tx_head;
			conn->tx_head = node_tx;
			conn->tx_ctrl = node_tx;
			conn->tx_ctrl_last = node_tx;
		} else {
			ctrl_tx_data_last_enqueue(conn, node_tx);
		}
	}

	/* Update last pointer if ctrl added at end of tx list */
	if (node_tx->next == 0) {
		conn->tx_data_last = node_tx;
	}
}

static void ctrl_tx_sec_enqueue(struct ll_conn *conn, struct node_tx *node_tx)
{
	if (conn->pause_tx) {
		if (!conn->tx_ctrl) {
			node_tx->next = conn->tx_head;
			conn->tx_head = node_tx;
		} else {
			node_tx->next = conn->tx_ctrl_last->next;
			conn->tx_ctrl_last->next = node_tx;
		}
	} else {
		ctrl_tx_enqueue(conn, node_tx);
	}
}

static u8_t unknown_rsp_send(struct ll_conn *conn, u8_t type)
{
	struct node_tx *node_tx;
	struct pdu_data *pdu;

	/* acquire ctrl tx mem */
	node_tx = mem_acquire(&mem_conn_tx_ctrl.free);
	if (!node_tx) {
		return 1;
	}

	pdu = (void *)node_tx->pdu;
	pdu->ll_id = PDU_DATA_LLID_CTRL;
	pdu->len = offsetof(struct pdu_data_llctrl, unknown_rsp) +
			   sizeof(struct pdu_data_llctrl_unknown_rsp);
	pdu->llctrl.opcode = PDU_DATA_LLCTRL_TYPE_UNKNOWN_RSP;
	pdu->llctrl.unknown_rsp.type = type;

	ctrl_tx_enqueue(conn, node_tx);

	return 0;
}

static inline void ctrl_tx_ack(struct lll_conn *lll, struct pdu_data *pdu)
{
	switch (pdu->llctrl.opcode) {
	case PDU_DATA_LLCTRL_TYPE_TERMINATE_IND:
	{
		u8_t reason = (pdu->llctrl.terminate_ind.error_code ==
			       BT_HCI_ERR_REMOTE_USER_TERM_CONN) ?
			      BT_HCI_ERR_LOCALHOST_TERM_CONN :
			      pdu->llctrl.terminate_ind.error_code;

		terminate_ind_rx_enqueue(lll, reason);
		conn_cleanup(lll);
	}
	break;

	default:
		/* Do nothing for other ctrl packet ack */
		break;
	}
}

static inline bool pdu_len_cmp(u8_t opcode, u8_t len)
{
	const u8_t ctrl_len_lut[] = {
		(offsetof(struct pdu_data_llctrl, conn_update_ind) +
		 sizeof(struct pdu_data_llctrl_conn_update_ind)),
		(offsetof(struct pdu_data_llctrl, chan_map_ind) +
		 sizeof(struct pdu_data_llctrl_chan_map_ind)),
		(offsetof(struct pdu_data_llctrl, terminate_ind) +
		 sizeof(struct pdu_data_llctrl_terminate_ind)),
		(offsetof(struct pdu_data_llctrl, enc_req) +
		 sizeof(struct pdu_data_llctrl_enc_req)),
		(offsetof(struct pdu_data_llctrl, enc_rsp) +
		 sizeof(struct pdu_data_llctrl_enc_rsp)),
		(offsetof(struct pdu_data_llctrl, start_enc_req) +
		 sizeof(struct pdu_data_llctrl_start_enc_req)),
		(offsetof(struct pdu_data_llctrl, start_enc_rsp) +
		 sizeof(struct pdu_data_llctrl_start_enc_rsp)),
		(offsetof(struct pdu_data_llctrl, unknown_rsp) +
		 sizeof(struct pdu_data_llctrl_unknown_rsp)),
		(offsetof(struct pdu_data_llctrl, feature_req) +
		 sizeof(struct pdu_data_llctrl_feature_req)),
		(offsetof(struct pdu_data_llctrl, feature_rsp) +
		 sizeof(struct pdu_data_llctrl_feature_rsp)),
		(offsetof(struct pdu_data_llctrl, pause_enc_req) +
		 sizeof(struct pdu_data_llctrl_pause_enc_req)),
		(offsetof(struct pdu_data_llctrl, pause_enc_rsp) +
		 sizeof(struct pdu_data_llctrl_pause_enc_rsp)),
		(offsetof(struct pdu_data_llctrl, version_ind) +
		 sizeof(struct pdu_data_llctrl_version_ind)),
		(offsetof(struct pdu_data_llctrl, reject_ind) +
		 sizeof(struct pdu_data_llctrl_reject_ind)),
		(offsetof(struct pdu_data_llctrl, slave_feature_req) +
		 sizeof(struct pdu_data_llctrl_slave_feature_req)),
		(offsetof(struct pdu_data_llctrl, conn_param_req) +
		 sizeof(struct pdu_data_llctrl_conn_param_req)),
		(offsetof(struct pdu_data_llctrl, conn_param_rsp) +
		 sizeof(struct pdu_data_llctrl_conn_param_rsp)),
		(offsetof(struct pdu_data_llctrl, reject_ext_ind) +
		 sizeof(struct pdu_data_llctrl_reject_ext_ind)),
		(offsetof(struct pdu_data_llctrl, ping_req) +
		 sizeof(struct pdu_data_llctrl_ping_req)),
		(offsetof(struct pdu_data_llctrl, ping_rsp) +
		 sizeof(struct pdu_data_llctrl_ping_rsp)),
		(offsetof(struct pdu_data_llctrl, length_req) +
		 sizeof(struct pdu_data_llctrl_length_req)),
		(offsetof(struct pdu_data_llctrl, length_rsp) +
		 sizeof(struct pdu_data_llctrl_length_rsp)),
		(offsetof(struct pdu_data_llctrl, phy_req) +
		 sizeof(struct pdu_data_llctrl_phy_req)),
		(offsetof(struct pdu_data_llctrl, phy_rsp) +
		 sizeof(struct pdu_data_llctrl_phy_rsp)),
		(offsetof(struct pdu_data_llctrl, phy_upd_ind) +
		 sizeof(struct pdu_data_llctrl_phy_upd_ind)),
		(offsetof(struct pdu_data_llctrl, min_used_chans_ind) +
		 sizeof(struct pdu_data_llctrl_min_used_chans_ind)),
	};

	return ctrl_len_lut[opcode] == len;
}

static inline u8_t ctrl_rx(u16_t handle, struct pdu_data *pdu)
{
	struct ll_conn *conn;
	u8_t nack = 0;
	u8_t opcode;

	conn = ll_conn_get(handle);
	LL_ASSERT(conn);

	opcode = pdu->llctrl.opcode;
	switch (opcode) {
	case PDU_DATA_LLCTRL_TYPE_TERMINATE_IND:
		if (!pdu_len_cmp(PDU_DATA_LLCTRL_TYPE_TERMINATE_IND,
				 pdu->len)) {
			goto ull_conn_rx_unknown_rsp_send;
		}

		/* Ack and then terminate */
		conn->llcp_terminate.reason_peer =
			pdu->llctrl.terminate_ind.error_code;
		break;

	default:
ull_conn_rx_unknown_rsp_send:
		nack = unknown_rsp_send(conn, opcode);
		break;
	}

	return nack;
}
