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

static struct ll_conn _conn[CONFIG_BT_MAX_CONN];
static void *_conn_free;

#define CONN_TX_BUF_SIZE (MROUND(offsetof(struct node_tx, pdu) + \
				 offsetof(struct pdu_data, lldata) + \
				 (CONFIG_BT_CTLR_TX_BUFFER_SIZE)) * \
			  (CONFIG_BT_CTLR_TX_BUFFERS))

static MFIFO_DEFINE(conn_tx, sizeof(struct lll_tx),
		    CONFIG_BT_CTLR_TX_BUFFERS);

static struct {
	void *free;
	u8_t pool[CONN_TX_BUF_SIZE * CONFIG_BT_CTLR_TX_BUFFERS];
} mem_conn_tx;

static struct {
	void *free;
	u8_t pool[sizeof(memq_link_t) * CONFIG_BT_CTLR_TX_BUFFERS];
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

	conn->lll.llcp_terminate.reason_own = reason;

	conn->lll.llcp_terminate.req++;

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

void ull_conn_done(struct node_rx_event_done *done)
{
	struct lll_conn *lll = (void *)HDR_ULL2LLL(done->param);
	u32_t ticks_drift_minus;
	u32_t ticks_drift_plus;
	u16_t latency_event;
	u16_t elapsed_event;
	u16_t trx_cnt;
	u16_t lazy;
	u8_t force;

	trx_cnt = done->extra.trx_cnt;

	ticks_drift_plus = 0;
	ticks_drift_minus = 0;
	latency_event = lll->latency_event;
	elapsed_event = latency_event + 1;

	if (trx_cnt) {
		if (IS_ENABLED(CONFIG_BT_PERIPHERAL) && lll->role) {
			ull_slave_done(done, &ticks_drift_plus,
				       &ticks_drift_minus);
		}

		/* Reset connection failed to establish countdown */
		lll->connect_expire = 0;

		/* Reset supervision countdown */
		lll->supervision_expire = 0;
	}

	/* check connection failed to establish */
	else if (lll->connect_expire) {
		if (lll->connect_expire > elapsed_event) {
			lll->connect_expire -= elapsed_event;
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
		if (!lll->supervision_expire) {
			lll->supervision_expire = lll->supervision_reload;
		}
	}

	/* check supervision timeout */
	force = 0;
	if (lll->supervision_expire) {
		if (lll->supervision_expire > elapsed_event) {
			lll->supervision_expire -= elapsed_event;

			/* break latency */
			lll->latency_event = 0;

			/* Force both master and slave when close to
			 * supervision timeout.
			 */
			if (lll->supervision_expire <= 6) {
				force = 1;
			}
			/* use randomness to force slave role when anchor
			 * points are being missed.
			 */
			else if (lll->role) {
				if (latency_event != 0) {
					force = 1;
				} else {
					force = lll->slave.force & 0x01;

					/* rotate force bits */
					lll->slave.force >>= 1;
					if (force) {
						lll->slave.force |= BIT(31);
					}
				}
			}
		} else {
			terminate_ind_rx_enqueue(lll, 0x08);

			conn_cleanup(lll);

			return;
		}
	}

	/* check procedure timeout */
	if (lll->procedure_expire != 0) {
		if (lll->procedure_expire > elapsed_event) {
			lll->procedure_expire -= elapsed_event;
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
	if ((lll->llcp_ack != lll->llcp_req) &&
	    ((lll->llcp_type == LLCP_CONN_UPD) ||
	     (lll->llcp_type == LLCP_CHAN_MAP))) {
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
			memq_link_t *link;

			link = mem_acquire(&mem_link_tx.free);
			LL_ASSERT(link);

			memq_enqueue(link, tx->node, &conn->lll.memq_tx.tail);
		} else {
			struct node_tx *node_tx = tx->node;
			struct pdu_data *p = (void *)node_tx->pdu;

			p->ll_id = PDU_DATA_LLID_RESV;
			ull_tx_ack_put(tx->handle, tx->node);
		}

		MFIFO_DEQUEUE(conn_tx);
	} while (--count);
}

void ull_conn_link_tx_release(void *link)
{
	mem_release(link, &mem_link_tx.free);
}

static int _init_reset(void)
{
	/* Initialize conn pool. */
	mem_init(_conn, sizeof(struct ll_conn),
		 sizeof(_conn) / sizeof(struct ll_conn), &_conn_free);

	/* Initialize tx pool. */
	mem_init(mem_conn_tx.pool, CONN_TX_BUF_SIZE, CONFIG_BT_CTLR_TX_BUFFERS,
		 &mem_conn_tx.free);

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
	struct node_rx_pdu *rx;
	memq_link_t *link;

	/* Prepare the rx packet structure */
	rx = (void *)&lll->llcp_terminate.node_rx;
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
