/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stddef.h>
#include <stdbool.h>
#include <toolchain.h>
#include <zephyr/types.h>
#include <misc/util.h>

#include "hal/ticker.h"
#include "hal/ccm.h"
#include "util/memq.h"
#include "util/mayfly.h"
#include "ticker/ticker.h"

#include "util/util.h"

#include "pdu.h"

#include "lll.h"
#include "lll_vendor.h"
#include "lll_adv.h"
#include "lll_conn.h"
#include "lll_slave.h"
#include "lll_tim_internal.h"

#include "ull_adv_types.h"
#include "ull_conn_types.h"

#include "ull_internal.h"
#include "ull_adv_internal.h"
#include "ull_conn_internal.h"

#include "common/log.h"
#include <soc.h>
#include "hal/debug.h"

static void ticker_op_stop_adv_cb(u32_t status, void *params);
static void ticker_op_cb(u32_t status, void *params);
static void ticker_cb(u32_t ticks_at_expire, u32_t remainder, u16_t lazy,
		      void *param);

void ull_slave_setup(memq_link_t *link, struct node_rx_hdr *rx,
		     struct node_rx_ftr *ftr, struct lll_conn *lll)
{
	u32_t conn_offset_us, conn_interval_us;
	u8_t ticker_id_adv, ticker_id_conn;
	u8_t peer_addr[BDADDR_SIZE];
	u32_t ticks_slot_overhead;
	u32_t mayfly_was_enabled;
	u32_t ticks_slot_offset;
	struct pdu_adv *pdu_adv;
	struct ll_adv_set *adv;
	struct node_rx_cc *cc;
	struct ll_conn *conn;
	u32_t ticker_status;
	u8_t peer_addr_type;
	u16_t win_offset;
	u16_t timeout;

	((struct lll_adv *)ftr->param)->conn = NULL;

	adv = ((struct lll_adv *)ftr->param)->hdr.parent;
	conn = lll->hdr.parent;

	/* Populate the slave context */
	pdu_adv = (void *)((struct node_rx_pdu *)rx)->pdu;
	memcpy(&lll->crc_init[0], &pdu_adv->connect_ind.crc_init[0], 3);
	memcpy(&lll->access_addr[0], &pdu_adv->connect_ind.access_addr[0], 4);
	memcpy(&lll->data_chan_map[0], &pdu_adv->connect_ind.chan_map[0],
	       sizeof(lll->data_chan_map));
	lll->data_chan_count = util_ones_count_get(&lll->data_chan_map[0],
			       sizeof(lll->data_chan_map));
	lll->data_chan_hop = pdu_adv->connect_ind.hop;
	lll->data_chan_sel = 0;
	lll->interval = pdu_adv->connect_ind.interval;
	lll->latency = pdu_adv->connect_ind.latency;

	win_offset = pdu_adv->connect_ind.win_offset;
	conn_interval_us = pdu_adv->connect_ind.interval * 1250;

	/* calculate the window widening */
	lll->slave.sca = pdu_adv->connect_ind.sca;
	lll->slave.window_widening_periodic_us =
		(((lll_conn_ppm_local_get() +
		   lll_conn_ppm_get(lll->slave.sca)) *
		  conn_interval_us) + (1000000 - 1)) / 1000000;
	lll->slave.window_widening_max_us = (conn_interval_us >> 1) - TIFS_US;
	lll->slave.window_size_event_us = pdu_adv->connect_ind.win_size * 1250;

	/* procedure timeouts */
	lll->supervision_reload =
		RADIO_CONN_EVENTS((pdu_adv->connect_ind.timeout * 10 * 1000),
				  conn_interval_us);
	lll->procedure_reload =
		RADIO_CONN_EVENTS((40 * 1000 * 1000), conn_interval_us);

#if defined(CONFIG_BT_CTLR_LE_PING)
	/* APTO in no. of connection events */
	lll->apto_reload =
		RADIO_CONN_EVENTS((30 * 1000 * 1000), conn_interval_us);
	/* Dispatch LE Ping PDU 6 connection events (that peer would
	 * listen to) before 30s timeout
	 * TODO: "peer listens to" is greater than 30s due to latency
	 */
	lll->appto_reload = (lll->apto_reload > (lll->latency + 6)) ?
			     (lll->apto_reload - (lll->latency + 6)) :
			     lll->apto_reload;
#endif /* CONFIG_BT_CTLR_LE_PING */

	memcpy((void *)&lll->slave.force, &lll->access_addr[0],
	       sizeof(lll->slave.force));

	peer_addr_type = pdu_adv->tx_addr;
	memcpy(peer_addr, pdu_adv->connect_ind.init_addr, BDADDR_SIZE);
	timeout = pdu_adv->connect_ind.timeout;

	cc = (void *)pdu_adv;
	cc->status = 0;
	cc->role = 1;
	cc->peer_addr_type = peer_addr_type;
	memcpy(cc->peer_addr, peer_addr, BDADDR_SIZE);
	cc->interval = lll->interval;
	cc->latency = lll->latency;
	cc->timeout = timeout;
	cc->sca = lll->slave.sca;

	lll->handle = ll_conn_handle_get(conn);
	rx->handle = lll->handle;

	ll_rx_put(link, rx);
	ll_rx_sched();
#if 0
		/* Prepare the rx packet structure */
		node_rx->hdr.handle = conn->handle;
		node_rx->hdr.type = NODE_RX_TYPE_CONNECTION;

		/* prepare connection complete structure */
		pdu_data = (void *)node_rx->pdu_data;
		radio_le_conn_cmplt = (void *)pdu_data->lldata;
		radio_le_conn_cmplt->status = 0x00;
		radio_le_conn_cmplt->role = 0x01;
#if defined(CONFIG_BT_CTLR_PRIVACY)
		radio_le_conn_cmplt->own_addr_type = pdu_adv->rx_addr;
		memcpy(&radio_le_conn_cmplt->own_addr[0],
		       &pdu_adv->connect_ind.adv_addr[0], BDADDR_SIZE);
		if (rl_idx != FILTER_IDX_NONE) {
			/* TODO: store rl_idx instead if safe */
			/* Store identity address */
			ll_rl_id_addr_get(rl_idx,
					  &radio_le_conn_cmplt->peer_addr_type,
					  &radio_le_conn_cmplt->peer_addr[0]);
			/* Mark it as identity address from RPA (0x02, 0x03) */
			radio_le_conn_cmplt->peer_addr_type += 2;

			/* Store peer RPA */
			memcpy(&radio_le_conn_cmplt->peer_rpa[0],
			       &pdu_adv->connect_ind.init_addr[0],
			       BDADDR_SIZE);
		} else {
			memset(&radio_le_conn_cmplt->peer_rpa[0], 0x0,
			       BDADDR_SIZE);
#else
		if (1) {
#endif /* CONFIG_BT_CTLR_PRIVACY */
			radio_le_conn_cmplt->peer_addr_type = pdu_adv->tx_addr;
			memcpy(&radio_le_conn_cmplt->peer_addr[0],
			       &pdu_adv->connect_ind.init_addr[0],
			       BDADDR_SIZE);
		}

		radio_le_conn_cmplt->interval =
			pdu_adv->connect_ind.interval;
		radio_le_conn_cmplt->latency =
			pdu_adv->connect_ind.latency;
		radio_le_conn_cmplt->timeout =
			pdu_adv->connect_ind.timeout;
		radio_le_conn_cmplt->mca =
			pdu_adv->connect_ind.sca;

		/* enqueue connection complete structure into queue */
		rx_fc_lock(conn->handle);
		packet_rx_enqueue();

		/* Use Channel Selection Algorithm #2 if peer too supports it */
		if (IS_ENABLED(CONFIG_BT_CTLR_CHAN_SEL_2)) {
			struct radio_le_chan_sel_algo *le_chan_sel_algo;

			/* Generate LE Channel Selection Algorithm event */
			node_rx = packet_rx_reserve_get(3);
			LL_ASSERT(node_rx);

			node_rx->hdr.handle = conn->handle;
			node_rx->hdr.type = NODE_RX_TYPE_CHAN_SEL_ALGO;

			pdu_data = (void *)node_rx->pdu_data;
			le_chan_sel_algo = (void *)pdu_data->lldata;

			if (pdu_adv->chan_sel) {
				u16_t aa_ls =
					((u16_t)conn->access_addr[1] << 8) |
					conn->access_addr[0];
				u16_t aa_ms =
					((u16_t)conn->access_addr[3] << 8) |
					 conn->access_addr[2];

				conn->data_chan_sel = 1;
				conn->data_chan_id = aa_ms ^ aa_ls;

				le_chan_sel_algo->chan_sel_algo = 0x01;
			} else {
				le_chan_sel_algo->chan_sel_algo = 0x00;
			}

			packet_rx_enqueue();
		}
#endif

	/* TODO: active_to_start feature port */
	conn->evt.ticks_active_to_start = 0;
	conn->evt.ticks_xtal_to_start =
		HAL_TICKER_US_TO_TICKS(EVENT_OVERHEAD_XTAL_US);
	conn->evt.ticks_preempt_to_start =
		HAL_TICKER_US_TO_TICKS(EVENT_OVERHEAD_PREEMPT_MIN_US);
	conn->evt.ticks_slot =
		HAL_TICKER_US_TO_TICKS(EVENT_OVERHEAD_START_US +
				       ftr->us_radio_rdy + 328 + TIFS_US +
				       328);

	ticks_slot_offset = max(conn->evt.ticks_active_to_start,
				conn->evt.ticks_xtal_to_start);

	if (IS_ENABLED(CONFIG_BT_CTLR_LOW_LAT)) {
		ticks_slot_overhead = ticks_slot_offset;
	} else {
		ticks_slot_overhead = 0;
	}

	conn_interval_us -= lll->slave.window_widening_periodic_us;

	conn_offset_us = ftr->us_radio_end;
	conn_offset_us += ((u64_t)win_offset + 1) * 1250;
	conn_offset_us -= EVENT_OVERHEAD_START_US;
	conn_offset_us -= EVENT_JITTER_US << 1;
	conn_offset_us -= EVENT_JITTER_US;
	conn_offset_us -= ftr->us_radio_rdy;

	/* disable ticker job, in order to chain stop and start to avoid RTC
	 * being stopped if no tickers active.
	 */
#if (CONFIG_BT_CTLR_ULL_HIGH_PRIO == CONFIG_BT_CTLR_ULL_LOW_PRIO)
	mayfly_was_enabled = mayfly_is_enabled(TICKER_USER_ID_ULL_HIGH,
					       TICKER_USER_ID_ULL_LOW);
	mayfly_enable(TICKER_USER_ID_ULL_HIGH, TICKER_USER_ID_ULL_LOW, 0);
#endif

	/* Stop Advertiser */
	ticker_id_adv = TICKER_ID_ADV_BASE + ull_adv_handle_get(adv);
	ticker_status = ticker_stop(TICKER_INSTANCE_ID_CTLR,
				    TICKER_USER_ID_ULL_HIGH,
				    ticker_id_adv, ticker_op_stop_adv_cb,
				    (void *)(u32_t)ticker_id_adv);
	ticker_op_stop_adv_cb(ticker_status, (void *)(u32_t)ticker_id_adv);

	/* TODO: Stop Direct Adv Stop */
	#if 0
	if (_pdu_adv->type == PDU_ADV_TYPE_DIRECT_IND) {
		/* Advertiser stop can expire while here in this ISR.
		 * Deferred attempt to stop can fail as it would have
		 * expired, hence ignore failure.
		 */
		ticker_stop(RADIO_TICKER_INSTANCE_ID_RADIO,
			    RADIO_TICKER_USER_ID_WORKER,
			    RADIO_TICKER_ID_ADV_STOP, NULL, NULL);
	}
	#endif

	/* Start Slave */
	ticker_id_conn = TICKER_ID_CONN_BASE + ll_conn_handle_get(conn);
	ticker_status = ticker_start(TICKER_INSTANCE_ID_CTLR,
				     TICKER_USER_ID_ULL_HIGH,
				     ticker_id_conn,
				     ftr->ticks_anchor - ticks_slot_offset,
				     HAL_TICKER_US_TO_TICKS(conn_offset_us),
				     HAL_TICKER_US_TO_TICKS(conn_interval_us),
				     HAL_TICKER_REMAINDER(conn_interval_us),
				     TICKER_NULL_LAZY,
				     (conn->evt.ticks_slot +
				      ticks_slot_overhead),
				     ticker_cb, conn, ticker_op_cb,
				     (void *)__LINE__);
	LL_ASSERT((ticker_status == TICKER_STATUS_SUCCESS) ||
		  (ticker_status == TICKER_STATUS_BUSY));

#if (CONFIG_BT_CTLR_ULL_HIGH_PRIO == CONFIG_BT_CTLR_ULL_LOW_PRIO)
	/* enable ticker job, if disabled in this function */
	if (mayfly_was_enabled) {
		mayfly_enable(TICKER_USER_ID_ULL_HIGH, TICKER_USER_ID_ULL_LOW,
			      1);
	}
#else
	ARG_UNUSED(mayfly_was_enabled);
#endif
}

void ull_slave_done(struct node_rx_event_done *done, u32_t *ticks_drift_plus,
		    u32_t *ticks_drift_minus)
{
	u32_t start_to_address_expected_us;
	u32_t start_to_address_actual_us;
	u32_t window_widening_event_us;
	u32_t preamble_to_addr_us;

	start_to_address_actual_us =
		done->extra.slave.start_to_address_actual_us;
	window_widening_event_us =
		done->extra.slave.window_widening_event_us;
	preamble_to_addr_us =
		done->extra.slave.preamble_to_addr_us;

	start_to_address_expected_us = EVENT_JITTER_US +
				       (EVENT_JITTER_US << 1) +
				       window_widening_event_us +
				       preamble_to_addr_us;

	if (start_to_address_actual_us <= start_to_address_expected_us) {
		*ticks_drift_plus =
			HAL_TICKER_US_TO_TICKS(window_widening_event_us);
		*ticks_drift_minus =
			HAL_TICKER_US_TO_TICKS((start_to_address_expected_us -
					       start_to_address_actual_us));
	} else {
		*ticks_drift_plus =
			HAL_TICKER_US_TO_TICKS(start_to_address_actual_us);
		*ticks_drift_minus =
			HAL_TICKER_US_TO_TICKS(EVENT_JITTER_US +
					       (EVENT_JITTER_US << 1) +
					       preamble_to_addr_us);
	}
}

#if defined(CONFIG_BT_CTLR_LE_ENC)
u8_t ll_start_enc_req_send(u16_t handle, u8_t error_code,
			    u8_t const *const ltk)
{
	return 0;
}
#endif /* CONFIG_BT_CTLR_LE_ENC */

static void ticker_op_stop_adv_cb(u32_t status, void *params)
{
	/* TODO: */
	#if 0
	ARG_UNUSED(params);

	if (status == TICKER_STATUS_FAILURE) {
		if (_radio.ticker_id_stop == RADIO_TICKER_ID_ADV) {
			/* ticker_stop failed due to race condition
			 * while in role_disable. Let the role_disable
			 * be made aware of, so it can return failure
			 * (to stop Adv role as it is now transitioned
			 * to Slave role).
			 */
			_radio.ticker_id_stop = 0;
		} else {
			LL_ASSERT(0);
		}
	} else {
		/* This assert shall not happen if advertiser role's slot
		 * calculation is correct, and next event shall not
		 * overlap/pre-empt the current advertise role event.
		 */
		LL_ASSERT(_radio.ticker_id_prepare != RADIO_TICKER_ID_ADV);
	}
	#endif
}

static void ticker_op_cb(u32_t status, void *params)
{
	ARG_UNUSED(params);

	LL_ASSERT(status == TICKER_STATUS_SUCCESS);
}

static void ticker_cb(u32_t ticks_at_expire, u32_t remainder, u16_t lazy,
		      void *param)
{
	static memq_link_t _link;
	static struct mayfly _mfy = {0, 0, &_link, NULL, lll_slave_prepare};
	static struct lll_prepare_param p;
	struct ll_conn *conn = param;
	struct lll_conn *lll;
	u32_t ret;
	u8_t ref;

	DEBUG_RADIO_PREPARE_S(1);

	/* Increment prepare reference count */
	ref = ull_ref_inc(&conn->ull);
	LL_ASSERT(ref);

	lll = &conn->lll;

	/* Append timing parameters */
	p.ticks_at_expire = ticks_at_expire;
	p.remainder = remainder;
	p.lazy = lazy;
	p.param = lll;
	_mfy.param = &p;

	/* Kick LLL prepare */
	ret = mayfly_enqueue(TICKER_USER_ID_ULL_HIGH, TICKER_USER_ID_LLL,
			     0, &_mfy);
	LL_ASSERT(!ret);

	DEBUG_RADIO_PREPARE_S(1);
}
