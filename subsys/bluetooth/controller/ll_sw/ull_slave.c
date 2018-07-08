/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stddef.h>
#include <stdbool.h>
#include <zephyr/types.h>
#include <toolchain.h>

#include "hal/ticker.h"
#include "hal/ccm.h"
#include "util/memq.h"
#include "util/mayfly.h"
#include "ticker/ticker.h"

#include "pdu.h"

#include "ull_types.h"

#include "lll.h"
#include "lll_conn.h"
#include "lll_slave.h"

#include "ull_conn_types.h"
#include "ull_internal.h"

#include "common/log.h"
#include <soc.h>
#include "hal/debug.h"

static void ticker_op_stop_adv_cb(u32_t status, void *params);
static void ticker_op_cb(u32_t status, void *params);
static void ticker_cb(u32_t ticks_at_expire, u32_t remainder, u16_t lazy,
		      void *param);

void ull_slave_setup(memq_link_t *link, struct node_rx_hdr *rx)
{
	u32_t ticker_status, ticker_anchor, ticks_slot;
	u32_t conn_offset_us, conn_interval_us;
	u8_t ticker_id_adv, ticker_id_conn;
	u16_t handle_adv, handle_conn;
	struct ll_conn *conn;

	/* Stop Advertiser */
	ticker_id_adv = TICKER_ID_ADV_BASE + handle_adv;
	ticker_status = ticker_stop(TICKER_INSTANCE_ID_CTLR,
				    TICKER_USER_ID_ULL_HIGH,
				    ticker_id_adv, ticker_op_stop_adv_cb,
				    (void *)(u32_t)ticker_id_adv);
	ticker_op_stop_adv_cb(ticker_status, (void *)(u32_t)ticker_id_adv);

	/* TODO: Stop Direct Adv Stopper */
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
	ticker_id_conn = TICKER_ID_CONN_BASE + handle_conn;
	ticker_status = ticker_start(TICKER_INSTANCE_ID_CTLR,
				     TICKER_USER_ID_ULL_HIGH,
				     ticker_id_conn,
				     ticker_anchor,
				     HAL_TICKER_US_TO_TICKS(conn_offset_us),
				     HAL_TICKER_US_TO_TICKS(conn_interval_us),
				     HAL_TICKER_REMAINDER(conn_interval_us),
				     TICKER_NULL_LAZY, ticks_slot, ticker_cb,
				     conn, ticker_op_cb,
				     (void *)__LINE__);
	LL_ASSERT((ticker_status == TICKER_STATUS_SUCCESS) ||
		  (ticker_status == TICKER_STATUS_BUSY));
}

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

