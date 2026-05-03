/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Unit Under Test (UUT): lll_prepare_resolve() and its static helpers.
 *
 * This file extracts only the logic needed to test lll_prepare_resolve()
 * from subsys/bluetooth/controller/ll_sw/nordic/lll/lll.c, replacing
 * hardware-specific includes with the portable equivalents available to
 * native_sim. All external symbol references (ticker_*, ull_prepare_*,
 * mayfly_enqueue) are fulfilled by FFF fakes defined in main.c.
 */

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>

#include <zephyr/types.h>
#include <zephyr/sys/util.h>
#include <zephyr/toolchain.h>

#include "util/memq.h"
#include "util/mayfly.h"

#include "ticker/ticker.h"

#include "lll.h"

#include "hal/debug.h"

/*
 * Event overhead constants - mirrors the values defined in lll_vendor.h
 * for a platform without HFXO DT node.
 */
#define EVENT_OVERHEAD_XTAL_US        1500U
#define EVENT_OVERHEAD_PREEMPT_MIN_US 0U

/*
 * HAL ticker constants - mirrors hal/nrf5/ticker.h for a 24-bit RTC counter.
 * They are used in preempt_ticker_start() to compute the preempt timeout.
 */
#define HAL_TICKER_CNTR_CLK_UNIT_FSEC 30517578125ULL
#define HAL_TICKER_FSEC_PER_USEC      1000000000ULL
#define HAL_TICKER_CNTR_MSBIT         23U
#define HAL_TICKER_CNTR_MASK          0x00FFFFFFU

#define HAL_TICKER_US_TO_TICKS(x) \
	(((uint32_t)(((uint64_t)(x) * HAL_TICKER_FSEC_PER_USEC) / \
		     HAL_TICKER_CNTR_CLK_UNIT_FSEC)) & HAL_TICKER_CNTR_MASK)

/* -------------------------------------------------------------------------
 * Internal event state (mirrors the static 'event' variable in lll.c).
 * -------------------------------------------------------------------------
 */
static struct {
	struct {
		void              *param;
		lll_is_abort_cb_t  is_abort_cb;
		lll_abort_cb_t     abort_cb;
		uint32_t           ticks_at_expire;
		uint32_t           remainder;
		uint16_t           lazy;
	} curr;
} event;

/* -------------------------------------------------------------------------
 * Preempt-ticker static state (mirrors the variables in lll.c,
 * !CONFIG_BT_CTLR_LOW_LAT section).
 * -------------------------------------------------------------------------
 */
static uint8_t volatile preempt_start_req;
static uint8_t          preempt_start_ack;
static uint8_t volatile preempt_stop_req;
static uint8_t          preempt_stop_ack;
static uint8_t          preempt_req;
static uint8_t volatile preempt_ack;

/* -------------------------------------------------------------------------
 * Forward declarations of static helpers (mirrors lll.c).
 * -------------------------------------------------------------------------
 */
static inline bool is_done_sync(void);
static inline struct lll_event *prepare_dequeue_iter_ready_get(void **idx);
static void     ticker_stop_op_cb(uint32_t status, void *param);
static void     ticker_start_op_cb(uint32_t status, void *param);
static uint32_t preempt_ticker_start(struct lll_event *first,
				     struct lll_event *prev,
				     struct lll_event *next);
static uint32_t preempt_ticker_stop(void);
static void     preempt_ticker_cb(uint32_t ticks_at_expire,
				  uint32_t ticks_drift,
				  uint32_t remainder, uint16_t lazy,
				  uint8_t force, void *param);
static void preempt(void *param);

/* -------------------------------------------------------------------------
 * Static helper implementations (extracted from lll.c).
 * -------------------------------------------------------------------------
 */

/* is_done_sync: without CONFIG_BT_CTLR_LOW_LAT_ULL_DONE the done counters
 * are not maintained, so the radio event is always considered "done" and
 * in-sync.
 */
static inline bool is_done_sync(void)
{
	return true;
}

/*
 * Iterate through the prepare pipeline and return the first event that is
 * ready: not aborted, not a resume, and not deferred.
 */
static inline struct lll_event *prepare_dequeue_iter_ready_get(void **idx)
{
	struct lll_event *ready;

	ready = ull_prepare_dequeue_iter(idx);
	while ((ready != NULL) &&
	       ((ready->is_aborted != 0U) || (ready->is_resume != 0U) ||
		(ready->prepare_param.defer != 0U))) {
		if (!*idx) {
			ready = NULL;
			break;
		}

		ready = ull_prepare_dequeue_iter(idx);
	}

	return ready;
}

static uint32_t preempt_ticker_stop(void)
{
	uint32_t ret;

	if ((preempt_stop_req != preempt_stop_ack) ||
	    (preempt_req == preempt_ack)) {
		return TICKER_STATUS_SUCCESS;
	}

	preempt_stop_req++;

	ret = ticker_stop(TICKER_INSTANCE_ID_CTLR,
			  TICKER_USER_ID_LLL,
			  TICKER_ID_LLL_PREEMPT,
			  ticker_stop_op_cb, NULL);

	return ret;
}

static void ticker_stop_op_cb(uint32_t status, void *param)
{
	ARG_UNUSED(param);

	LL_ASSERT_ERR(preempt_stop_req != preempt_stop_ack);
	preempt_stop_ack = preempt_stop_req;

	if (status == TICKER_STATUS_SUCCESS) {
		LL_ASSERT_ERR(preempt_req != preempt_ack);
	}

	preempt_req = preempt_ack;
}

static void ticker_start_op_cb(uint32_t status, void *param)
{
	ARG_UNUSED(param);
	LL_ASSERT_ERR(status == TICKER_STATUS_SUCCESS);

	LL_ASSERT_ERR(preempt_req == preempt_ack);
	preempt_req++;

	LL_ASSERT_ERR(preempt_start_req != preempt_start_ack);
	preempt_start_ack = preempt_start_req;
}

static uint32_t preempt_ticker_start(struct lll_event *first,
				     struct lll_event *prev,
				     struct lll_event *next)
{
	const struct lll_prepare_param *p;
	static uint32_t ticks_at_preempt;
	uint32_t ticks_at_preempt_new;
	uint32_t preempt_anchor;
	uint32_t preempt_to;
	uint32_t ret;

	if ((preempt_start_req != preempt_start_ack) ||
	    (preempt_req != preempt_ack)) {
		uint32_t diff;

		p = &next->prepare_param;
		preempt_anchor = p->ticks_at_expire;
		preempt_to = HAL_TICKER_US_TO_TICKS(EVENT_OVERHEAD_XTAL_US) -
			     HAL_TICKER_US_TO_TICKS(EVENT_OVERHEAD_PREEMPT_MIN_US);

		ticks_at_preempt_new = preempt_anchor + preempt_to;
		ticks_at_preempt_new &= HAL_TICKER_CNTR_MASK;

		diff = ticker_ticks_diff_get(ticks_at_preempt_new,
					     ticks_at_preempt);
		if ((diff & BIT(HAL_TICKER_CNTR_MSBIT)) == 0U) {
			return TICKER_STATUS_SUCCESS;
		}

		ret = preempt_ticker_stop();
		LL_ASSERT_ERR((ret == TICKER_STATUS_SUCCESS) ||
			      (ret == TICKER_STATUS_BUSY));

		first = next;
	} else {
		p = &first->prepare_param;
		preempt_anchor = p->ticks_at_expire;
		preempt_to = HAL_TICKER_US_TO_TICKS(EVENT_OVERHEAD_XTAL_US) -
			     HAL_TICKER_US_TO_TICKS(EVENT_OVERHEAD_PREEMPT_MIN_US);

		ticks_at_preempt_new = preempt_anchor + preempt_to;
		ticks_at_preempt_new &= HAL_TICKER_CNTR_MASK;
	}

	preempt_start_req++;

	ticks_at_preempt = ticks_at_preempt_new;

	ret = ticker_start(TICKER_INSTANCE_ID_CTLR,
			   TICKER_USER_ID_LLL,
			   TICKER_ID_LLL_PREEMPT,
			   preempt_anchor,
			   preempt_to,
			   TICKER_NULL_PERIOD,
			   TICKER_NULL_REMAINDER,
			   TICKER_NULL_LAZY,
			   TICKER_NULL_SLOT,
			   preempt_ticker_cb, first->prepare_param.param,
			   ticker_start_op_cb, NULL);

	return ret;
}

static void preempt_ticker_cb(uint32_t ticks_at_expire, uint32_t ticks_drift,
			      uint32_t remainder, uint16_t lazy, uint8_t force,
			      void *param)
{
	static memq_link_t link;
	static struct mayfly mfy = {0, 0, &link, NULL, preempt};
	uint32_t ret;

	LL_ASSERT_ERR(preempt_ack != preempt_req);
	preempt_ack = preempt_req;

	mfy.param = param;
	ret = mayfly_enqueue(TICKER_USER_ID_ULL_HIGH, TICKER_USER_ID_LLL,
			     0U, &mfy);
	LL_ASSERT_ERR(!ret);
}

static void preempt(void *param)
{
	ull_prepare_dequeue(TICKER_USER_ID_LLL);
}

/* -------------------------------------------------------------------------
 * lll_prepare_resolve - the function under test.
 *
 * Extracted verbatim from subsys/bluetooth/controller/ll_sw/nordic/lll/lll.c
 * (CONFIG_BT_CTLR_LOW_LAT not set, CONFIG_BT_CTLR_LOW_LAT_ULL_DONE not set,
 *  CONFIG_BT_CTLR_LLL_PREPARE_AT_MARGIN not set).
 * -------------------------------------------------------------------------
 */
int lll_prepare_resolve(lll_is_abort_cb_t is_abort_cb, lll_abort_cb_t abort_cb,
			lll_prepare_cb_t prepare_cb,
			struct lll_prepare_param *prepare_param,
			uint8_t is_resume, uint8_t is_dequeue)
{
	struct lll_event *ready_short;
	struct lll_event *ready;
	struct lll_event *next;
	void *idx;
	int err;

	/* Find the ready prepare in the pipeline */
	idx = NULL;
	ready = prepare_dequeue_iter_ready_get(&idx);

	/* Find any short prepare */
	if ((ready != NULL) && (&ready->prepare_param != prepare_param)) {
		uint32_t diff;

		diff = ticker_ticks_diff_get(prepare_param->ticks_at_expire,
					     ready->prepare_param.ticks_at_expire);
		if ((is_resume != 0U) ||
		    ((diff != 0U) &&
		     ((diff & BIT(HAL_TICKER_CNTR_MSBIT)) == 0U))) {
			ready_short = ready;
		} else {
			ready_short = NULL;
			ready = NULL;
		}
	} else {
		ready_short = NULL;
		ready = NULL;
	}

	/* Current event active or another prepare is ready in the pipeline */
	if (((is_dequeue == 0U) && (is_done_sync() == 0U)) ||
	    (event.curr.abort_cb != NULL) ||
	    (ready_short != NULL) ||
	    ((ready != NULL) && (is_resume != 0U))) {
		/* Store the next prepare for deferred call */
		next = ull_prepare_enqueue(is_abort_cb, abort_cb, prepare_param,
					   prepare_cb, is_resume);
		LL_ASSERT_ERR(next);

		if (is_resume || prepare_param->defer) {
			return -EINPROGRESS;
		}

		/* Always start preempt timeout for first prepare in pipeline */
		struct lll_event *first = ready ? ready : next;
		uint32_t ret;

		ret = preempt_ticker_start(first, ready, next);
		LL_ASSERT_ERR((ret == TICKER_STATUS_SUCCESS) ||
			      (ret == TICKER_STATUS_BUSY));

		return -EINPROGRESS;
	}

	LL_ASSERT_ERR(!ready || &ready->prepare_param == prepare_param);

	event.curr.param = prepare_param->param;
	event.curr.ticks_at_expire = prepare_param->ticks_at_expire;
	event.curr.remainder = prepare_param->remainder;
	event.curr.lazy = prepare_param->lazy;
	event.curr.is_abort_cb = is_abort_cb;
	event.curr.abort_cb = abort_cb;

	err = prepare_cb(prepare_param);

	if (!IS_ENABLED(CONFIG_BT_CTLR_ASSERT_OVERHEAD_START) &&
	    (err == -ECANCELED)) {
		err = 0;
	}

	/* Find next prepare needing preempt timeout to be setup */
	idx = NULL;
	next = prepare_dequeue_iter_ready_get(&idx);
	if (!next) {
		return err;
	}

	/* Start the preempt timeout */
	uint32_t ret;

	ret = preempt_ticker_start(next, NULL, next);
	LL_ASSERT_ERR((ret == TICKER_STATUS_SUCCESS) ||
		      (ret == TICKER_STATUS_BUSY));

	return err;
}

/* -------------------------------------------------------------------------
 * Test-visible helpers to manipulate internal state between test cases.
 * -------------------------------------------------------------------------
 */

void lll_prepare_resolve_test_reset(void)
{
	event.curr.param        = NULL;
	event.curr.is_abort_cb  = NULL;
	event.curr.abort_cb     = NULL;
	event.curr.ticks_at_expire = 0U;
	event.curr.remainder    = 0U;
	event.curr.lazy         = 0U;

	preempt_start_req  = 0U;
	preempt_start_ack  = 0U;
	preempt_stop_req   = 0U;
	preempt_stop_ack   = 0U;
	preempt_req        = 0U;
	preempt_ack        = 0U;
}

void lll_prepare_resolve_test_set_curr(lll_abort_cb_t abort_cb,
				       lll_is_abort_cb_t is_abort_cb,
				       void *param)
{
	event.curr.abort_cb    = abort_cb;
	event.curr.is_abort_cb = is_abort_cb;
	event.curr.param       = param;
}

void *lll_prepare_resolve_test_get_curr_param(void)
{
	return event.curr.param;
}

lll_abort_cb_t lll_prepare_resolve_test_get_curr_abort_cb(void)
{
	return event.curr.abort_cb;
}
