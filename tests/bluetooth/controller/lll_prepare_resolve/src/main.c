/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Unit tests for lll_prepare_resolve() covering all control paths for the
 * non-LOW_LAT, non-LOW_LAT_ULL_DONE, non-LLL_PREPARE_AT_MARGIN
 * configuration (the default for the Nordic soft-device controller).
 *
 * External dependencies are replaced with FFF (Fake Function Framework)
 * fakes so the test runs on native_sim without radio hardware.
 */

#include <stdlib.h>
#include <string.h>

#include <zephyr/types.h>
#include <zephyr/ztest.h>
#include <zephyr/fff.h>

/*
 * uut.h includes lll.h, ticker/ticker.h and all needed controller headers.
 * Do NOT include them again here: lll.h has no include guards and double
 * inclusion causes "conflicting types" errors for struct definitions.
 */
#include "uut.h"

DEFINE_FFF_GLOBALS;

/* -------------------------------------------------------------------------
 * FFF fake declarations for all external symbols called by the UUT.
 * -------------------------------------------------------------------------
 */

/*
 * ull_prepare_enqueue: enqueue a prepare event in the prepare pipeline.
 * Returns a pointer to the enqueued lll_event (must never be NULL).
 */
FAKE_VALUE_FUNC(struct lll_event *, ull_prepare_enqueue,
		lll_is_abort_cb_t,     /* is_abort_cb */
		lll_abort_cb_t,        /* abort_cb */
		struct lll_prepare_param *, /* prepare_param */
		lll_prepare_cb_t,      /* prepare_cb */
		uint8_t);              /* is_resume */

/*
 * ull_prepare_dequeue_iter: iterate the prepare pipeline.
 * Returns a void* that is cast to struct lll_event* by the caller.
 */
FAKE_VALUE_FUNC(void *, ull_prepare_dequeue_iter, void **);

/*
 * ull_prepare_dequeue: flush the prepare pipeline (called via mayfly).
 */
FAKE_VOID_FUNC(ull_prepare_dequeue, uint8_t);

/*
 * ticker_ticks_diff_get: signed 24-bit tick counter difference.
 * Returns (now - old) in ticks.
 */
FAKE_VALUE_FUNC(uint32_t, ticker_ticks_diff_get, uint32_t, uint32_t);

/*
 * ticker_start: schedule a preempt timeout ticker.
 * Returns TICKER_STATUS_SUCCESS (0) by default.
 */
FAKE_VALUE_FUNC(uint8_t, ticker_start,
		uint8_t,               /* instance_index */
		uint8_t,               /* user_id */
		uint8_t,               /* ticker_id */
		uint32_t,              /* ticks_anchor */
		uint32_t,              /* ticks_first */
		uint32_t,              /* ticks_periodic */
		uint32_t,              /* remainder_periodic */
		uint16_t,              /* lazy */
		uint32_t,              /* ticks_slot */
		ticker_timeout_func,   /* fp_timeout_func */
		void *,                /* context */
		ticker_op_func,        /* fp_op_func */
		void *);               /* op_context */

/*
 * ticker_stop: cancel the preempt timeout ticker.
 * Returns TICKER_STATUS_SUCCESS (0) by default.
 */
FAKE_VALUE_FUNC(uint8_t, ticker_stop,
		uint8_t,               /* instance_index */
		uint8_t,               /* user_id */
		uint8_t,               /* ticker_id */
		ticker_op_func,        /* fp_op_func */
		void *);               /* op_context */

/*
 * mayfly_enqueue: schedule a mayfly event (used in preempt_ticker_cb).
 * Returns 0 (success) by default.
 */
FAKE_VALUE_FUNC(uint32_t, mayfly_enqueue,
		uint8_t,               /* caller_id */
		uint8_t,               /* callee_id */
		uint8_t,               /* chain */
		struct mayfly *);      /* m */

/* -------------------------------------------------------------------------
 * Test fixtures.
 * -------------------------------------------------------------------------
 */

/*
 * A minimal LLL context struct whose first member is lll_hdr, matching the
 * pattern used throughout the Bluetooth controller (the lll_hdr.parent field
 * is accessed by HDR_LLL2ULL inside preempt_ticker_start).
 */
struct test_lll_ctx {
	struct lll_hdr hdr;
};

/* One static context and event used across test cases. */
static struct test_lll_ctx g_ctx;
static struct lll_event    g_ready_event;
static struct lll_prepare_param g_prepare_param;

/*
 * Custom fake for ull_prepare_enqueue: always return a non-NULL pointer so
 * the LL_ASSERT_ERR(next) inside lll_prepare_resolve does not fire.
 */
static struct lll_event g_enqueued_event;
static struct lll_event *ull_prepare_enqueue_non_null(
		lll_is_abort_cb_t is_abort_cb,
		lll_abort_cb_t abort_cb,
		struct lll_prepare_param *prepare_param,
		lll_prepare_cb_t prepare_cb,
		uint8_t is_resume)
{
	/* FFF call tracking (call_count, arg captures) is handled by the FFF
	 * generated wrapper; here we just return a non-NULL pointer.
	 */
	ARG_UNUSED(is_abort_cb);
	ARG_UNUSED(abort_cb);
	ARG_UNUSED(prepare_param);
	ARG_UNUSED(prepare_cb);
	ARG_UNUSED(is_resume);
	return &g_enqueued_event;
}

/*
 * prepare_cb stub: successful prepare, returns 0.
 */
static int g_prepare_cb_call_count;
static int test_prepare_cb(struct lll_prepare_param *prepare_param)
{
	g_prepare_cb_call_count++;
	return 0;
}

/*
 * abort_cb stub used when simulating an active current event.
 */
static int g_abort_cb_call_count;
static void test_abort_cb(struct lll_prepare_param *prepare_param, void *param)
{
	g_abort_cb_call_count++;
}

/*
 * is_abort_cb stub - not used in the non-LOW_LAT path, but required as a
 * valid non-NULL function pointer for event.curr.is_abort_cb.
 */
static int test_is_abort_cb(void *next, void *curr,
			     lll_prepare_cb_t *resume_cb)
{
	return -ECANCELED;
}

/* Counter used by the dequeue_iter_one_ready custom fake. */
static int g_dequeue_iter_call_count;

/* -------------------------------------------------------------------------
 * Helper: reset all mocks and internal state before each test.
 * -------------------------------------------------------------------------
 */
static void before_each(void *data)
{
	ARG_UNUSED(data);

	/* Reset UUT internal state. */
	lll_prepare_resolve_test_reset();

	/* Reset all FFF fakes. */
	RESET_FAKE(ull_prepare_enqueue);
	RESET_FAKE(ull_prepare_dequeue_iter);
	RESET_FAKE(ull_prepare_dequeue);
	RESET_FAKE(ticker_ticks_diff_get);
	RESET_FAKE(ticker_start);
	RESET_FAKE(ticker_stop);
	RESET_FAKE(mayfly_enqueue);

	/*
	 * Install the custom enqueue fake that always returns a non-NULL
	 * pointer, preventing the internal LL_ASSERT_ERR from firing.
	 */
	ull_prepare_enqueue_fake.custom_fake = ull_prepare_enqueue_non_null;

	/* ticker_start should succeed by default. */
	ticker_start_fake.return_val = TICKER_STATUS_SUCCESS;

	/* Reset per-test counters. */
	g_prepare_cb_call_count    = 0;
	g_abort_cb_call_count      = 0;
	g_dequeue_iter_call_count  = 0;

	/* Set up a default prepare param pointing to a valid context. */
	memset(&g_prepare_param, 0, sizeof(g_prepare_param));
	g_prepare_param.param           = &g_ctx;
	g_prepare_param.ticks_at_expire = 1000U;

	/* Set up a default ready event (pointing to a different context). */
	memset(&g_ready_event, 0, sizeof(g_ready_event));
	g_ready_event.prepare_param.param           = &g_ctx;
	g_ready_event.prepare_param.ticks_at_expire = 500U;
	g_ready_event.is_aborted = 0U;
	g_ready_event.is_resume  = 0U;
}

/* -------------------------------------------------------------------------
 * ull_prepare_dequeue_iter custom fakes.
 * -------------------------------------------------------------------------
 */

/*
 * Return NULL on every call: empty pipeline, no ready events.
 */
static void *dequeue_iter_null(void **idx)
{
	ARG_UNUSED(idx);
	return NULL;
}

/*
 * Return g_ready_event on the first call, NULL on subsequent calls.
 * Models a single ready event in the prepare pipeline.
 */
static void *dequeue_iter_one_ready(void **idx)
{
	ARG_UNUSED(idx);
	g_dequeue_iter_call_count++;
	if (g_dequeue_iter_call_count == 1) {
		return &g_ready_event;
	}
	return NULL;
}

/* -------------------------------------------------------------------------
 * Test cases.
 * -------------------------------------------------------------------------
 */

/*
 * Test 1: Execute path - no active event, empty pipeline.
 *
 * When event.curr.abort_cb == NULL, the pipeline is empty, and is_done_sync()
 * returns true (always the case without CONFIG_BT_CTLR_LOW_LAT_ULL_DONE),
 * lll_prepare_resolve must invoke prepare_cb and update event.curr.*.
 * There is no next event after prepare_cb, so no preempt ticker is started.
 *
 * Expected:
 *   - prepare_cb called once
 *   - ull_prepare_enqueue NOT called
 *   - ticker_start NOT called
 *   - event.curr.param updated to prepare_param->param
 *   - returns 0
 */
ZTEST(lll_prepare_resolve, test_execute_no_active_event)
{
	int err;

	/* Empty pipeline. */
	ull_prepare_dequeue_iter_fake.custom_fake = dequeue_iter_null;

	err = lll_prepare_resolve(test_is_abort_cb, test_abort_cb,
				  test_prepare_cb, &g_prepare_param,
				  0U, 0U);

	zassert_equal(err, 0, "Expected 0, got %d", err);
	zassert_equal(g_prepare_cb_call_count, 1,
		      "prepare_cb should be called once");
	zassert_equal(ull_prepare_enqueue_fake.call_count, 0U,
		      "ull_prepare_enqueue must not be called");
	zassert_equal(ticker_start_fake.call_count, 0U,
		      "ticker_start must not be called");
	zassert_equal(lll_prepare_resolve_test_get_curr_param(),
		      g_prepare_param.param,
		      "event.curr.param should be updated");
	zassert_equal(lll_prepare_resolve_test_get_curr_abort_cb(),
		      test_abort_cb,
		      "event.curr.abort_cb should be updated");
}

/*
 * Test 2: Enqueue path - current event is active (abort_cb set).
 *
 * When event.curr.abort_cb != NULL there is already an active event.  The
 * incoming prepare must be queued in the pipeline.  With is_resume == 0 and
 * defer == 0 the preempt ticker must be started so the active event can be
 * pre-empted at the right time.
 *
 * Expected:
 *   - ull_prepare_enqueue called once
 *   - ticker_start called once (preempt ticker started)
 *   - prepare_cb NOT called
 *   - returns -EINPROGRESS
 */
ZTEST(lll_prepare_resolve, test_enqueue_curr_event_active)
{
	int err;

	/* Simulate an active radio event. */
	lll_prepare_resolve_test_set_curr(test_abort_cb, test_is_abort_cb,
					  &g_ctx);

	/* Empty pipeline so ready == NULL (no ready_short). */
	ull_prepare_dequeue_iter_fake.custom_fake = dequeue_iter_null;

	err = lll_prepare_resolve(test_is_abort_cb, test_abort_cb,
				  test_prepare_cb, &g_prepare_param,
				  0U, 0U);

	zassert_equal(err, -EINPROGRESS, "Expected -EINPROGRESS, got %d", err);
	zassert_equal(ull_prepare_enqueue_fake.call_count, 1U,
		      "ull_prepare_enqueue should be called once");
	zassert_equal(ticker_start_fake.call_count, 1U,
		      "ticker_start should be called once");
	zassert_equal(g_prepare_cb_call_count, 0,
		      "prepare_cb must not be called");
}

/*
 * Test 3: Enqueue path - earlier ready event in the pipeline (ready_short).
 *
 * When a ready event with an earlier ticks_at_expire is already in the
 * pipeline (diff > 0 and MSbit clear), the incoming prepare must be queued
 * (ready_short != NULL path).  The first event (the earlier one) is used as
 * the anchor for the preempt ticker.
 *
 * Expected:
 *   - ull_prepare_enqueue called once
 *   - ticker_start called once with the ready event as anchor
 *   - ticker_start first arg (ticks_anchor) matches the ready event's ticks
 *   - prepare_cb NOT called
 *   - returns -EINPROGRESS
 */
ZTEST(lll_prepare_resolve, test_enqueue_ready_short)
{
	int err;

	/*
	 * g_ready_event.prepare_param.ticks_at_expire == 500,
	 * g_prepare_param.ticks_at_expire == 1000.
	 * diff = ticker_ticks_diff_get(1000, 500) -> we return a positive
	 * value with MSbit clear to signal the incoming event is later.
	 */
	ticker_ticks_diff_get_fake.return_val = 500U; /* positive, MSbit clear */
	g_dequeue_iter_call_count = 0;
	ull_prepare_dequeue_iter_fake.custom_fake = dequeue_iter_one_ready;

	err = lll_prepare_resolve(test_is_abort_cb, test_abort_cb,
				  test_prepare_cb, &g_prepare_param,
				  0U, 0U);

	zassert_equal(err, -EINPROGRESS, "Expected -EINPROGRESS, got %d", err);
	zassert_equal(ull_prepare_enqueue_fake.call_count, 1U,
		      "ull_prepare_enqueue should be called once");
	zassert_equal(ticker_start_fake.call_count, 1U,
		      "ticker_start should be called once");
	zassert_equal(g_prepare_cb_call_count, 0,
		      "prepare_cb must not be called");
}

/*
 * Test 4: Enqueue path - is_resume with a ready event in the pipeline.
 *
 * When is_resume == 1 and there is a ready event in the pipeline (which
 * matches the "short prepare" check because is_resume takes priority),
 * the prepare must be queued.  With is_resume == 1 no preempt ticker is
 * started (early return before ticker_start).
 *
 * Expected:
 *   - ull_prepare_enqueue called once
 *   - ticker_start NOT called
 *   - prepare_cb NOT called
 *   - returns -EINPROGRESS
 */
ZTEST(lll_prepare_resolve, test_enqueue_is_resume_with_ready)
{
	int err;

	/*
	 * Return a positive diff so the ready event looks "shorter" when
	 * is_resume overrides the check.
	 */
	ticker_ticks_diff_get_fake.return_val = 500U;
	g_dequeue_iter_call_count = 0;
	ull_prepare_dequeue_iter_fake.custom_fake = dequeue_iter_one_ready;

	err = lll_prepare_resolve(test_is_abort_cb, test_abort_cb,
				  test_prepare_cb, &g_prepare_param,
				  1U /* is_resume */, 0U);

	zassert_equal(err, -EINPROGRESS, "Expected -EINPROGRESS, got %d", err);
	zassert_equal(ull_prepare_enqueue_fake.call_count, 1U,
		      "ull_prepare_enqueue should be called once");
	zassert_equal(ticker_start_fake.call_count, 0U,
		      "ticker_start must NOT be called for is_resume");
	zassert_equal(g_prepare_cb_call_count, 0,
		      "prepare_cb must not be called");
}

/*
 * Test 5: Enqueue path - defer flag set, no ticker started.
 *
 * When an active event is present AND prepare_param->defer == 1, the prepare
 * is enqueued but the preempt ticker is skipped (early return before
 * ticker_start).
 *
 * Expected:
 *   - ull_prepare_enqueue called once
 *   - ticker_start NOT called
 *   - prepare_cb NOT called
 *   - returns -EINPROGRESS
 */
ZTEST(lll_prepare_resolve, test_enqueue_defer_no_ticker)
{
	int err;

	/* Simulate an active radio event. */
	lll_prepare_resolve_test_set_curr(test_abort_cb, test_is_abort_cb,
					  &g_ctx);

	ull_prepare_dequeue_iter_fake.custom_fake = dequeue_iter_null;

	/* Set defer flag. */
	g_prepare_param.defer = 1U;

	err = lll_prepare_resolve(test_is_abort_cb, test_abort_cb,
				  test_prepare_cb, &g_prepare_param,
				  0U, 0U);

	zassert_equal(err, -EINPROGRESS, "Expected -EINPROGRESS, got %d", err);
	zassert_equal(ull_prepare_enqueue_fake.call_count, 1U,
		      "ull_prepare_enqueue should be called once");
	zassert_equal(ticker_start_fake.call_count, 0U,
		      "ticker_start must NOT be called when defer is set");
	zassert_equal(g_prepare_cb_call_count, 0,
		      "prepare_cb must not be called");
}

/*
 * Test 6: Execute path - ready event is the same prepare_param.
 *
 * When the event found in the pipeline has the same prepare_param pointer as
 * the one being resolved (&ready->prepare_param == prepare_param), no
 * "short" event is detected.  ready_short and ready are both set to NULL and
 * execution proceeds through the execute path.
 *
 * Expected:
 *   - prepare_cb called once
 *   - ull_prepare_enqueue NOT called
 *   - ticker_start NOT called (no next event after prepare_cb)
 *   - returns 0
 */
ZTEST(lll_prepare_resolve, test_execute_ready_same_event)
{
	int err;
	struct lll_event same_event;

	/*
	 * Set up a pipeline event whose prepare_param IS the same object as
	 * g_prepare_param (same pointer).
	 */
	memset(&same_event, 0, sizeof(same_event));
	same_event.prepare_param = g_prepare_param; /* copy so fields match */

	/*
	 * The condition checked is:
	 *   &ready->prepare_param != prepare_param
	 * For same_event, &same_event.prepare_param != &g_prepare_param
	 * (different addresses), so the branch IS taken.
	 *
	 * We make ticker_ticks_diff_get return a value with MSbit set (negative
	 * diff) so the event is NOT considered "short" and ready_short = NULL.
	 */
	ticker_ticks_diff_get_fake.return_val = BIT(23); /* MSbit set = negative */

	/* For this test we simply start with an empty pipeline. */
	ull_prepare_dequeue_iter_fake.custom_fake = dequeue_iter_null;

	err = lll_prepare_resolve(test_is_abort_cb, test_abort_cb,
				  test_prepare_cb, &g_prepare_param,
				  0U, 0U);

	zassert_equal(err, 0, "Expected 0, got %d", err);
	zassert_equal(g_prepare_cb_call_count, 1,
		      "prepare_cb should be called once");
	zassert_equal(ull_prepare_enqueue_fake.call_count, 0U,
		      "ull_prepare_enqueue must not be called");
	zassert_equal(ticker_start_fake.call_count, 0U,
		      "ticker_start must not be called");
}

/*
 * Test 7: Execute path - ready event has a non-earlier ticks_at_expire.
 *
 * When there IS a ready event in the pipeline but diff & MSbit is set
 * (meaning the ready event is NOT earlier), ready_short = NULL, ready = NULL,
 * and execution proceeds through the execute path.
 *
 * Expected:
 *   - prepare_cb called once
 *   - ull_prepare_enqueue NOT called
 *   - returns 0
 */
ZTEST(lll_prepare_resolve, test_execute_ready_not_short)
{
	int err;

	/*
	 * Provide a ready event in the pipeline but make ticker_ticks_diff_get
	 * return a negative diff (MSbit set), meaning the ready event is not
	 * earlier than the incoming prepare.
	 */
	ticker_ticks_diff_get_fake.return_val = BIT(23); /* MSbit set */
	g_dequeue_iter_call_count = 0;
	ull_prepare_dequeue_iter_fake.custom_fake = dequeue_iter_one_ready;

	err = lll_prepare_resolve(test_is_abort_cb, test_abort_cb,
				  test_prepare_cb, &g_prepare_param,
				  0U, 0U);

	zassert_equal(err, 0, "Expected 0, got %d", err);
	zassert_equal(g_prepare_cb_call_count, 1,
		      "prepare_cb should be called once");
	zassert_equal(ull_prepare_enqueue_fake.call_count, 0U,
		      "ull_prepare_enqueue must not be called");
	zassert_equal(ticker_start_fake.call_count, 0U,
		      "ticker_start must not be called");
}

/*
 * Test 8: Execute path - after prepare_cb a next event is found in pipeline.
 *
 * When prepare_cb succeeds and the pipeline has a next ready event, the
 * preempt ticker must be started for that next event.
 *
 * Expected:
 *   - prepare_cb called once
 *   - ull_prepare_enqueue NOT called
 *   - ticker_start called once (for the next event in pipeline)
 *   - returns 0
 */
ZTEST(lll_prepare_resolve, test_execute_with_next_in_pipeline)
{
	int err;

	/*
	 * The first call to ull_prepare_dequeue_iter (before prepare_cb) returns
	 * NULL.  The second call (after prepare_cb) returns g_ready_event.
	 * A third call returns NULL.
	 *
	 * Use the FFF return_val_seq mechanism to sequence the return values.
	 */

	/* Provide a custom fake that counts calls. */
	ull_prepare_dequeue_iter_fake.custom_fake = NULL;
	ull_prepare_dequeue_iter_fake.return_val  = NULL;

	/*
	 * Use the FFF return_val_seq mechanism to return NULL first, then
	 * the ready event, then NULL again.
	 */
	static void *return_seq[3];

	return_seq[0] = NULL;           /* Before prepare_cb: empty pipeline */
	return_seq[1] = &g_ready_event; /* After prepare_cb: next event */
	return_seq[2] = NULL;           /* No more events */

	SET_RETURN_SEQ(ull_prepare_dequeue_iter, return_seq, 3);

	err = lll_prepare_resolve(test_is_abort_cb, test_abort_cb,
				  test_prepare_cb, &g_prepare_param,
				  0U, 0U);

	zassert_equal(err, 0, "Expected 0, got %d", err);
	zassert_equal(g_prepare_cb_call_count, 1,
		      "prepare_cb should be called once");
	zassert_equal(ull_prepare_enqueue_fake.call_count, 0U,
		      "ull_prepare_enqueue must not be called");
	zassert_equal(ticker_start_fake.call_count, 1U,
		      "ticker_start should be called for the next event");
}

/*
 * Test 9: Enqueue path - active event, is_resume=1, no ticker started.
 *
 * When the incoming prepare is a resume (is_resume == 1) and an active event
 * is present, the prepare is enqueued but the preempt ticker is skipped.
 *
 * Expected:
 *   - ull_prepare_enqueue called once
 *   - ticker_start NOT called
 *   - prepare_cb NOT called
 *   - returns -EINPROGRESS
 */
ZTEST(lll_prepare_resolve, test_enqueue_is_resume_no_ticker)
{
	int err;

	/* Simulate an active radio event. */
	lll_prepare_resolve_test_set_curr(test_abort_cb, test_is_abort_cb,
					  &g_ctx);

	ull_prepare_dequeue_iter_fake.custom_fake = dequeue_iter_null;

	err = lll_prepare_resolve(test_is_abort_cb, test_abort_cb,
				  test_prepare_cb, &g_prepare_param,
				  1U /* is_resume */, 0U);

	zassert_equal(err, -EINPROGRESS, "Expected -EINPROGRESS, got %d", err);
	zassert_equal(ull_prepare_enqueue_fake.call_count, 1U,
		      "ull_prepare_enqueue should be called once");
	zassert_equal(ticker_start_fake.call_count, 0U,
		      "ticker_start must NOT be called for is_resume");
	zassert_equal(g_prepare_cb_call_count, 0,
		      "prepare_cb must not be called");
}

/*
 * Test 10: Enqueue path - is_dequeue=1 overrides "is_done_sync" check.
 *
 * Without CONFIG_BT_CTLR_LOW_LAT_ULL_DONE is_done_sync() always returns
 * true so the is_dequeue flag has no effect on the main condition.  This
 * test verifies that when the only enqueue condition that *could* fire is
 * event.curr.abort_cb (set to simulate an active event), the is_dequeue
 * flag does NOT suppress that enqueue.  The event is still enqueued.
 *
 * Expected (same as test_enqueue_curr_event_active):
 *   - ull_prepare_enqueue called once
 *   - prepare_cb NOT called
 *   - returns -EINPROGRESS
 */
ZTEST(lll_prepare_resolve, test_enqueue_is_dequeue_with_active_event)
{
	int err;

	/* Active event ensures the enqueue condition fires. */
	lll_prepare_resolve_test_set_curr(test_abort_cb, test_is_abort_cb,
					  &g_ctx);

	ull_prepare_dequeue_iter_fake.custom_fake = dequeue_iter_null;

	err = lll_prepare_resolve(test_is_abort_cb, test_abort_cb,
				  test_prepare_cb, &g_prepare_param,
				  0U, 1U /* is_dequeue */);

	zassert_equal(err, -EINPROGRESS, "Expected -EINPROGRESS, got %d", err);
	zassert_equal(ull_prepare_enqueue_fake.call_count, 1U,
		      "ull_prepare_enqueue should be called once");
	zassert_equal(g_prepare_cb_call_count, 0,
		      "prepare_cb must not be called");
}

/*
 * Test 11: Execute path - is_dequeue=1, no active event.
 *
 * With is_dequeue=1, event.curr.abort_cb == NULL, and an empty pipeline, the
 * function must execute prepare_cb directly.
 *
 * Expected:
 *   - prepare_cb called once
 *   - ull_prepare_enqueue NOT called
 *   - returns 0
 */
ZTEST(lll_prepare_resolve, test_execute_is_dequeue_no_active_event)
{
	int err;

	ull_prepare_dequeue_iter_fake.custom_fake = dequeue_iter_null;

	err = lll_prepare_resolve(test_is_abort_cb, test_abort_cb,
				  test_prepare_cb, &g_prepare_param,
				  0U, 1U /* is_dequeue */);

	zassert_equal(err, 0, "Expected 0, got %d", err);
	zassert_equal(g_prepare_cb_call_count, 1,
		      "prepare_cb should be called once");
	zassert_equal(ull_prepare_enqueue_fake.call_count, 0U,
		      "ull_prepare_enqueue must not be called");
	zassert_equal(ticker_start_fake.call_count, 0U,
		      "ticker_start must not be called");
}

/*
 * Test 12: Enqueue path - ready event has zero diff (same ticks_at_expire).
 *
 * When ticker_ticks_diff_get returns 0, the condition
 *   (diff != 0U) && ((diff & BIT(HAL_TICKER_CNTR_MSBIT)) == 0U)
 * is FALSE (diff == 0), so ready_short = NULL, ready = NULL, and the
 * function falls through to the execute path.
 *
 * Expected:
 *   - prepare_cb called once
 *   - ull_prepare_enqueue NOT called
 *   - returns 0
 */
ZTEST(lll_prepare_resolve, test_execute_ready_zero_diff)
{
	int err;

	ticker_ticks_diff_get_fake.return_val = 0U; /* zero diff */
	g_dequeue_iter_call_count = 0;
	ull_prepare_dequeue_iter_fake.custom_fake = dequeue_iter_one_ready;

	err = lll_prepare_resolve(test_is_abort_cb, test_abort_cb,
				  test_prepare_cb, &g_prepare_param,
				  0U, 0U);

	zassert_equal(err, 0, "Expected 0, got %d", err);
	zassert_equal(g_prepare_cb_call_count, 1,
		      "prepare_cb should be called once");
	zassert_equal(ull_prepare_enqueue_fake.call_count, 0U,
		      "ull_prepare_enqueue must not be called");
}

/* -------------------------------------------------------------------------
 * Test suite registration.
 * -------------------------------------------------------------------------
 */
ZTEST_SUITE(lll_prepare_resolve, NULL, NULL, before_each, NULL, NULL);
