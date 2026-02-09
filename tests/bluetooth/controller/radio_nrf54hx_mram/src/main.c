/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>
#include <stdbool.h>

/**
 * @file
 * Unit test for MRAM no-latency state management in nRF54H20 radio.
 *
 * This test validates the reference-counting behavior of MRAM no-latency
 * requests and releases through radio_reset() and radio_stop() operations.
 */

/* State tracking */
static int pending_requests;
static bool mram_no_latency_state;

/**
 * Stub for mram_no_latency_request().
 * Increments the pending_requests counter.
 */
static void mram_no_latency_request(void)
{
	pending_requests++;
}

/**
 * Stub for mram_no_latency_cancel_or_release().
 * Decrements the pending_requests counter.
 */
static void mram_no_latency_cancel_or_release(void)
{
	if (pending_requests > 0) {
		pending_requests--;
		mram_no_latency_state = (pending_requests > 0);
	}
}

/**
 * Test callback function that resolves the async MRAM state.
 * If pending_requests > 0, MRAM no-latency is ON.
 * If pending_requests == 0, MRAM no-latency is OFF.
 */
static volatile uint8_t mram_no_latency_start_req;
static volatile uint8_t mram_no_latency_stop_req;
static uint8_t mram_no_latency_start_ack;
static uint8_t mram_no_latency_stop_ack;
static atomic_val_t mram_refcnt;

#define LL_ASSERT_ERR(x) \
		{ \
			while (!(x)) { \
			} \
		}

static void mram_no_latency_callback(void)
{
	/* There shall be an outstanding request to acknowledge */
	LL_ASSERT_ERR(mram_no_latency_start_ack != mram_no_latency_start_req);
	mram_no_latency_start_ack++;

#if 0
	/* There could be cancel or release */
	if (mram_no_latency_stop_req != mram_no_latency_stop_ack) {
		mram_no_latency_stop_ack++;

		/* There shall be no more than one cancel or release */
		LL_ASSERT_ERR(mram_no_latency_stop_ack == mram_no_latency_stop_req);

		/* Handle cancel or release if was no request placed again */
		if (mram_no_latency_start_ack == mram_no_latency_start_req) {
			mram_no_latency_cancel_or_release();

			mram_no_latency_state = (pending_requests > 0);
		} else {
			/* Request placed after cancel or release */
			mram_no_latency_start_ack++;
			LL_ASSERT_ERR(mram_no_latency_start_ack == mram_no_latency_start_req);

			mram_no_latency_state = (pending_requests > 0);
		}
	} else {
		mram_no_latency_state = (pending_requests > 0);
	}
#else
	/* Count the requests and releases */
	uint8_t req = mram_no_latency_start_req - mram_no_latency_start_ack;
	uint8_t rel = mram_no_latency_stop_req - mram_no_latency_stop_ack;

	/* Reset requests and releases */
	mram_no_latency_start_ack = mram_no_latency_start_req;
	mram_no_latency_stop_ack = mram_no_latency_stop_req;

	/* Handle cancel or release */
	if (rel > req) {
		mram_no_latency_cancel_or_release();
	} else {
		/* No cancel or release before this callback */
	}

	mram_no_latency_state = (pending_requests > 0);
#endif
}

/**
 * Simplified radio_reset() that mirrors hal_radio_reset() behavior.
 * Calls mram_no_latency_request().
 */
static void radio_reset(void)
{
	atomic_val_t refcnt;

	/* Check and request mram no latency if we are the first instance */
	refcnt = atomic_inc(&mram_refcnt);
	if (refcnt == 0) {
		uint8_t old = mram_no_latency_start_req;
		uint8_t ack = mram_no_latency_start_ack;
		uint8_t req = old + 1U;

		/* Check rollover condition, which shall not happen by design */
		LL_ASSERT_ERR(req != ack);

		/* Mark for mram no latency requested */
		mram_no_latency_start_req = req;

		if (mram_no_latency_stop_req == mram_no_latency_stop_ack) {
			mram_no_latency_request();
		} else {
			/* We leave the request marked so that the callback will
			 * retain the mram_no_latency.
			 */
		}
	} else {
		/* Nothing to do, reference count increased. */
	}
}

/**
 * Simplified radio_stop() that mirrors hal_radio_stop() behavior.
 * Calls mram_no_latency_cancel_or_release().
 */
static void radio_stop(void)
{
	atomic_val_t refcnt;

	/* Check and request a cancel or release if we are the last instance */
	refcnt = atomic_get(&mram_refcnt);
	if (refcnt > 0) {
		refcnt = atomic_dec(&mram_refcnt);
		if (refcnt == 1) {
			uint8_t old = mram_no_latency_stop_req;
			uint8_t ack = mram_no_latency_stop_ack;
			uint8_t req = old + 1U;

			/* Mark for cancel or release being requested */
			LL_ASSERT_ERR(req != ack);
			mram_no_latency_stop_req = req;

			if (mram_no_latency_start_req == mram_no_latency_start_ack) {
				mram_no_latency_cancel_or_release();

				/* Unmark cancel or release, as its handled here
				 * with successful value being returned.
				 */
				mram_no_latency_stop_req = old;
			} else {
				/* Nothing to do, mram_no_latency not started
				 * yet, cancel or release will be performed in
				 * the callback when mram_no_latency is started.
				 */
			}
		} else {
			/* Nothing to do, reference count decremented. */
		}
	} else {
		/* NOTE: radio_stop() will be called more times than radio_reset
		 *       hence it is ok for the refcnt being zero.
		 */
	}
}

/* Test setup and teardown */
static void *test_setup(void)
{
	return NULL;
}

static void test_before(void *fixture)
{
	ARG_UNUSED(fixture);
	
	/* Reset state before each test */
	pending_requests = 0;
	mram_no_latency_state = false;

	mram_no_latency_start_req = 0U;
	mram_no_latency_stop_req = 0U;
	mram_no_latency_start_ack = 0U;
	mram_no_latency_stop_ack = 0U;
	mram_refcnt = 0U;
}

/**
 * Test: Single reset without stop (reset -> callback -> ON)
 * Verifies state remains ON when no stop is called.
 */
ZTEST(radio_nrf54hx_mram, test_single_reset_no_stop)
{
	radio_reset();
	zassert_equal(pending_requests, 1, "Expected 1 pending request after reset");
	
	mram_no_latency_callback();
	zassert_true(mram_no_latency_state,
		     "MRAM no-latency should be ON after reset without stop");
}

/**
 * Test: Single radio event cycle (reset -> stop -> callback -> OFF)
 * Verifies basic operation cycle.
 */
ZTEST(radio_nrf54hx_mram, test_single_radio_event_cycle)
{
	radio_reset();
	zassert_equal(pending_requests, 1, "Expected 1 pending request after reset");
	
	radio_stop();
	zassert_equal(pending_requests, 1, "Expected 1 pending requests after stop");
	
	mram_no_latency_callback();
	zassert_equal(pending_requests, 0, "Expected 0 pending requests after callback");
	zassert_false(mram_no_latency_state,
		      "MRAM no-latency should be OFF after complete cycle");
}

/**
 * Test: Multiple radio event cycle (reset -> stop -> reset -> callback -> ON -> ...)
 * Verifies basic operation cycle.
 */
ZTEST(radio_nrf54hx_mram, test_multiple_radio_event_cycle)
{
	radio_reset();
	zassert_equal(pending_requests, 1, "Expected 1 pending request after reset");
	
	radio_stop();
	zassert_equal(pending_requests, 1, "Expected 1 pending requests after stop");
	
	radio_reset();
	zassert_equal(pending_requests, 1, "Expected 1 pending request after reset");
	
	mram_no_latency_callback();
	zassert_equal(pending_requests, 1, "Expected 1 pending requests after callback");
	zassert_true(mram_no_latency_state,
		      "MRAM no-latency should be ON after complete cycle and a reset call");

	radio_reset();
	zassert_equal(pending_requests, 1, "Expected 1 pending request after reset");

	radio_reset();
	zassert_equal(pending_requests, 1, "Expected 1 pending request after reset");
	
	radio_stop();
	zassert_equal(pending_requests, 1, "Expected 1 pending requests after stop");
	zassert_true(mram_no_latency_state,
		     "MRAM no-latency should be ON with 1 pending request");
	
	radio_stop();
	zassert_equal(pending_requests, 1, "Expected 1 pending requests after stop");
	zassert_true(mram_no_latency_state,
		     "MRAM no-latency should be ON with 1 pending request");
	
	radio_stop();
	zassert_equal(pending_requests, 0, "Expected 0 pending requests after stop");
	zassert_false(mram_no_latency_state,
		     "MRAM no-latency should be OFF with 0 pending request");

	radio_reset();
	zassert_equal(pending_requests, 1, "Expected 1 pending request after reset");
	zassert_false(mram_no_latency_state,
		     "MRAM no-latency should be OFF with 0 pending request");
	
	mram_no_latency_callback();
	zassert_equal(pending_requests, 1, "Expected 1 pending requests after callback");
	zassert_true(mram_no_latency_state,
		      "MRAM no-latency should be ON after complete cycle and a reset call");
}

/**
 * Test: Interleaved radio_reset/radio_stop calls
 * Verifies state management with interleaved operations.
 */
ZTEST(radio_nrf54hx_mram, test_interleaved_reset_stop)
{
	/* Pattern: reset, reset, stop, reset, stop, stop */
	radio_reset();
	zassert_equal(pending_requests, 1, "Expected 1 pending request");
	
	radio_reset();
	zassert_equal(pending_requests, 1, "Expected 2 pending requests");
	
	radio_stop();
	zassert_equal(pending_requests, 1, "Expected 1 pending request after first stop");
	
	radio_reset();
	zassert_equal(pending_requests, 1, "Expected 2 pending requests");
	
	radio_stop();
	zassert_equal(pending_requests, 1, "Expected 1 pending request");
	
	mram_no_latency_callback();
	zassert_true(mram_no_latency_state,
		     "MRAM no-latency should be ON with 1 pending request");
	
	radio_stop();
	zassert_equal(pending_requests, 0, "Expected 0 pending requests");
	zassert_false(mram_no_latency_state,
		      "MRAM no-latency should be OFF with 0 pending requests");
}

/**
 * Test: n calls to request and n calls to release, then callback -> OFF
 * Verifies that equal requests and releases result in OFF state.
 */
ZTEST(radio_nrf54hx_mram, test_n_requests_n_releases_then_callback_off)
{
	int test_values[] = {1, 2, 3, 5, 10};
	
	for (int i = 0; i < ARRAY_SIZE(test_values); i++) {
		int n = test_values[i];
		
		/* Reset state */
		test_before(NULL);
		
		/* n requests */
		for (int j = 0; j < n; j++) {
			radio_reset();
		}
		
		zassert_equal(pending_requests, 1,
			      "Expected 1pending requests, got %d", pending_requests);
		
		/* n releases */
		for (int j = 0; j < n; j++) {
			radio_stop();
		}
		
		zassert_equal(pending_requests, 1,
			      "Expected 1 pending requests after releases, got %d",
			      pending_requests);
		
		/* Callback resolves state */
		mram_no_latency_callback();
		
		zassert_equal(pending_requests, 0,
			      "Expected 0 pending requests after releases, got %d",
			      pending_requests);
		zassert_false(mram_no_latency_state,
			      "MRAM no-latency should be OFF after %d requests and %d releases",
			      n, n);
	}
}

/**
 * Test: (n+1) calls to request and n calls to release, then callback -> ON
 * Verifies that one extra request results in ON state.
 */
ZTEST(radio_nrf54hx_mram, test_n_plus_1_requests_n_releases_then_callback_on)
{
	int test_values[] = {0, 1, 2, 3, 5, 10};
	
	for (int i = 0; i < ARRAY_SIZE(test_values); i++) {
		int n = test_values[i];
		
		/* Reset state */
		test_before(NULL);
		
		/* (n+1) requests */
		for (int j = 0; j < n + 1; j++) {
			radio_reset();
		}
		
		zassert_equal(pending_requests, 1, "Expected 1 pending requests, got %d",
			      pending_requests);
		
		/* n releases */
		for (int j = 0; j < n; j++) {
			radio_stop();
		}
		
		zassert_equal(pending_requests, 1,
			      "Expected 1 pending request after %d releases, got %d",
			      n, pending_requests);
		
		/* Callback resolves state */
		mram_no_latency_callback();
		
		zassert_true(mram_no_latency_state,
			     "MRAM no-latency should be ON after %d requests and %d releases",
			     n + 1, n);
	}
}

ZTEST_SUITE(radio_nrf54hx_mram, NULL, test_setup, test_before, NULL, NULL);
