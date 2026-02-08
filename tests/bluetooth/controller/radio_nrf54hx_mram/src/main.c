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
	}
}

/**
 * Test callback function that resolves the async MRAM state.
 * If pending_requests > 0, MRAM no-latency is ON.
 * If pending_requests == 0, MRAM no-latency is OFF.
 */
static void mram_no_latency_callback(void)
{
	mram_no_latency_state = (pending_requests > 0);
}

/**
 * Simplified radio_reset() that mirrors hal_radio_reset() behavior.
 * Calls mram_no_latency_request().
 */
static void radio_reset(void)
{
	mram_no_latency_request();
}

/**
 * Simplified radio_stop() that mirrors hal_radio_stop() behavior.
 * Calls mram_no_latency_cancel_or_release().
 */
static void radio_stop(void)
{
	mram_no_latency_cancel_or_release();
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
		pending_requests = 0;
		mram_no_latency_state = false;
		
		/* n requests */
		for (int j = 0; j < n; j++) {
			radio_reset();
		}
		
		zassert_equal(pending_requests, n,
			      "Expected %d pending requests, got %d", n, pending_requests);
		
		/* n releases */
		for (int j = 0; j < n; j++) {
			radio_stop();
		}
		
		zassert_equal(pending_requests, 0,
			      "Expected 0 pending requests after releases, got %d",
			      pending_requests);
		
		/* Callback resolves state */
		mram_no_latency_callback();
		
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
		pending_requests = 0;
		mram_no_latency_state = false;
		
		/* (n+1) requests */
		for (int j = 0; j < n + 1; j++) {
			radio_reset();
		}
		
		zassert_equal(pending_requests, n + 1,
			      "Expected %d pending requests, got %d",
			      n + 1, pending_requests);
		
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
	zassert_equal(pending_requests, 2, "Expected 2 pending requests");
	
	radio_stop();
	zassert_equal(pending_requests, 1, "Expected 1 pending request after first stop");
	
	radio_reset();
	zassert_equal(pending_requests, 2, "Expected 2 pending requests");
	
	radio_stop();
	zassert_equal(pending_requests, 1, "Expected 1 pending request");
	
	mram_no_latency_callback();
	zassert_true(mram_no_latency_state,
		     "MRAM no-latency should be ON with 1 pending request");
	
	radio_stop();
	zassert_equal(pending_requests, 0, "Expected 0 pending requests");
	
	mram_no_latency_callback();
	zassert_false(mram_no_latency_state,
		      "MRAM no-latency should be OFF with 0 pending requests");
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
	zassert_equal(pending_requests, 0, "Expected 0 pending requests after stop");
	
	mram_no_latency_callback();
	zassert_false(mram_no_latency_state,
		      "MRAM no-latency should be OFF after complete cycle");
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
 * Test: No operations, callback -> OFF
 * Verifies initial state is OFF.
 */
ZTEST(radio_nrf54hx_mram, test_zero_requests_callback_off)
{
	mram_no_latency_callback();
	zassert_false(mram_no_latency_state,
		      "MRAM no-latency should be OFF with no operations");
	zassert_equal(pending_requests, 0, "Expected 0 pending requests");
}

/**
 * Test: Multiple callbacks maintain correct state
 * Verifies that multiple callbacks between operations maintain state.
 */
ZTEST(radio_nrf54hx_mram, test_multiple_callbacks)
{
	/* First cycle */
	radio_reset();
	mram_no_latency_callback();
	zassert_true(mram_no_latency_state, "Should be ON after first reset");
	
	/* Second callback without state change */
	mram_no_latency_callback();
	zassert_true(mram_no_latency_state, "Should remain ON");
	
	/* Stop and verify */
	radio_stop();
	mram_no_latency_callback();
	zassert_false(mram_no_latency_state, "Should be OFF after stop");
	
	/* Multiple callbacks while OFF */
	mram_no_latency_callback();
	zassert_false(mram_no_latency_state, "Should remain OFF");
	
	mram_no_latency_callback();
	zassert_false(mram_no_latency_state, "Should remain OFF");
	
	/* New cycle */
	radio_reset();
	radio_reset();
	mram_no_latency_callback();
	zassert_true(mram_no_latency_state, "Should be ON with 2 requests");
	zassert_equal(pending_requests, 2, "Expected 2 pending requests");
	
	radio_stop();
	mram_no_latency_callback();
	zassert_true(mram_no_latency_state, "Should be ON with 1 request remaining");
	zassert_equal(pending_requests, 1, "Expected 1 pending request");
	
	radio_stop();
	mram_no_latency_callback();
	zassert_false(mram_no_latency_state, "Should be OFF after all releases");
	zassert_equal(pending_requests, 0, "Expected 0 pending requests");
}

ZTEST_SUITE(radio_nrf54hx_mram, NULL, test_setup, test_before, NULL, NULL);
