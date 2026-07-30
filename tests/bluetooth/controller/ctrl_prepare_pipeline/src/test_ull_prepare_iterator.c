/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Prepare pipeline iterator tests
 *
 * Tests the iterator functionality with the new void** interface
 * introduced in PR #79444 (changed from uint8_t*).
 */

#include <string.h>
#include <zephyr/types.h>
#include <zephyr/ztest.h>

#include "util/util.h"
#include "util/mem.h"
#include "util/memq.h"

#include "lll.h"
#include "ull_internal.h"

#include "test_common.h"

/* Mock callbacks */
static int test_prepare_cb(struct lll_prepare_param *prepare_param)
{
	return 0;
}

static int test_is_abort_cb(void *next, void *curr, lll_prepare_cb_t *resume_cb)
{
	return 0;
}

static void test_abort_cb(struct lll_prepare_param *prepare_param, void *param)
{
}

/*
 * Test: Iterator initialization with NULL
 * PR #79444: idx = NULL should initialize iterator to head of list.
 */
ZTEST(test_ull_prepare_iterator, test_iterator_init_null)
{
	struct lll_prepare_param prepare_param;
	struct lll_event *event, *first;
	void *idx = NULL;

	/* Enqueue one event */
	memset(&prepare_param, 0, sizeof(prepare_param));
	prepare_param.ticks_at_expire = 1000;
	prepare_param.param = (void *)0x1111;

	event = ull_prepare_enqueue(test_is_abort_cb, test_abort_cb,
				    &prepare_param, test_prepare_cb, 0);
	zassert_not_null(event, "Failed to enqueue event");

	/* Initialize iterator with NULL */
	first = ull_prepare_dequeue_iter(&idx);
	zassert_not_null(first, "Iterator should return first event");
	zassert_equal(first, event, "First event should match enqueued event");
}

/*
 * Test: Complete iteration through list
 * Validates that iterator traverses all events in the list.
 */
ZTEST(test_ull_prepare_iterator, test_complete_iteration)
{
	struct lll_prepare_param prepare_params[5];
	struct lll_event *events[5];
	void *idx = NULL;
	struct lll_event *current;
	int count = 0;
	int i;

	/* Enqueue 5 events */
	for (i = 0; i < 5; i++) {
		memset(&prepare_params[i], 0, sizeof(prepare_params[i]));
		prepare_params[i].ticks_at_expire = 1000 + (i * 100);
		prepare_params[i].param = (void *)(uintptr_t)i;

		events[i] = ull_prepare_enqueue(test_is_abort_cb, test_abort_cb,
						&prepare_params[i], test_prepare_cb, 0);
		zassert_not_null(events[i], "Failed to enqueue event %d", i);
	}

	/* Iterate through all events */
	while ((current = ull_prepare_dequeue_iter(&idx)) != NULL) {
		zassert_not_null(current, "Current event should not be NULL");
		count++;
	}

	zassert_equal(count, 5, "Should iterate through all 5 events");
}

/*
 * Test: Iterator termination condition
 * PR #79444: Iterator returns NULL when idx becomes NULL (end of list).
 */
ZTEST(test_ull_prepare_iterator, test_iterator_termination)
{
	struct lll_prepare_param prepare_params[3];
	struct lll_event *events[3];
	void *idx = NULL;
	struct lll_event *current;
	int i;

	/* Enqueue 3 events */
	for (i = 0; i < 3; i++) {
		memset(&prepare_params[i], 0, sizeof(prepare_params[i]));
		prepare_params[i].ticks_at_expire = 1000 + (i * 200);

		events[i] = ull_prepare_enqueue(test_is_abort_cb, test_abort_cb,
						&prepare_params[i], test_prepare_cb, 0);
	}

	/* Iterate through all events */
	for (i = 0; i < 3; i++) {
		current = ull_prepare_dequeue_iter(&idx);
		zassert_not_null(current, "Event %d should not be NULL", i);
	}

	/* Next call should return NULL (end of list) */
	current = ull_prepare_dequeue_iter(&idx);
	zassert_is_null(current, "Iterator should return NULL at end of list");

	/* Subsequent calls should also return NULL */
	current = ull_prepare_dequeue_iter(&idx);
	zassert_is_null(current, "Iterator should remain NULL");
}

/*
 * Test: Iterator on empty pipeline
 * Validates iterator behavior on empty pipeline.
 */
ZTEST(test_ull_prepare_iterator, test_iterator_empty_pipeline)
{
	void *idx = NULL;
	struct lll_event *current;

	/* Try to iterate on empty pipeline */
	current = ull_prepare_dequeue_iter(&idx);
	zassert_is_null(current, "Iterator on empty pipeline should return NULL");
}

/*
 * Test: Iterator with single event
 * Edge case: pipeline with only one event.
 */
ZTEST(test_ull_prepare_iterator, test_iterator_single_event)
{
	struct lll_prepare_param prepare_param;
	struct lll_event *event, *current;
	void *idx = NULL;

	/* Enqueue single event */
	memset(&prepare_param, 0, sizeof(prepare_param));
	prepare_param.ticks_at_expire = 1000;

	event = ull_prepare_enqueue(test_is_abort_cb, test_abort_cb,
				    &prepare_param, test_prepare_cb, 0);
	zassert_not_null(event, "Failed to enqueue event");

	/* First iteration should return the event */
	current = ull_prepare_dequeue_iter(&idx);
	zassert_equal(current, event, "Should return the single event");

	/* Second iteration should return NULL */
	current = ull_prepare_dequeue_iter(&idx);
	zassert_is_null(current, "Should return NULL after single event");
}

/*
 * Test: Iterator with mixed event types
 * Tests iteration through normal, resume, and aborted events.
 */
ZTEST(test_ull_prepare_iterator, test_iterator_mixed_event_types)
{
	struct lll_prepare_param prepare_params[6];
	struct lll_event *events[6];
	void *idx = NULL;
	struct lll_event *current;
	int count = 0;
	int normal_count = 0;
	int resume_count = 0;
	int aborted_count = 0;
	int i;

	/* Enqueue mix of event types */
	for (i = 0; i < 6; i++) {
		memset(&prepare_params[i], 0, sizeof(prepare_params[i]));
		prepare_params[i].ticks_at_expire = 1000 + (i * 100);

		/* Alternating: normal, resume, normal, resume, normal, resume */
		uint8_t is_resume = (i % 2);
		events[i] = ull_prepare_enqueue(test_is_abort_cb, test_abort_cb,
						&prepare_params[i], test_prepare_cb,
						is_resume);
	}

	/* Mark some as aborted */
	events[1]->is_aborted = 1;
	events[4]->is_aborted = 1;

	/* Iterate and count event types */
	while ((current = ull_prepare_dequeue_iter(&idx)) != NULL) {
		count++;
		if (current->is_aborted) {
			aborted_count++;
		} else if (current->is_resume) {
			resume_count++;
		} else {
			normal_count++;
		}
	}

	zassert_equal(count, 6, "Should iterate through all 6 events");
	zassert_equal(aborted_count, 2, "Should find 2 aborted events");
	/* Note: resume_count includes aborted resume events */
}

/*
 * Test: Iterator break condition safety
 * PR #79444: Breaking when !idx should be safe.
 */
ZTEST(test_ull_prepare_iterator, test_iterator_break_condition)
{
	struct lll_prepare_param prepare_params[5];
	struct lll_event *events[5];
	void *idx = NULL;
	struct lll_event *current;
	int count = 0;
	int i;

	/* Enqueue 5 events */
	for (i = 0; i < 5; i++) {
		memset(&prepare_params[i], 0, sizeof(prepare_params[i]));
		prepare_params[i].ticks_at_expire = 1000 + (i * 100);

		events[i] = ull_prepare_enqueue(test_is_abort_cb, test_abort_cb,
						&prepare_params[i], test_prepare_cb, 0);
	}

	/* Iterate with early break on condition */
	while ((current = ull_prepare_dequeue_iter(&idx)) != NULL) {
		count++;
		/* Break after 3 events */
		if (count >= 3) {
			break;
		}
	}

	zassert_equal(count, 3, "Should have broken after 3 events");

	/* Verify we can restart iteration */
	idx = NULL;
	count = 0;
	while ((current = ull_prepare_dequeue_iter(&idx)) != NULL) {
		count++;
	}

	zassert_equal(count, 5, "Restarted iteration should see all events");
}

/*
 * Test: Iterator parameter passing
 * Validates that idx is properly updated during iteration.
 */
ZTEST(test_ull_prepare_iterator, test_iterator_parameter_update)
{
	struct lll_prepare_param prepare_params[3];
	struct lll_event *events[3];
	void *idx = NULL;
	void *prev_idx;
	struct lll_event *current;
	int i;

	/* Enqueue 3 events */
	for (i = 0; i < 3; i++) {
		memset(&prepare_params[i], 0, sizeof(prepare_params[i]));
		prepare_params[i].ticks_at_expire = 1000 + (i * 100);

		events[i] = ull_prepare_enqueue(test_is_abort_cb, test_abort_cb,
						&prepare_params[i], test_prepare_cb, 0);
	}

	/* Verify idx is updated during iteration */
	prev_idx = idx;
	current = ull_prepare_dequeue_iter(&idx);
	zassert_not_null(current, "First event should not be NULL");
	zassert_not_equal(idx, prev_idx, "idx should be updated after first call");

	prev_idx = idx;
	current = ull_prepare_dequeue_iter(&idx);
	zassert_not_null(current, "Second event should not be NULL");
	/* idx may or may not change depending on implementation */

	current = ull_prepare_dequeue_iter(&idx);
	zassert_not_null(current, "Third event should not be NULL");

	/* End of list */
	current = ull_prepare_dequeue_iter(&idx);
	zassert_is_null(current, "Should reach end of list");
}

/* Test suite setup/teardown */
static void *test_ull_prepare_iterator_setup(void)
{
	return NULL;
}

static void test_ull_prepare_iterator_before(void *f)
{
	ull_prepare_pipeline_init();
}

static void test_ull_prepare_iterator_after(void *f)
{
	ull_prepare_pipeline_cleanup();
}

static void test_ull_prepare_iterator_teardown(void *f)
{
}

ZTEST_SUITE(test_ull_prepare_iterator, NULL,
	   test_ull_prepare_iterator_setup,
	   test_ull_prepare_iterator_before,
	   test_ull_prepare_iterator_after,
	   test_ull_prepare_iterator_teardown);
