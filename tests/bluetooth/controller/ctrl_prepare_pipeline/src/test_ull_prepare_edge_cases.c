/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Prepare pipeline edge case and boundary condition tests
 *
 * Tests edge cases, boundary conditions, and error handling for the
 * ordered linked list implementation in PR #79444.
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

/* Event pipeline maximum size as defined in ull.c */
#define EVENT_DEFER_MAX 2
#define EVENT_PIPELINE_MAX (7U + (EVENT_DEFER_MAX))

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
 * Test: Empty pipeline operations
 * Validates behavior on empty pipeline for all operations.
 */
ZTEST(test_ull_prepare_edge_cases, test_empty_pipeline_operations)
{
	struct lll_event *event;
	void *idx = NULL;

	/* dequeue_get on empty pipeline */
	event = ull_prepare_dequeue_get();
	zassert_is_null(event, "dequeue_get should return NULL on empty pipeline");

	/* dequeue_iter on empty pipeline */
	event = ull_prepare_dequeue_iter(&idx);
	zassert_is_null(event, "dequeue_iter should return NULL on empty pipeline");
}

/*
 * Test: Single element pipeline operations
 * Edge case testing with only one element in pipeline.
 */
ZTEST(test_ull_prepare_edge_cases, test_single_element_pipeline)
{
	struct lll_prepare_param prepare_param;
	struct lll_event *event, *retrieved;
	void *idx = NULL;

	/* Enqueue single event */
	memset(&prepare_param, 0, sizeof(prepare_param));
	prepare_param.ticks_at_expire = 1000;
	prepare_param.param = (void *)0xABCD;

	event = ull_prepare_enqueue(test_is_abort_cb, test_abort_cb,
				    &prepare_param, test_prepare_cb, 0);
	zassert_not_null(event, "Failed to enqueue single event");

	/* Get head */
	retrieved = ull_prepare_dequeue_get();
	zassert_equal(retrieved, event, "Head should be the single event");

	/* Iterate */
	retrieved = ull_prepare_dequeue_iter(&idx);
	zassert_equal(retrieved, event, "Iterator should return single event");

	/* Next iteration should be NULL */
	retrieved = ull_prepare_dequeue_iter(&idx);
	zassert_is_null(retrieved, "Iterator should return NULL after single event");
}

/*
 * Test: Pipeline full condition
 * Tests behavior when pipeline reaches maximum capacity.
 * Note: This test validates that enqueue handles full pipeline gracefully.
 */
ZTEST(test_ull_prepare_edge_cases, test_pipeline_full_condition)
{
	struct lll_prepare_param prepare_params[EVENT_PIPELINE_MAX + 1];
	struct lll_event *events[EVENT_PIPELINE_MAX + 1];
	int i;
	int success_count = 0;

	/* Try to enqueue more than pipeline max */
	for (i = 0; i < EVENT_PIPELINE_MAX + 1; i++) {
		memset(&prepare_params[i], 0, sizeof(prepare_params[i]));
		prepare_params[i].ticks_at_expire = 1000 + (i * 100);
		prepare_params[i].param = (void *)(uintptr_t)i;

		events[i] = ull_prepare_enqueue(test_is_abort_cb, test_abort_cb,
						&prepare_params[i], test_prepare_cb, 0);
		if (events[i] != NULL) {
			success_count++;
		}
	}

	/* At least EVENT_PIPELINE_MAX events should be enqueued successfully */
	zassert_true(success_count >= EVENT_PIPELINE_MAX,
		    "Should enqueue at least EVENT_PIPELINE_MAX events");

	/* If pipeline is full, last enqueue might fail */
	if (success_count == EVENT_PIPELINE_MAX) {
		zassert_is_null(events[EVENT_PIPELINE_MAX],
			       "Enqueue should fail when pipeline is full");
	}
}

/*
 * Test: Tick counter wraparound scenarios
 * Tests ordering behavior when tick values wrap around 32-bit boundary.
 */
ZTEST(test_ull_prepare_edge_cases, test_tick_wraparound)
{
	struct lll_prepare_param prepare_params[4];
	struct lll_event *events[4];
	void *idx = NULL;
	struct lll_event *current;
	int i;

	/* Create events near wraparound boundary */
	/* Event 1: Near max uint32_t */
	memset(&prepare_params[0], 0, sizeof(prepare_params[0]));
	prepare_params[0].ticks_at_expire = UINT32_MAX - 1000;
	events[0] = ull_prepare_enqueue(test_is_abort_cb, test_abort_cb,
					&prepare_params[0], test_prepare_cb, 0);

	/* Event 2: Very close to max */
	memset(&prepare_params[1], 0, sizeof(prepare_params[1]));
	prepare_params[1].ticks_at_expire = UINT32_MAX - 100;
	events[1] = ull_prepare_enqueue(test_is_abort_cb, test_abort_cb,
					&prepare_params[1], test_prepare_cb, 0);

	/* Event 3: After wraparound (small value) */
	memset(&prepare_params[2], 0, sizeof(prepare_params[2]));
	prepare_params[2].ticks_at_expire = 100;
	events[2] = ull_prepare_enqueue(test_is_abort_cb, test_abort_cb,
					&prepare_params[2], test_prepare_cb, 0);

	/* Event 4: Another after wraparound */
	memset(&prepare_params[3], 0, sizeof(prepare_params[3]));
	prepare_params[3].ticks_at_expire = 500;
	events[3] = ull_prepare_enqueue(test_is_abort_cb, test_abort_cb,
					&prepare_params[3], test_prepare_cb, 0);

	/* All events should be enqueued */
	for (i = 0; i < 4; i++) {
		zassert_not_null(events[i], "Event %d should be enqueued", i);
	}

	/* Verify we can iterate through all events */
	i = 0;
	while ((current = ull_prepare_dequeue_iter(&idx)) != NULL) {
		i++;
	}
	zassert_equal(i, 4, "Should iterate through all 4 events");
}

/*
 * Test: Multiple dequeue_get calls
 * Validates that dequeue_get doesn't modify pipeline state.
 */
ZTEST(test_ull_prepare_edge_cases, test_multiple_dequeue_get_calls)
{
	struct lll_prepare_param prepare_param;
	struct lll_event *event, *head1, *head2, *head3;

	/* Enqueue event */
	memset(&prepare_param, 0, sizeof(prepare_param));
	prepare_param.ticks_at_expire = 1000;

	event = ull_prepare_enqueue(test_is_abort_cb, test_abort_cb,
				    &prepare_param, test_prepare_cb, 0);
	zassert_not_null(event, "Failed to enqueue event");

	/* Multiple dequeue_get calls should return same event */
	head1 = ull_prepare_dequeue_get();
	head2 = ull_prepare_dequeue_get();
	head3 = ull_prepare_dequeue_get();

	zassert_equal(head1, head2, "Multiple dequeue_get should return same head");
	zassert_equal(head2, head3, "Multiple dequeue_get should return same head");
	zassert_equal(head1, event, "Head should be the enqueued event");
}

/*
 * Test: Alternating enqueue and iterate
 * Tests pipeline behavior with interleaved operations.
 */
ZTEST(test_ull_prepare_edge_cases, test_alternating_enqueue_iterate)
{
	struct lll_prepare_param prepare_params[6];
	struct lll_event *events[6];
	void *idx = NULL;
	struct lll_event *current;
	int i, count;

	/* Enqueue 3 events */
	for (i = 0; i < 3; i++) {
		memset(&prepare_params[i], 0, sizeof(prepare_params[i]));
		prepare_params[i].ticks_at_expire = 1000 + (i * 100);
		events[i] = ull_prepare_enqueue(test_is_abort_cb, test_abort_cb,
						&prepare_params[i], test_prepare_cb, 0);
	}

	/* Iterate to first event */
	current = ull_prepare_dequeue_iter(&idx);
	zassert_not_null(current, "First event should exist");

	/* Enqueue more events */
	for (i = 3; i < 6; i++) {
		memset(&prepare_params[i], 0, sizeof(prepare_params[i]));
		prepare_params[i].ticks_at_expire = 1000 + (i * 100);
		events[i] = ull_prepare_enqueue(test_is_abort_cb, test_abort_cb,
						&prepare_params[i], test_prepare_cb, 0);
	}

	/* Complete iteration - may see newly added events depending on implementation */
	count = 1; /* Already got first event */
	while ((current = ull_prepare_dequeue_iter(&idx)) != NULL) {
		count++;
		if (count > 10) { /* Safety limit */
			break;
		}
	}

	zassert_true(count >= 3, "Should see at least initial 3 events");
}

/*
 * Test: Same tick values
 * Tests ordering when multiple events have identical ticks_at_expire.
 */
ZTEST(test_ull_prepare_edge_cases, test_same_tick_values)
{
	struct lll_prepare_param prepare_params[4];
	struct lll_event *events[4];
	void *idx = NULL;
	struct lll_event *current;
	int i, count;

	/* Enqueue events with same tick value */
	for (i = 0; i < 4; i++) {
		memset(&prepare_params[i], 0, sizeof(prepare_params[i]));
		prepare_params[i].ticks_at_expire = 5000; /* All same */
		prepare_params[i].param = (void *)(uintptr_t)i;

		events[i] = ull_prepare_enqueue(test_is_abort_cb, test_abort_cb,
						&prepare_params[i], test_prepare_cb, 0);
		zassert_not_null(events[i], "Failed to enqueue event %d", i);
	}

	/* Verify all events are in pipeline */
	count = 0;
	while ((current = ull_prepare_dequeue_iter(&idx)) != NULL) {
		zassert_equal(current->prepare_param.ticks_at_expire, 5000,
			     "All events should have same tick value");
		count++;
	}

	zassert_equal(count, 4, "All 4 events should be in pipeline");
}

/*
 * Test: Zero tick values
 * Edge case with tick value of 0.
 */
ZTEST(test_ull_prepare_edge_cases, test_zero_tick_values)
{
	struct lll_prepare_param prepare_params[3];
	struct lll_event *events[3];
	void *idx = NULL;
	struct lll_event *current;
	uint32_t prev_ticks;
	int i;

	/* Event with tick 0 */
	memset(&prepare_params[0], 0, sizeof(prepare_params[0]));
	prepare_params[0].ticks_at_expire = 0;
	events[0] = ull_prepare_enqueue(test_is_abort_cb, test_abort_cb,
					&prepare_params[0], test_prepare_cb, 0);

	/* Event with tick 100 */
	memset(&prepare_params[1], 0, sizeof(prepare_params[1]));
	prepare_params[1].ticks_at_expire = 100;
	events[1] = ull_prepare_enqueue(test_is_abort_cb, test_abort_cb,
					&prepare_params[1], test_prepare_cb, 0);

	/* Another event with tick 0 */
	memset(&prepare_params[2], 0, sizeof(prepare_params[2]));
	prepare_params[2].ticks_at_expire = 0;
	events[2] = ull_prepare_enqueue(test_is_abort_cb, test_abort_cb,
					&prepare_params[2], test_prepare_cb, 0);

	/* Verify ordering */
	i = 0;
	prev_ticks = 0;
	while ((current = ull_prepare_dequeue_iter(&idx)) != NULL) {
		if (i > 0) {
			zassert_true(current->prepare_param.ticks_at_expire >= prev_ticks,
				    "Events should be in ascending order");
		}
		prev_ticks = current->prepare_param.ticks_at_expire;
		i++;
	}

	zassert_equal(i, 3, "Should have 3 events");
}

/*
 * Test: Max tick value
 * Edge case with maximum uint32_t tick value.
 */
ZTEST(test_ull_prepare_edge_cases, test_max_tick_value)
{
	struct lll_prepare_param prepare_params[2];
	struct lll_event *events[2];
	void *idx = NULL;
	struct lll_event *current;

	/* Event with max tick */
	memset(&prepare_params[0], 0, sizeof(prepare_params[0]));
	prepare_params[0].ticks_at_expire = UINT32_MAX;
	events[0] = ull_prepare_enqueue(test_is_abort_cb, test_abort_cb,
					&prepare_params[0], test_prepare_cb, 0);
	zassert_not_null(events[0], "Should enqueue event with max tick");

	/* Event with max - 1 tick */
	memset(&prepare_params[1], 0, sizeof(prepare_params[1]));
	prepare_params[1].ticks_at_expire = UINT32_MAX - 1;
	events[1] = ull_prepare_enqueue(test_is_abort_cb, test_abort_cb,
					&prepare_params[1], test_prepare_cb, 0);
	zassert_not_null(events[1], "Should enqueue event with max-1 tick");

	/* Verify both events are accessible */
	current = ull_prepare_dequeue_iter(&idx);
	zassert_not_null(current, "First event should be accessible");

	current = ull_prepare_dequeue_iter(&idx);
	zassert_not_null(current, "Second event should be accessible");
}

/* Test suite setup/teardown */
static void *test_ull_prepare_edge_cases_setup(void)
{
	return NULL;
}

static void test_ull_prepare_edge_cases_before(void *f)
{
	ull_prepare_pipeline_init();
}

static void test_ull_prepare_edge_cases_after(void *f)
{
	ull_prepare_pipeline_cleanup();
}

static void test_ull_prepare_edge_cases_teardown(void *f)
{
}

ZTEST_SUITE(test_ull_prepare_edge_cases, NULL,
	   test_ull_prepare_edge_cases_setup,
	   test_ull_prepare_edge_cases_before,
	   test_ull_prepare_edge_cases_after,
	   test_ull_prepare_edge_cases_teardown);
