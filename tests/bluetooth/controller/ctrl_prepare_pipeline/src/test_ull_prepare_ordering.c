/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Prepare pipeline ordering tests
 *
 * Tests the ordered insertion behavior based on ticks_at_expire,
 * which is a key feature of PR #79444's linked list implementation.
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
 * Test: Events inserted in ascending time order
 * Validates that events enqueued in ascending order maintain that order.
 * With PR #79444, events should be ordered by ticks_at_expire.
 */
ZTEST(test_ull_prepare_ordering, test_ascending_order_insertion)
{
	struct lll_prepare_param prepare_params[4];
	struct lll_event *events[4];
	void *idx = NULL;
	struct lll_event *current;
	uint32_t prev_ticks = 0;
	int i;

	/* Enqueue events in ascending time order */
	for (i = 0; i < 4; i++) {
		memset(&prepare_params[i], 0, sizeof(prepare_params[i]));
		prepare_params[i].ticks_at_expire = 1000 + (i * 500);
		prepare_params[i].param = (void *)(uintptr_t)i;

		events[i] = ull_prepare_enqueue(test_is_abort_cb, test_abort_cb,
						&prepare_params[i], test_prepare_cb, 0);
		zassert_not_null(events[i], "Failed to enqueue event %d", i);
	}

	/* Iterate through pipeline and verify ascending order */
	i = 0;
	while ((current = ull_prepare_dequeue_iter(&idx)) != NULL) {
		if (i > 0) {
			zassert_true(current->prepare_param.ticks_at_expire >= prev_ticks,
				    "Events not in ascending order at position %d", i);
		}
		prev_ticks = current->prepare_param.ticks_at_expire;
		i++;
	}

	zassert_equal(i, 4, "Expected 4 events in pipeline");
}

/*
 * Test: Events inserted in descending time order
 * Validates that events enqueued in descending order are reordered
 * to ascending order by the ordered list implementation.
 */
ZTEST(test_ull_prepare_ordering, test_descending_order_insertion)
{
	struct lll_prepare_param prepare_params[4];
	struct lll_event *events[4];
	void *idx = NULL;
	struct lll_event *current;
	uint32_t prev_ticks = 0;
	int i;

	/* Enqueue events in descending time order */
	for (i = 0; i < 4; i++) {
		memset(&prepare_params[i], 0, sizeof(prepare_params[i]));
		/* Descending: 3500, 3000, 2500, 2000 */
		prepare_params[i].ticks_at_expire = 3500 - (i * 500);
		prepare_params[i].param = (void *)(uintptr_t)i;

		events[i] = ull_prepare_enqueue(test_is_abort_cb, test_abort_cb,
						&prepare_params[i], test_prepare_cb, 0);
		zassert_not_null(events[i], "Failed to enqueue event %d", i);
	}

	/* Iterate and verify they are in ascending order (reordered) */
	i = 0;
	while ((current = ull_prepare_dequeue_iter(&idx)) != NULL) {
		if (i > 0) {
			zassert_true(current->prepare_param.ticks_at_expire >= prev_ticks,
				    "Events not properly reordered at position %d", i);
		}
		prev_ticks = current->prepare_param.ticks_at_expire;
		i++;
	}

	zassert_equal(i, 4, "Expected 4 events in pipeline");
}

/*
 * Test: Events inserted in mixed time order
 * Validates ordering with randomly ordered insertion times.
 */
ZTEST(test_ull_prepare_ordering, test_mixed_order_insertion)
{
	struct lll_prepare_param prepare_params[5];
	struct lll_event *events[5];
	void *idx = NULL;
	struct lll_event *current;
	uint32_t prev_ticks = 0;
	uint32_t tick_values[] = {5000, 2000, 8000, 3000, 6000};
	int i;

	/* Enqueue events in mixed order */
	for (i = 0; i < 5; i++) {
		memset(&prepare_params[i], 0, sizeof(prepare_params[i]));
		prepare_params[i].ticks_at_expire = tick_values[i];
		prepare_params[i].param = (void *)(uintptr_t)i;

		events[i] = ull_prepare_enqueue(test_is_abort_cb, test_abort_cb,
						&prepare_params[i], test_prepare_cb, 0);
		zassert_not_null(events[i], "Failed to enqueue event %d", i);
	}

	/* Verify ascending order: should be 2000, 3000, 5000, 6000, 8000 */
	i = 0;
	while ((current = ull_prepare_dequeue_iter(&idx)) != NULL) {
		if (i > 0) {
			zassert_true(current->prepare_param.ticks_at_expire >= prev_ticks,
				    "Events not in ascending order at position %d", i);
		}
		prev_ticks = current->prepare_param.ticks_at_expire;
		i++;
	}

	zassert_equal(i, 5, "Expected 5 events in pipeline");
}

/*
 * Test: Resume events always placed at tail
 * Critical test for PR #79444: Resume events should go to tail regardless
 * of their ticks_at_expire value.
 */
ZTEST(test_ull_prepare_ordering, test_resume_events_at_tail)
{
	struct lll_prepare_param prepare_params[6];
	struct lll_event *events[6];
	void *idx = NULL;
	struct lll_event *current;
	int i, count;
	int resume_count = 0;

	/* Enqueue mix of normal and resume events */
	/* Normal event at 1000 */
	memset(&prepare_params[0], 0, sizeof(prepare_params[0]));
	prepare_params[0].ticks_at_expire = 1000;
	events[0] = ull_prepare_enqueue(test_is_abort_cb, test_abort_cb,
					&prepare_params[0], test_prepare_cb, 0);

	/* Resume event at 500 (earlier time, but should go to tail) */
	memset(&prepare_params[1], 0, sizeof(prepare_params[1]));
	prepare_params[1].ticks_at_expire = 500;
	events[1] = ull_prepare_enqueue(test_is_abort_cb, test_abort_cb,
					&prepare_params[1], test_prepare_cb, 1);

	/* Normal event at 2000 */
	memset(&prepare_params[2], 0, sizeof(prepare_params[2]));
	prepare_params[2].ticks_at_expire = 2000;
	events[2] = ull_prepare_enqueue(test_is_abort_cb, test_abort_cb,
					&prepare_params[2], test_prepare_cb, 0);

	/* Resume event at 100 (earliest time, but should go to tail) */
	memset(&prepare_params[3], 0, sizeof(prepare_params[3]));
	prepare_params[3].ticks_at_expire = 100;
	events[3] = ull_prepare_enqueue(test_is_abort_cb, test_abort_cb,
					&prepare_params[3], test_prepare_cb, 1);

	/* Normal event at 1500 */
	memset(&prepare_params[4], 0, sizeof(prepare_params[4]));
	prepare_params[4].ticks_at_expire = 1500;
	events[4] = ull_prepare_enqueue(test_is_abort_cb, test_abort_cb,
					&prepare_params[4], test_prepare_cb, 0);

	/* Resume event at 3000 */
	memset(&prepare_params[5], 0, sizeof(prepare_params[5]));
	prepare_params[5].ticks_at_expire = 3000;
	events[5] = ull_prepare_enqueue(test_is_abort_cb, test_abort_cb,
					&prepare_params[5], test_prepare_cb, 1);

	/* Verify order: normal events in ascending order, then resume events */
	count = 0;
	while ((current = ull_prepare_dequeue_iter(&idx)) != NULL) {
		if (current->is_resume) {
			/* Once we hit a resume event, all following should be resume */
			resume_count++;
		} else {
			/* Normal events should come before resume events */
			zassert_equal(resume_count, 0,
				     "Normal event found after resume event at position %d",
				     count);
		}
		count++;
	}

	zassert_equal(count, 6, "Expected 6 events in pipeline");
	zassert_equal(resume_count, 3, "Expected 3 resume events");
}

/*
 * Test: Ordering with aborted events
 * Aborted events can be placed anywhere but don't affect ordering logic.
 */
ZTEST(test_ull_prepare_ordering, test_ordering_with_aborted_events)
{
	struct lll_prepare_param prepare_params[5];
	struct lll_event *events[5];
	void *idx = NULL;
	struct lll_event *current;
	uint32_t prev_non_aborted_ticks = 0;
	int i;

	/* Enqueue events */
	for (i = 0; i < 5; i++) {
		memset(&prepare_params[i], 0, sizeof(prepare_params[i]));
		prepare_params[i].ticks_at_expire = 1000 + (i * 500);
		prepare_params[i].param = (void *)(uintptr_t)i;

		events[i] = ull_prepare_enqueue(test_is_abort_cb, test_abort_cb,
						&prepare_params[i], test_prepare_cb, 0);
		zassert_not_null(events[i], "Failed to enqueue event %d", i);
	}

	/* Mark some events as aborted */
	events[1]->is_aborted = 1;
	events[3]->is_aborted = 1;

	/* Iterate and verify non-aborted events maintain ordering */
	while ((current = ull_prepare_dequeue_iter(&idx)) != NULL) {
		if (!current->is_aborted && prev_non_aborted_ticks > 0) {
			zassert_true(current->prepare_param.ticks_at_expire >= 
				    prev_non_aborted_ticks,
				    "Non-aborted events not in ascending order");
		}
		if (!current->is_aborted) {
			prev_non_aborted_ticks = current->prepare_param.ticks_at_expire;
		}
	}
}

/*
 * Test: Interleaved resume and normal events
 * Tests complex scenario with multiple resume and normal events.
 */
ZTEST(test_ull_prepare_ordering, test_interleaved_resume_and_normal)
{
	struct lll_prepare_param prepare_params[8];
	struct lll_event *events[8];
	void *idx = NULL;
	struct lll_event *current;
	int i;
	bool found_resume = false;

	/* Alternate between normal and resume events */
	for (i = 0; i < 8; i++) {
		memset(&prepare_params[i], 0, sizeof(prepare_params[i]));
		prepare_params[i].ticks_at_expire = 1000 + (i * 300);
		prepare_params[i].param = (void *)(uintptr_t)i;

		/* Odd indices are resume events */
		uint8_t is_resume = (i % 2);
		events[i] = ull_prepare_enqueue(test_is_abort_cb, test_abort_cb,
						&prepare_params[i], test_prepare_cb,
						is_resume);
		zassert_not_null(events[i], "Failed to enqueue event %d", i);
	}

	/* Verify: normal events first, then resume events */
	while ((current = ull_prepare_dequeue_iter(&idx)) != NULL) {
		if (found_resume) {
			/* After first resume, all should be resume */
			zassert_equal(current->is_resume, 1,
				     "Non-resume event found after resume events");
		}
		if (current->is_resume) {
			found_resume = true;
		}
	}
}

/* Test suite setup/teardown */
static void *test_ull_prepare_ordering_setup(void)
{
	return NULL;
}

static void test_ull_prepare_ordering_before(void *f)
{
	/* Initialize pipeline state */
	ull_prepare_pipeline_init();
}

static void test_ull_prepare_ordering_after(void *f)
{
	/* Cleanup pipeline */
	ull_prepare_pipeline_cleanup();
}

static void test_ull_prepare_ordering_teardown(void *f)
{
}

ZTEST_SUITE(test_ull_prepare_ordering, NULL,
	   test_ull_prepare_ordering_setup,
	   test_ull_prepare_ordering_before,
	   test_ull_prepare_ordering_after,
	   test_ull_prepare_ordering_teardown);
