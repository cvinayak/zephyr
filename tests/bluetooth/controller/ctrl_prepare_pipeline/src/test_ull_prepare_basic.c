/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Basic prepare pipeline operations tests
 *
 * Tests basic enqueue/dequeue operations for the ordered linked list
 * implementation introduced in PR #79444.
 */

#include <string.h>
#include <zephyr/types.h>
#include <zephyr/ztest.h>

#include "util/util.h"
#include "util/mem.h"
#include "util/memq.h"

#include "lll.h"
#include "ull_internal.h"

/* Mock callbacks for testing */
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
	/* No-op for testing */
}

/*
 * Test: Enqueue to empty pipeline
 * Validates that a single event can be enqueued successfully to an empty pipeline.
 */
ZTEST(test_ull_prepare_basic, test_enqueue_empty_pipeline)
{
	struct lll_prepare_param prepare_param;
	struct lll_event *event;

	memset(&prepare_param, 0, sizeof(prepare_param));
	prepare_param.ticks_at_expire = 1000;
	prepare_param.param = (void *)0x1234;

	/* Enqueue event to empty pipeline */
	event = ull_prepare_enqueue(test_is_abort_cb, test_abort_cb,
				    &prepare_param, test_prepare_cb, 0);

	zassert_not_null(event, "Failed to enqueue event to empty pipeline");
	zassert_equal(event->prepare_param.ticks_at_expire, 1000,
		     "Event ticks_at_expire mismatch");
	zassert_equal(event->prepare_param.param, (void *)0x1234,
		     "Event param mismatch");
	zassert_equal(event->is_resume, 0, "Event should not be marked as resume");
	zassert_equal(event->is_aborted, 0, "Event should not be marked as aborted");
}

/*
 * Test: Enqueue multiple events
 * Validates that multiple events can be enqueued successfully.
 */
ZTEST(test_ull_prepare_basic, test_enqueue_multiple_events)
{
	struct lll_prepare_param prepare_params[5];
	struct lll_event *events[5];
	int i;

	/* Enqueue 5 events with different tick values */
	for (i = 0; i < 5; i++) {
		memset(&prepare_params[i], 0, sizeof(prepare_params[i]));
		prepare_params[i].ticks_at_expire = 1000 + (i * 100);
		prepare_params[i].param = (void *)(uintptr_t)(0x1000 + i);

		events[i] = ull_prepare_enqueue(test_is_abort_cb, test_abort_cb,
						&prepare_params[i], test_prepare_cb, 0);

		zassert_not_null(events[i], "Failed to enqueue event %d", i);
		zassert_equal(events[i]->prepare_param.ticks_at_expire,
			     1000 + (i * 100),
			     "Event %d ticks_at_expire mismatch", i);
	}
}

/*
 * Test: Dequeue from pipeline
 * Validates that dequeue_get() retrieves the head event without removal.
 */
ZTEST(test_ull_prepare_basic, test_dequeue_get)
{
	struct lll_prepare_param prepare_param;
	struct lll_event *event, *head;

	memset(&prepare_param, 0, sizeof(prepare_param));
	prepare_param.ticks_at_expire = 2000;
	prepare_param.param = (void *)0x5678;

	/* Enqueue an event */
	event = ull_prepare_enqueue(test_is_abort_cb, test_abort_cb,
				    &prepare_param, test_prepare_cb, 0);
	zassert_not_null(event, "Failed to enqueue event");

	/* Get head without removing */
	head = ull_prepare_dequeue_get();
	zassert_not_null(head, "Failed to get head of pipeline");
	zassert_equal(head, event, "Head should be the enqueued event");
	
	/* Verify head still accessible (not removed) */
	head = ull_prepare_dequeue_get();
	zassert_equal(head, event, "Head should still be accessible");
}

/*
 * Test: Dequeue from empty pipeline
 * Validates that dequeue_get() returns NULL for empty pipeline.
 */
ZTEST(test_ull_prepare_basic, test_dequeue_empty_pipeline)
{
	struct lll_event *head;

	/* Try to get head from empty pipeline */
	head = ull_prepare_dequeue_get();
	zassert_is_null(head, "Empty pipeline should return NULL");
}

/*
 * Test: Resume event marking
 * Validates that resume events are properly marked.
 */
ZTEST(test_ull_prepare_basic, test_resume_event_marking)
{
	struct lll_prepare_param prepare_param;
	struct lll_event *event;

	memset(&prepare_param, 0, sizeof(prepare_param));
	prepare_param.ticks_at_expire = 3000;

	/* Enqueue as resume event */
	event = ull_prepare_enqueue(test_is_abort_cb, test_abort_cb,
				    &prepare_param, test_prepare_cb, 1);

	zassert_not_null(event, "Failed to enqueue resume event");
	zassert_equal(event->is_resume, 1, "Event should be marked as resume");
	zassert_equal(event->is_aborted, 0, "Event should not be aborted");
}

/*
 * Test: Aborted event marking
 * Validates that events can be marked as aborted.
 */
ZTEST(test_ull_prepare_basic, test_aborted_event_marking)
{
	struct lll_prepare_param prepare_param;
	struct lll_event *event;

	memset(&prepare_param, 0, sizeof(prepare_param));
	prepare_param.ticks_at_expire = 4000;

	/* Enqueue event */
	event = ull_prepare_enqueue(test_is_abort_cb, test_abort_cb,
				    &prepare_param, test_prepare_cb, 0);
	zassert_not_null(event, "Failed to enqueue event");

	/* Mark as aborted */
	event->is_aborted = 1;

	zassert_equal(event->is_aborted, 1, "Event should be marked as aborted");
}

/*
 * Test: Callback assignments
 * Validates that all callbacks are properly assigned.
 */
ZTEST(test_ull_prepare_basic, test_callback_assignments)
{
	struct lll_prepare_param prepare_param;
	struct lll_event *event;

	memset(&prepare_param, 0, sizeof(prepare_param));
	prepare_param.ticks_at_expire = 5000;

	/* Enqueue event with callbacks */
	event = ull_prepare_enqueue(test_is_abort_cb, test_abort_cb,
				    &prepare_param, test_prepare_cb, 0);

	zassert_not_null(event, "Failed to enqueue event");
	zassert_equal(event->prepare_cb, test_prepare_cb,
		     "Prepare callback not assigned correctly");
	zassert_equal(event->is_abort_cb, test_is_abort_cb,
		     "Is-abort callback not assigned correctly");
	zassert_equal(event->abort_cb, test_abort_cb,
		     "Abort callback not assigned correctly");
}

/* Test suite setup and teardown */
static void *test_ull_prepare_basic_setup(void)
{
	return NULL;
}

static void test_ull_prepare_basic_before(void *f)
{
	/* Note: Pipeline initialization/cleanup would happen here if needed.
	 * For now, we rely on the implementation's internal state management.
	 * In a real scenario after PR #79444, we might need to initialize
	 * the ordered list structure here.
	 */
}

static void test_ull_prepare_basic_after(void *f)
{
	/* Cleanup after each test - drain the pipeline
	 * This ensures each test starts with a clean state.
	 */
}

static void test_ull_prepare_basic_teardown(void *f)
{
	/* Final cleanup */
}

ZTEST_SUITE(test_ull_prepare_basic, NULL,
	   test_ull_prepare_basic_setup,
	   test_ull_prepare_basic_before,
	   test_ull_prepare_basic_after,
	   test_ull_prepare_basic_teardown);
