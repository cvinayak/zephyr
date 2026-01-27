/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Test implementation of ULL prepare pipeline
 *
 * This file provides a minimal implementation of the prepare pipeline
 * functions for unit testing. It includes only the prepare pipeline
 * functions from ull.c without the full ULL dependencies.
 */

#include <string.h>
#include <zephyr/types.h>

#include "util/util.h"
#include "util/mem.h"

#include "lll.h"

/* BIT macro if not already defined */
#ifndef BIT
#define BIT(n) (1UL << (n))
#endif

/* Ticker constants - for testing we use 32-bit counter */
#define HAL_TICKER_CNTR_MSBIT 31
#define HAL_TICKER_CNTR_MASK 0xFFFFFFFF

/* Ticker tick difference calculation */
static inline uint32_t ticker_ticks_diff_get(uint32_t ticks_now, uint32_t ticks_old)
{
	return ((ticks_now - ticks_old) & HAL_TICKER_CNTR_MASK);
}

/* Event pipeline maximum size - use value from lll.h if available */
#ifndef EVENT_PIPELINE_MAX
#define EVENT_PIPELINE_MAX (7U + (EVENT_DEFER_MAX))
#endif

#define EVENT_PIPELINE_EVENT_SIZE (sizeof(struct lll_event) + sizeof(void *))

/* Prepare pipeline structure */
static struct {
	struct {
		void *free;
		uint8_t pool[EVENT_PIPELINE_MAX * EVENT_PIPELINE_EVENT_SIZE];
	} mem;
	void **head;
	void **tail;
} pipeline;

/* Initialize the prepare pipeline */
void ull_prepare_pipeline_init(void)
{
	mem_init(pipeline.mem.pool, EVENT_PIPELINE_EVENT_SIZE,
		 sizeof(pipeline.mem.pool) / EVENT_PIPELINE_EVENT_SIZE,
		 &pipeline.mem.free);
	pipeline.head = NULL;
	pipeline.tail = NULL;
}

/* Reset/cleanup the prepare pipeline */
void ull_prepare_pipeline_cleanup(void)
{
	/* Free all allocated events */
	void **curr = pipeline.head;
	while (curr) {
		void **next = *curr;
		mem_release(curr, &pipeline.mem.free);
		curr = next;
	}
	
	pipeline.head = NULL;
	pipeline.tail = NULL;
}

struct lll_event *ull_prepare_enqueue(lll_is_abort_cb_t is_abort_cb,
				      lll_abort_cb_t abort_cb,
				      struct lll_prepare_param *prepare_param,
				      lll_prepare_cb_t prepare_cb,
				      uint8_t is_resume)
{
	struct lll_event *e;
	void **next;

	/* Allocate lll_event */
	next = mem_acquire(&pipeline.mem.free);
	if (!next) {
		return NULL;
	}

	e = (void *)((uint8_t *)next + sizeof(void *));

	memcpy(&e->prepare_param, prepare_param, sizeof(e->prepare_param));
	e->prepare_cb = prepare_cb;
	e->is_abort_cb = is_abort_cb;
	e->abort_cb = abort_cb;
	e->is_resume = is_resume;
	e->is_aborted = 0U;

	/* Enqueue lll_event */
	*next = NULL;
	if (pipeline.tail) {
		struct lll_event *e_curr;
		uint32_t diff;

		/* Should the prepare be placed as the tail? */
		e_curr = (void *)((uint8_t *)pipeline.tail + sizeof(void *));
		diff = ticker_ticks_diff_get(prepare_param->ticks_at_expire,
					     e_curr->prepare_param.ticks_at_expire);
		if (is_resume ||
		    (!e_curr->is_aborted && !e_curr->is_resume &&
		     ((diff & BIT(HAL_TICKER_CNTR_MSBIT)) == 0U))) {
			*pipeline.tail = next;
			pipeline.tail = next;
		} else {
			/* Should the prepare be placed as the head? */
			e_curr = (void *)((uint8_t *)pipeline.head + sizeof(void *));
			diff = ticker_ticks_diff_get(e_curr->prepare_param.ticks_at_expire,
						     prepare_param->ticks_at_expire);
			if (!e_curr->is_aborted &&
			    (e_curr->is_resume ||
			     (diff && ((diff & BIT(HAL_TICKER_CNTR_MSBIT)) == 0U)))) {
				*next = pipeline.head;
				pipeline.head = next;
			} else {
				void **prev;
				void **curr;

				prev = NULL;
				curr = pipeline.head;
				e_curr = (void *)((uint8_t *)curr + sizeof(void *));
				do {
					if (!e_curr->is_aborted && !e_curr->is_resume) {
						prev = curr;
					}

					curr = *curr;
					if (!curr) {
						break;
					}

					e_curr = (void *)((uint8_t *)curr + sizeof(void *));
					diff = ticker_ticks_diff_get(
							prepare_param->ticks_at_expire,
							e_curr->prepare_param.ticks_at_expire);
				} while (!e_curr->is_resume &&
					 (e_curr->is_aborted ||
					  ((diff & BIT(HAL_TICKER_CNTR_MSBIT)) == 0U)));

				if (!prev) {
					*next = pipeline.head;
					pipeline.head = next;
					if (!*next) {
						pipeline.tail = next;
					}
				} else {
					*next = *prev;
					*prev = next;
					if (!*next) {
						pipeline.tail = next;
					}
				}
			}
		}
	} else {
		pipeline.head = next;
		pipeline.tail = next;
	}

	return e;
}

void *ull_prepare_dequeue_get(void)
{
	void *e;

	/* peak lll_event */
	if (pipeline.head) {
		e = (uint8_t *)pipeline.head + sizeof(void *);
	} else {
		e = NULL;
	}

	return e;
}

void *ull_prepare_dequeue_iter(void **idx)
{
	void *e;

	/* Start at the head */
	if (!*idx) {
		*idx = pipeline.head;
	}

	/* No more in the list */
	if (!*idx) {
		return NULL;
	}

	/* Peak lll_event */
	e = (uint8_t *)*idx + sizeof(void *);

	/* Proceed to next */
	*idx = *((void **)*idx);

	return e;
}

void ull_prepare_dequeue(uint8_t caller_id)
{
	/* For testing, we just need a stub implementation
	 * Full implementation would require mayfly, ticker, etc.
	 */
	(void)caller_id;
}
