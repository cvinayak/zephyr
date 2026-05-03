/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef UUT_H_
#define UUT_H_

/*
 * lll.h has no include guards: include it once here.  main.c must NOT
 * include it (or any header that transitively includes it) a second time.
 */
#include "util/memq.h"
#include "util/mayfly.h"
#include "ticker/ticker.h"
#include "lll.h"

/*
 * Reset all internal state of lll_prepare_resolve and its helper
 * functions to a clean initial condition. Call before each test case.
 */
void lll_prepare_resolve_test_reset(void);

/*
 * Program the event.curr fields that lll_prepare_resolve inspects to
 * decide whether to enqueue or execute a prepare.
 *
 * @param abort_cb     Value for event.curr.abort_cb (NULL == no active event)
 * @param is_abort_cb  Value for event.curr.is_abort_cb
 * @param param        Value for event.curr.param
 */
void lll_prepare_resolve_test_set_curr(lll_abort_cb_t abort_cb,
				       lll_is_abort_cb_t is_abort_cb,
				       void *param);

/* Return event.curr.param (verify lll_prepare_resolve updated it). */
void *lll_prepare_resolve_test_get_curr_param(void);

/* Return event.curr.abort_cb (verify lll_prepare_resolve updated it). */
lll_abort_cb_t lll_prepare_resolve_test_get_curr_abort_cb(void);

#endif /* UUT_H_ */
