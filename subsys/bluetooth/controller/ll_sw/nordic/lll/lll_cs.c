/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdint.h>
#include <stdbool.h>

#include <soc.h>
#include <zephyr/toolchain.h>
#include <zephyr/sys/util.h>
#include <zephyr/bluetooth/hci_types.h>

#include "hal/cpu.h"
#include "hal/ccm.h"
#include "hal/radio.h"
#include "hal/radio_cs.h"
#include "hal/ticker.h"

#include "util/memq.h"

#include "pdu_df.h"
#include "lll/pdu_vendor.h"
#include "pdu.h"

#include "lll.h"
#include "lll_vendor.h"
#include "lll_clock.h"
#include "lll_cs.h"

#include "lll_internal.h"

#include "ull_cs_internal.h"

#include "hal/debug.h"

static int init_reset(void);
static int prepare_cb(struct lll_prepare_param *p);
static int is_abort_cb(void *next, void *curr, lll_prepare_cb_t *resume_cb);
static void abort_cb(struct lll_prepare_param *prepare_param, void *param);
static void isr_done(void *param);

/* ---- Implementation ---- */

int lll_cs_init(void)
{
	int err;

	err = init_reset();
	if (err) {
		return err;
	}

	return 0;
}

int lll_cs_reset(void)
{
	int err;

	err = init_reset();
	if (err) {
		return err;
	}

	return 0;
}

void lll_cs_prepare(void *param)
{
	int err;

	err = lll_hfclock_on();
	LL_ASSERT_ERR(err >= 0);

	err = lll_prepare(is_abort_cb, abort_cb, prepare_cb, 0U, param);
	LL_ASSERT_ERR((err == 0) || (err == -EINPROGRESS));
}

static int prepare_cb(struct lll_prepare_param *p)
{
	struct lll_cs *lll;
	int err;

	DEBUG_RADIO_START_O(1);

	lll = p->param;

	/* TODO: Implement LLL preparation */

	err = lll_prepare_done(lll);
	LL_ASSERT_ERR(!err);

	DEBUG_RADIO_START_O(1);
	return 0;
}

static int is_abort_cb(void *next, void *curr, lll_prepare_cb_t *resume_cb)
{
	ARG_UNUSED(resume_cb);

	if (next != curr) {
		return -ECANCELED;
	}

	return 0;
}

static void abort_cb(struct lll_prepare_param *prepare_param, void *param)
{
	int err;

	if (!prepare_param) {
		radio_isr_set(isr_done, param);
		radio_disable();

		return;
	}

	err = lll_hfclock_off();
	LL_ASSERT_ERR(err >= 0);

	(void)ull_done_extra_type_set(EVENT_DONE_EXTRA_TYPE_NONE);

	lll_done(param);
}

static void isr_done(void *param)
{
	struct lll_cs *lll = param;

	lll_isr_status_reset();

	if (lll->rx_pending) {
		lll->rx_pending = 0U;
		ull_rx_sched();
	}

	(void)ull_done_extra_type_set(EVENT_DONE_EXTRA_TYPE_NONE);

	lll_isr_cleanup(param);
}

static int init_reset(void)
{
	return 0;
}
