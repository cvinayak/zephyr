/*
 * Copyright (c) 2018-2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <errno.h>
#include <string.h>

#include <zephyr/toolchain.h>
#include <zephyr/sys/util.h>

#include "hal/ccm.h"
#include "hal/radio.h"
#include "hal/ticker.h"

#include "util/memq.h"

#include "ticker/ticker.h"

#include "pdu_df.h"
#include "pdu_vendor.h"
#include "pdu.h"

#include "lll/lll_prof_internal.h"

#include "lll.h"

#include "hal/debug.h"

/* Below profiling measurements using a sample with:
 * 1 connectable legacy advertising or 1 peripheral ACL,
 * plus
 * 3 extended advertising sets, scanning on 2M PHY, scanning on Coded PHY, and
 * 2 auxiliary scan set.
 */
#if defined(CONFIG_SOC_SERIES_NRF54H)
#define LLL_PROF_RADIO_MAX_US    92   /* Max. Radio Rx/Tx ISR, O(1)*/
#define LLL_PROF_LLL_MAX_US      245  /* Max. LLL prepare, O(1) */
#define LLL_PROF_ULL_HIGH_MAX_US 640  /* Max. Radio + LLL + ULL High, O(1) */
#define LLL_PROF_ULL_LOW_MAX_US  855  /* Max. ULL Low, O(n) n is ticker nodes */
#elif defined(CONFIG_SOC_COMPATIBLE_NRF54LX) && !defined(CONFIG_RISCV)
#define LLL_PROF_RADIO_MAX_US    92   /* Max. Radio Rx/Tx ISR, O(1)*/
#define LLL_PROF_LLL_MAX_US      92   /* Max. LLL prepare, O(1) */
#define LLL_PROF_ULL_HIGH_MAX_US 336  /* Max. Radio + LLL + ULL High, O(1) */
#define LLL_PROF_ULL_LOW_MAX_US  763  /* Max. ULL Low, O(n) n is ticker nodes */
#else /* !CONFIG_SOC_COMPATIBLE_NRF54LX */
#define LLL_PROF_RADIO_MAX_US    123  /* Max. Radio Rx/Tx ISR, O(1)*/
#define LLL_PROF_LLL_MAX_US      123  /* Max. LLL prepare, O(1) */
#define LLL_PROF_ULL_HIGH_MAX_US 733  /* Max. Radio + LLL + ULL High, O(1) */
#define LLL_PROF_ULL_LOW_MAX_US  1730 /* Max. ULL Low, O(n) n is ticker nodes */
#endif /* !CONFIG_SOC_COMPATIBLE_NRF54LX */

#define LLL_PROF_ASSERT(_val, _max) \
	{ \
		LL_ASSERT_MSG(((_val) <= (_max)), \
			      "%s: %u (%u), %u (%u), %u (%u), %u (%u)\n", __func__, \
			      HAL_TICKER_TICKS_TO_US(cputime_ticks_radio), \
			      LLL_PROF_RADIO_MAX_US, \
			      HAL_TICKER_TICKS_TO_US(cputime_ticks_lll), \
			      LLL_PROF_LLL_MAX_US, \
			      HAL_TICKER_TICKS_TO_US(cputime_ticks_ull_high), \
			      LLL_PROF_ULL_HIGH_MAX_US, \
			      HAL_TICKER_TICKS_TO_US(cputime_ticks_ull_low), \
			      LLL_PROF_ULL_LOW_MAX_US); \
	}

static int send(struct node_rx_pdu *rx);
static uint16_t latency_get(void);
static inline void sample(uint32_t *timestamp);
static inline void sample_ticks(uint32_t *timestamp_ticks);
static inline void delta(uint32_t timestamp, uint16_t *cputime);
static inline void delta_ticks(uint32_t timestamp_ticks, uint16_t *cputime_ticks);

static uint32_t timestamp_radio;
static uint32_t timestamp_lll;
static uint32_t timestamp_ull_high;
static uint32_t timestamp_ull_low;
static uint16_t cputime_radio;
static uint16_t cputime_lll;
static uint16_t cputime_ull_high;
static uint16_t cputime_ull_low;
static uint16_t latency_min = UINT16_MAX;
static uint16_t latency_max;
static uint16_t latency_prev;
static uint16_t cputime_min = UINT16_MAX;
static uint16_t cputime_max;
static uint16_t cputime_prev;
static uint32_t timestamp_latency;
static bool latency_valid;

static uint32_t timestamp_ticks_radio;
static uint32_t timestamp_ticks_lll;
static uint32_t timestamp_ticks_ull_high;
static uint32_t timestamp_ticks_ull_low;
static uint16_t cputime_ticks_radio;
static uint16_t cputime_ticks_lll;
static uint16_t cputime_ticks_ull_high;
static uint16_t cputime_ticks_ull_low;
static uint16_t cputime_ticks_overhead;
static uint16_t cputime_ticks_overhead_max;

/* Priority-0 handlers (radio, radio timer and LLL) execute back-to-back
 * without nesting. A handler entered within LLL_PROF_CHAIN_GAP_US of the
 * previous handler exit is accounted as continuing the same chain of
 * continuous CPU usage; this covers the exception tail-chaining, ISR wrapper
 * and profiling sample overheads at ticker tick resolution.
 */
#define LLL_PROF_CHAIN_GAP_US 2U

static void chain_enter(uint8_t type);
static void chain_exit(void);
static void chain_copy(uint8_t *dst_dur, uint8_t len);

static uint32_t chain_ticks_start;
static uint32_t chain_ticks_exit;
static uint32_t chain_ticks_handler;
static uint16_t chain_seq;
static uint8_t chain_cnt;
static uint8_t chain_dur[PROFILE_CHAIN_LEN_MAX];
static bool chain_valid;
static bool chain_chg;

static uint16_t chain_max_us;
static uint16_t chain_max_seq;
static uint8_t chain_max_cnt;
static uint8_t chain_max_dur[PROFILE_CHAIN_LEN_MAX];

static uint16_t overhead_max_chain_us;
static int16_t overhead_max_chain_lead;
static uint16_t overhead_max_chain_seq;
static uint8_t overhead_max_chain_cnt;
static uint8_t overhead_max_ticker_id;
static uint8_t overhead_max_chain_dur[PROFILE_CHAIN_LEN_MAX];

/* Ticker id of the prepare executed in each chain handler */
static uint8_t chain_tag[PROFILE_CHAIN_LEN_MAX];
static uint8_t chain_max_tag[PROFILE_CHAIN_LEN_MAX];
static uint8_t overhead_max_chain_tag[PROFILE_CHAIN_LEN_MAX];

/* Ticker expiry relative to chain start and ticks_slot of the tagged prepare */
static int16_t chain_at[PROFILE_CHAIN_LEN_MAX];
static uint16_t chain_slot[PROFILE_CHAIN_LEN_MAX];
static int16_t chain_max_at[PROFILE_CHAIN_LEN_MAX];
static uint16_t chain_max_slot[PROFILE_CHAIN_LEN_MAX];
static int16_t overhead_max_chain_at[PROFILE_CHAIN_LEN_MAX];
static uint16_t overhead_max_chain_slot[PROFILE_CHAIN_LEN_MAX];

BUILD_ASSERT(TICKER_PROF_EXPIRE_MAX == PROFILE_TICKER_LEN_MAX);

/* Reserved ticker expiries preceding the max chain */
static struct ticker_prof_expire chain_max_tkr[TICKER_PROF_EXPIRE_MAX];
static uint8_t chain_max_tkr_cnt;
static uint32_t chain_max_ticks_start;

/* Handler executing a resume prepare */
static uint32_t chain_ticks_resume;
static bool chain_resume_valid;

static int16_t ticks_rel_us(uint32_t ticks, uint32_t ticks_ref);

static inline void radio_enter(void)
{
	sample(&timestamp_radio);
	sample_ticks(&timestamp_ticks_radio);
}

static inline void radio_exit(void)
{
	delta(timestamp_radio, &cputime_radio);
	delta_ticks(timestamp_ticks_radio, &cputime_ticks_radio);
	LLL_PROF_ASSERT(cputime_ticks_radio, HAL_TICKER_US_TO_TICKS(LLL_PROF_RADIO_MAX_US));
}

void lll_prof_enter_radio(void)
{
	chain_enter(PROFILE_CHAIN_RADIO);
	radio_enter();
}

void lll_prof_exit_radio(void)
{
	radio_exit();
	chain_exit();
}

void lll_prof_enter_radio_tmr(void)
{
	chain_enter(PROFILE_CHAIN_RADIO_TMR);
	radio_enter();
}

void lll_prof_exit_radio_tmr(void)
{
	radio_exit();
	chain_exit();
}

uint16_t lll_prof_radio_get(void)
{
	return HAL_TICKER_TICKS_TO_US(cputime_ticks_radio);
}

void lll_prof_enter_lll(void)
{
	chain_enter(PROFILE_CHAIN_LLL);

	sample(&timestamp_lll);
	sample_ticks(&timestamp_ticks_lll);
}

void lll_prof_exit_lll(void)
{
	delta(timestamp_lll, &cputime_lll);
	delta_ticks(timestamp_ticks_lll, &cputime_ticks_lll);
	LLL_PROF_ASSERT(cputime_ticks_lll, HAL_TICKER_US_TO_TICKS(LLL_PROF_LLL_MAX_US));

	chain_exit();
}

uint16_t lll_prof_lll_get(void)
{
	return HAL_TICKER_TICKS_TO_US(cputime_ticks_lll);
}

void lll_prof_enter_ull_high(void)
{
	sample(&timestamp_ull_high);
	sample_ticks(&timestamp_ticks_ull_high);
}

void lll_prof_exit_ull_high(void)
{
	delta(timestamp_ull_high, &cputime_ull_high);
	delta_ticks(timestamp_ticks_ull_high, &cputime_ticks_ull_high);
	LLL_PROF_ASSERT(cputime_ticks_ull_high, HAL_TICKER_US_TO_TICKS(LLL_PROF_ULL_HIGH_MAX_US));
}

uint16_t lll_prof_ull_high_get(void)
{
	return HAL_TICKER_TICKS_TO_US(cputime_ticks_ull_high);
}

void lll_prof_enter_ull_low(void)
{
	sample(&timestamp_ull_low);
	sample_ticks(&timestamp_ticks_ull_low);
}

void lll_prof_exit_ull_low(void)
{
	delta(timestamp_ull_low, &cputime_ull_low);
	delta_ticks(timestamp_ticks_ull_low, &cputime_ticks_ull_low);
	LLL_PROF_ASSERT(cputime_ticks_ull_low, HAL_TICKER_US_TO_TICKS(LLL_PROF_ULL_LOW_MAX_US));
}

uint16_t lll_prof_ull_low_get(void)
{
	return HAL_TICKER_TICKS_TO_US(cputime_ticks_ull_low);
}

void lll_prof_overhead(uint16_t overhead_ticks, uint32_t ticks_at_event, uint8_t ticker_id)
{
	uint32_t ticks_now;
	uint32_t us;

	cputime_ticks_overhead = overhead_ticks;
	if (overhead_ticks <= cputime_ticks_overhead_max) {
		return;
	}

	cputime_ticks_overhead_max = overhead_ticks;
	overhead_max_ticker_id = ticker_id;
	chain_chg = true;

	/* Overhead sampled outside a priority-0 handler, no chain to snapshot */
	if (!chain_valid) {
		return;
	}

	ticks_now = ticker_ticks_now_get();
	overhead_max_chain_us = MIN(HAL_TICKER_TICKS_TO_US(ticks_now - chain_ticks_start),
				    UINT16_MAX);

	/* Positive lead when the chain started before the preempt timeout */
	overhead_max_chain_lead = -ticks_rel_us(chain_ticks_start, ticks_at_event);

	/* Current handler duration until the overhead sample */
	if (chain_cnt <= PROFILE_CHAIN_LEN_MAX) {
		us = HAL_TICKER_TICKS_TO_US(ticks_now - chain_ticks_handler);
		chain_dur[chain_cnt - 1U] = MIN(us, UINT8_MAX);
	}

	overhead_max_chain_seq = chain_seq;
	overhead_max_chain_cnt = chain_cnt;
	chain_copy(overhead_max_chain_dur, chain_cnt);
	(void)memcpy(overhead_max_chain_tag, chain_tag, sizeof(chain_tag));
	(void)memcpy(overhead_max_chain_at, chain_at, sizeof(chain_at));
	(void)memcpy(overhead_max_chain_slot, chain_slot, sizeof(chain_slot));
}

void lll_prof_chain_tag(uint8_t ticker_id, uint32_t ticks_at_expire, uint32_t ticks_slot)
{
	uint32_t us;

	uint8_t tag = ticker_id & PROFILE_CHAIN_TAG_ID_MASK;

	if (!chain_valid || (chain_cnt == 0U) || (chain_cnt > PROFILE_CHAIN_LEN_MAX)) {
		return;
	}

	if (chain_resume_valid && (chain_ticks_resume == chain_ticks_handler)) {
		tag |= PROFILE_CHAIN_TAG_RESUME;
	}

	chain_tag[chain_cnt - 1U] = tag;
	chain_at[chain_cnt - 1U] = ticks_rel_us(ticks_at_expire, chain_ticks_start);

	us = HAL_TICKER_TICKS_TO_US(ticks_slot);
	chain_slot[chain_cnt - 1U] = MIN(us, UINT16_MAX);
}

void lll_prof_chain_resume(void)
{
	chain_ticks_resume = chain_ticks_handler;
	chain_resume_valid = chain_valid;
}

static void chain_enter(uint8_t type)
{
	uint32_t ticks_now;
	uint8_t idx;

	ticks_now = ticker_ticks_now_get();
	if (!chain_valid ||
	    ((ticks_now - chain_ticks_exit) >
	     HAL_TICKER_US_TO_TICKS_CEIL(LLL_PROF_CHAIN_GAP_US))) {
		chain_ticks_start = ticks_now;
		chain_seq = 0U;
		chain_cnt = 0U;
		chain_valid = true;
	}

	chain_ticks_handler = ticks_now;

	idx = chain_cnt;
	if (idx < PROFILE_CHAIN_LEN_MAX) {
		chain_seq |= (uint16_t)(type & PROFILE_CHAIN_TYPE_MASK) <<
			     (idx * PROFILE_CHAIN_TYPE_BITS);
		chain_dur[idx] = 0U;
		chain_tag[idx] = PROFILE_CHAIN_TAG_NONE;
		chain_at[idx] = 0;
		chain_slot[idx] = 0U;
	}

	if (chain_cnt < UINT8_MAX) {
		chain_cnt++;
	}
}

static void chain_exit(void)
{
	uint32_t ticks_now;
	uint32_t us;

	ticks_now = ticker_ticks_now_get();
	chain_ticks_exit = ticks_now;

	if (chain_cnt <= PROFILE_CHAIN_LEN_MAX) {
		us = HAL_TICKER_TICKS_TO_US(ticks_now - chain_ticks_handler);
		chain_dur[chain_cnt - 1U] = MIN(us, UINT8_MAX);
	}

	us = HAL_TICKER_TICKS_TO_US(ticks_now - chain_ticks_start);
	if (us > chain_max_us) {
		chain_max_us = MIN(us, UINT16_MAX);
		chain_max_seq = chain_seq;
		chain_max_cnt = chain_cnt;
		chain_copy(chain_max_dur, chain_cnt);
		(void)memcpy(chain_max_tag, chain_tag, sizeof(chain_tag));
		(void)memcpy(chain_max_at, chain_at, sizeof(chain_at));
		(void)memcpy(chain_max_slot, chain_slot, sizeof(chain_slot));

		chain_max_ticks_start = chain_ticks_start;
		chain_max_tkr_cnt = ticker_prof_expire_get(chain_max_tkr);

		chain_chg = true;
	}
}

static int16_t ticks_rel_us(uint32_t ticks, uint32_t ticks_ref)
{
	uint32_t diff;
	uint32_t us;

	diff = ticker_ticks_diff_get(ticks, ticks_ref);
	if ((diff & BIT(HAL_TICKER_CNTR_MSBIT)) == 0U) {
		us = HAL_TICKER_TICKS_TO_US(diff);

		return MIN(us, INT16_MAX);
	}

	diff = ticker_ticks_diff_get(ticks_ref, ticks);
	us = HAL_TICKER_TICKS_TO_US(diff);

	return -(int16_t)MIN(us, INT16_MAX);
}

static void chain_copy(uint8_t *dst_dur, uint8_t len)
{
	len = MIN(len, PROFILE_CHAIN_LEN_MAX);
	(void)memcpy(dst_dur, chain_dur, len);
	(void)memset(&dst_dur[len], 0U, PROFILE_CHAIN_LEN_MAX - len);
}

void lll_prof_latency_capture(void)
{
	/* sample the packet timer, use it to calculate ISR latency
	 * and generate the profiling event at the end of the ISR.
	 */
	radio_tmr_sample();

	/* Initialize so that if we call lll_prof_latency_get before it is
	 * set, we can set it.
	 */
	timestamp_latency = UINT16_MAX;

	/* Latency is valid unless invalidated, i.e. on-air packet END
	 * timestamp has been captured.
	 */
	latency_valid = true;
}

void lll_prof_latency_invalidate(void)
{
	/* No on-air packet END timestamp captured, the ISR entry sample is
	 * used only to measure the ISR CPU time.
	 */
	latency_valid = false;
}

uint16_t lll_prof_latency_get(void)
{
	uint16_t latency;

	/* No on-air packet END timestamp captured, latency not measured */
	if (!latency_valid) {
		return 0U;
	}

	/* We are here before latency timestamp was fetched */
	if (timestamp_latency == UINT16_MAX) {
		/* get the ISR latency sample */
		timestamp_latency = radio_tmr_sample_get();
	}

	/* Get the elapsed time in us since on-air radio packet end to ISR
	 * entry.
	 */
	latency = latency_get();

	return latency;
}

#if defined(HAL_RADIO_GPIO_HAVE_PA_PIN)
static uint32_t timestamp_radio_end;

void lll_prof_radio_end_backup(void)
{
	/* PA enable is overwriting packet end used in ISR profiling, hence
	 * back it up for later use.
	 */
	timestamp_radio_end = radio_tmr_end_get();
}
#endif /* !HAL_RADIO_GPIO_HAVE_PA_PIN */

void lll_prof_cputime_capture(void)
{
	/* We are here before latency timestamp was fetched */
	if (timestamp_latency == UINT16_MAX) {
		/* get the ISR latency sample */
		timestamp_latency = radio_tmr_sample_get();
	}

	/* sample the packet timer again, use it to calculate ISR execution time
	 * and use it in profiling event
	 */
	radio_tmr_sample();
}

uint16_t lll_prof_cputime_get(void)
{
	uint16_t cputime;

	/* calculate the elapsed time in us since ISR entry */
	cputime = radio_tmr_sample_get() - timestamp_latency;

	return cputime;
}

void lll_prof_send(void)
{
	struct node_rx_pdu *rx;

	rx = lll_prof_reserve();
	lll_prof_reserve_send(rx);
}

struct node_rx_pdu *lll_prof_reserve(void)
{
	struct node_rx_pdu *rx;

	rx = ull_pdu_rx_alloc_peek(3);
	if (rx != NULL) {
		(void)ull_pdu_rx_alloc();
	}

	return rx;
}

void lll_prof_reserve_send(struct node_rx_pdu *rx)
{
	int err;

	err = send(rx);
	if ((err != 0) && (rx != NULL)) {
		rx->hdr.type = NODE_RX_TYPE_RELEASE;

		ull_rx_put_sched(rx->hdr.link, rx);
	}
}

static int send(struct node_rx_pdu *rx)
{
	uint16_t latency, cputime, prev;
	struct pdu_data *pdu;
	struct profile *p;
	uint8_t chg = 0U;

	if (latency_valid) {
		/* Latency consumed, require a new capture before it is measured again */
		latency_valid = false;

		/* Get the elapsed time in us since on-air radio packet end to
		 * ISR entry.
		 */
		latency = latency_get();

		/* check changes in min, avg and max of latency */
		if (latency > latency_max) {
			latency_max = latency;
			chg = 1U;
		}
		if (latency < latency_min) {
			latency_min = latency;
			chg = 1U;
		}

		/* check for +/- 1us change */
		prev = ((uint16_t)latency_prev + latency) >> 1;
		if (prev != latency_prev) {
			latency_prev = latency;
			chg = 1U;
		}
	} else {
		/* Latency not measured, no on-air packet END timestamp */
		latency = 0U;
	}

	/* ISR entry sample not fetched yet, as neither latency nor CPU time was
	 * captured in this ISR, capture them now.
	 */
	if (timestamp_latency == UINT16_MAX) {
		lll_prof_cputime_capture();

		timestamp_latency = UINT16_MAX;
	}

	/* calculate the elapsed time in us since ISR entry */
	cputime = lll_prof_cputime_get();

	/* check changes in min, avg and max */
	if (cputime > cputime_max) {
		cputime_max = cputime;
		chg = 1U;
	}

	if (cputime < cputime_min) {
		cputime_min = cputime;
		chg = 1U;
	}

	/* check for +/- 1us change */
	prev = ((uint16_t)cputime_prev + cputime) >> 1;
	if (prev != cputime_prev) {
		cputime_prev = cputime;
		chg = 1U;
	}

	if (chain_chg) {
		chg = 1U;
	}

	/* generate event if any change */
	if (!chg) {
		return -ENODATA;
	}

	/* Do not encode profile if no node rx allocated */
	if (rx == NULL) {
		return -ENOMEM;
	}

	/* Generate event with the allocated node rx */
	rx->hdr.type = NODE_RX_TYPE_PROFILE;
	rx->hdr.handle = NODE_RX_HANDLE_INVALID;

	pdu = (void *)rx->pdu;
	p = &pdu->profile;
	p->lcur = latency;
	p->lmin = latency_min;
	p->lmax = latency_max;
	p->cur = cputime;
	p->min = cputime_min;
	p->max = cputime_max;
	p->radio = cputime_radio;
	p->lll = cputime_lll;
	p->ull_high = cputime_ull_high;
	p->ull_low = cputime_ull_low;
	p->overhead = HAL_TICKER_TICKS_TO_US(cputime_ticks_overhead);
	p->overhead_max = HAL_TICKER_TICKS_TO_US(cputime_ticks_overhead_max);
	p->radio_ticks = cputime_ticks_radio;
	p->lll_ticks = cputime_ticks_lll;
	p->ull_high_ticks = cputime_ticks_ull_high;
	p->ull_low_ticks = cputime_ticks_ull_low;
	p->ovh_chain_us = overhead_max_chain_us;
	p->ovh_chain_lead = overhead_max_chain_lead;
	p->ovh_chain_seq = overhead_max_chain_seq;
	p->ovh_chain_cnt = overhead_max_chain_cnt;
	p->ovh_ticker_id = overhead_max_ticker_id;
	(void)memcpy(p->ovh_chain_dur, overhead_max_chain_dur, sizeof(p->ovh_chain_dur));
	p->chain_max_us = chain_max_us;
	p->chain_max_seq = chain_max_seq;
	p->chain_max_cnt = chain_max_cnt;
	(void)memcpy(p->chain_max_dur, chain_max_dur, sizeof(p->chain_max_dur));
	(void)memcpy(p->ovh_chain_tag, overhead_max_chain_tag, sizeof(p->ovh_chain_tag));
	(void)memcpy(p->chain_max_tag, chain_max_tag, sizeof(p->chain_max_tag));
	(void)memcpy(p->ovh_chain_at, overhead_max_chain_at, sizeof(p->ovh_chain_at));
	(void)memcpy(p->ovh_chain_slot, overhead_max_chain_slot, sizeof(p->ovh_chain_slot));
	(void)memcpy(p->chain_max_at, chain_max_at, sizeof(p->chain_max_at));
	(void)memcpy(p->chain_max_slot, chain_max_slot, sizeof(p->chain_max_slot));

	p->tkr_cnt = chain_max_tkr_cnt;
	for (uint8_t i = 0U; i < chain_max_tkr_cnt; i++) {
		const struct ticker_prof_expire *e = &chain_max_tkr[i];
		int16_t at;
		uint16_t us;

		p->tkr_id[i] = e->ticker_id;
		p->tkr_prev_id[i] = e->ticker_id_slot_previous;
		p->tkr_reserved[i] = e->slot_reserved;

		at = ticks_rel_us(e->ticks_at_expire, chain_max_ticks_start);
		(void)memcpy(&p->tkr_at[i], &at, sizeof(at));
		us = MIN(HAL_TICKER_TICKS_TO_US(e->ticks_slot), UINT16_MAX);
		(void)memcpy(&p->tkr_slot[i], &us, sizeof(us));
		us = MIN(HAL_TICKER_TICKS_TO_US(e->ticks_slot_previous), UINT16_MAX);
		(void)memcpy(&p->tkr_prev[i], &us, sizeof(us));
		us = MIN(HAL_TICKER_TICKS_TO_US(e->ticks_to_expire_minus), UINT16_MAX);
		(void)memcpy(&p->tkr_late[i], &us, sizeof(us));
		us = MIN(HAL_TICKER_TICKS_TO_US(e->ticks_drift), UINT16_MAX);
		(void)memcpy(&p->tkr_drift[i], &us, sizeof(us));
		(void)memcpy(&p->tkr_lazy[i], &e->lazy, sizeof(e->lazy));
	}

	chain_chg = false;

	ull_rx_put_sched(rx->hdr.link, rx);

	return 0;
}

static uint16_t latency_get(void)
{
	uint16_t latency;

	/* calculate the elapsed time in us since on-air radio packet end
	 * to ISR entry
	 */
	if (!IS_ENABLED(CONFIG_BT_CTLR_SW_SWITCH_SINGLE_TIMER)) {
#if defined(HAL_RADIO_GPIO_HAVE_PA_PIN)
		latency = timestamp_latency - timestamp_radio_end;
#else /* !HAL_RADIO_GPIO_HAVE_PA_PIN */
		latency = timestamp_latency - radio_tmr_end_get();
#endif /* !HAL_RADIO_GPIO_HAVE_PA_PIN */
	} else {
		latency = timestamp_latency;
	}

	return latency;
}

static inline void sample(uint32_t *timestamp)
{
	radio_tmr_sample();
	*timestamp = radio_tmr_sample_get();
}

static inline void sample_ticks(uint32_t *timestamp_ticks)
{
	*timestamp_ticks = ticker_ticks_now_get();
}

static inline void delta(uint32_t timestamp, uint16_t *cputime)
{
	uint32_t delta;

	radio_tmr_sample();
	delta = radio_tmr_sample_get() - timestamp;
	if (delta < UINT16_MAX && delta > *cputime) {
		*cputime = delta;
	}
}

static inline void delta_ticks(uint32_t timestamp_ticks, uint16_t *cputime_ticks)
{
	uint32_t delta;

	delta = ticker_ticks_now_get() - timestamp_ticks;
	if (delta < UINT16_MAX && delta > *cputime_ticks) {
		*cputime_ticks = delta;
	}
}
