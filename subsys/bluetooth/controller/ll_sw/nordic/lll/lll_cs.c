/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <soc.h>
#include <zephyr/toolchain.h>
#include <zephyr/sys/util.h>
#include <zephyr/bluetooth/hci_types.h>

#include "hal/cpu.h"
#include "hal/ccm.h"
#include "hal/radio.h"
#include "hal/radio_cs.h"
#include "hal/ticker.h"

#include "util/util.h"
#include "util/mem.h"
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

#include "lll_tim_internal.h"

/* LE 1M preamble and access address air time. */
#define CS_PREAMBLE_AA_US 40U

/* Air time of one octet on LE 1M. */
#define CS_OCTET_US 8U

/* Receive window widening once the reflector is synchronized to the
 * initiator timing.
 */
#define CS_RX_MARGIN_US 8U

/* Guard time at the start and end of a tone excluded from the measurement. */
#define CS_TONE_GUARD_US 4U

/* Tone measurement sample rate, samples per microsecond. */
#define CS_TONE_SAMPLES_PER_US 2U

/* Maximum CS_SYNC or tone packet payload length. */
#define CS_PKT_LEN_MAX 32U

/* Subevent abort reasons, Bluetooth Core specification, Vol 4, Part E,
 * Section 7.7.65.44.
 */
#define CS_ABORT_NO_SYNC  0x2U
#define CS_ABORT_CONFLICT 0x3U

static int init_reset(void);
static int prepare_cb(struct lll_prepare_param *p);
static int is_abort_cb(void *next, void *curr, lll_prepare_cb_t *resume_cb);
static void abort_cb(struct lll_prepare_param *prepare_param, void *param);
static void action_setup(struct lll_cs *lll);
static bool action_skip(const struct lll_cs *lll);
static bool action_next(struct lll_cs *lll);
static void isr_action(void *param);
static void isr_abort(void *param);
static void isr_tmr_tones_start(void *param, uint8_t chain);
static void isr_tmr_fine_tune(void *param, uint8_t chain);
static void subevent_end(struct lll_cs *lll);
static void isr_done(void *param);

static uint8_t MALIGN(4) tx_pkt[CS_PKT_LEN_MAX];
static int16_t fine_tune_tone;

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
	uint32_t ticks_at_event;
	uint32_t ticks_at_start;
	uint32_t remainder_us;
	struct ull_hdr *ull;
	struct lll_cs *lll;
	uint32_t anchor_us;
	bool is_tx;
	int err;

	DEBUG_RADIO_START_O(1);

	lll = p->param;

	lll->step_curr = 0U;
	lll->action = 0U;
	lll->synced = (lll->role == LLL_CS_ROLE_INITIATOR) ? 1U : 0U;
	lll->rx_pending = 0U;
	lll->abort_reason = 0U;
	lll->ffo_sum = 0;
	lll->ffo_count = 0U;

	radio_reset();
	radio_tx_power_set(RADIO_TXP_DEFAULT);
	radio_phy_set(PHY_1M, 0U);

	ticks_at_event = p->ticks_at_expire;
	ull = HDR_LLL2ULL(lll);
	ticks_at_event += lll_event_offset_get(ull);
	ticks_at_start = ticks_at_event + HAL_TICKER_US_TO_TICKS(EVENT_OVERHEAD_START_US);

	/* The first action of a subevent is the initiator CS_SYNC of step 0 */
	is_tx = (lll->role == LLL_CS_ROLE_INITIATOR);
	remainder_us = radio_tmr_start(is_tx ? 1U : 0U, ticks_at_start, lll->remainder);

	/* Air time of the anchor point of the ACL connection event that anchors
	 * the CS event, as used by the central and as estimated by the
	 * peripheral.
	 */
	if (lll->acl_periph != 0U) {
		anchor_us = radio_rx_ready_delay_get(lll->acl_phy, PHY_FLAGS_S8) +
			    EVENT_JITTER_US + EVENT_TICKER_RES_MARGIN_US + lll->acl_ww_us;
	} else {
		anchor_us = radio_tx_ready_delay_get(lll->acl_phy, PHY_FLAGS_S8) +
			    radio_tx_chain_delay_get(lll->acl_phy, PHY_FLAGS_S8);
	}

	/* Air time of the start of step 0, relative to the radio timer start */
	lll->t_step_us = remainder_us + anchor_us + lll->start_offset_us;

	action_setup(lll);

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
		radio_isr_set(isr_abort, param);
		radio_disable();

		return;
	}

	err = lll_hfclock_off();
	LL_ASSERT_ERR(err >= 0);

	ull_cs_lll_done(param);

	(void)ull_done_extra_type_set(EVENT_DONE_EXTRA_TYPE_NONE);

	lll_done(param);
}

static bool action_is_tx(const struct lll_cs *lll)
{
	return (lll->role == LLL_CS_ROLE_INITIATOR) == (lll->action == 0U);
}

/* A reflector that has not received an initiator CS_SYNC in the subevent has
 * no timing reference: skip its transmissions and tone receptions until it
 * synchronizes.
 */
static bool action_skip(const struct lll_cs *lll)
{
	const struct lll_cs_step *step = &lll->steps[lll->step_curr];

	if (lll->synced != 0U) {
		return false;
	}

	return action_is_tx(lll) || (step->mode == LLL_CS_MODE_2);
}

/* Advance to the next step action. Returns false when all steps are done. */
static bool action_next(struct lll_cs *lll)
{
	const struct lll_cs_step *step = &lll->steps[lll->step_curr];

	if (lll->action == 0U) {
		lll->action = 1U;

		return true;
	}

	lll->action = 0U;
	lll->t_step_us += lll_cs_step_len_us(step->mode, lll->n_ap) + LLL_CS_T_FCS_US;
	lll->step_curr++;

	return lll->step_curr < lll->step_count;
}

/* Fine frequency compensation of the initiator transmissions and receptions
 * from the mean frequency offset measured in the mode-0 steps.
 *
 * TODO: The direction of the compensation and the unit of the CSTONES
 * fractional frequency offset estimate are not verified on hardware.
 */
static int16_t fine_tune_compensation_get(const struct lll_cs *lll, uint8_t chan)
{
	int32_t ffo;

	if ((lll->role != LLL_CS_ROLE_INITIATOR) || (lll->ffo_count == 0U)) {
		return 0;
	}

	ffo = lll->ffo_sum / lll->ffo_count;

	/* FFO in 62.5 ppb units to 488.28125 Hz steps at 2402 + chan MHz */
	return (int16_t)((ffo * (2402 + (int32_t)chan) * 2) / 15625);
}

static void action_setup(struct lll_cs *lll)
{
	const struct lll_cs_step *step = &lll->steps[lll->step_curr];
	struct lll_cs_step_result *res = &lll->results[lll->step_curr];
	uint8_t action = lll->action;
	uint32_t t_start_us;
	uint32_t t_air_us;
	uint32_t len_us;
	int16_t fine_tune;
	uint8_t len;
	bool is_tone;
	bool is_tx;

	if (action == 0U) {
		res->iq.i = 0;
		res->iq.q = 0;
		res->time_diff = LLL_CS_TIME_DIFF_NA;
		res->freq_offset = LLL_CS_TIME_DIFF_NA;
		res->rssi = LLL_CS_RSSI_NA;
		res->aa_quality = LLL_CS_AA_NOT_FOUND;
		res->tone_quality = RADIO_CS_TONE_QUALITY_UNAVAILABLE;
	}

	is_tx = action_is_tx(lll);
	is_tone = (step->mode == LLL_CS_MODE_2);
	t_air_us = lll->t_step_us + lll_cs_action_offset_us(step->mode, action, lll->n_ap);
	len_us = lll_cs_action_len_us(step->mode, action, lll->n_ap);

	radio_freq_chan_set(2U + step->channel_index);
	radio_aa_set((action == 0U) ? step->access_address : step->access_address_r);

	if (step->mode != LLL_CS_MODE_0) {
		fine_tune = fine_tune_compensation_get(lll, step->channel_index);
	} else {
		fine_tune = 0;
	}

	(void)memset(tx_pkt, 0U, sizeof(tx_pkt));

	if (is_tone) {
		/* TODO: The tone is generated as an unmodulated carrier by
		 * transmitting an all-zero payload with the carrier offset by
		 * +250 kHz, its preamble and access address are sent before the
		 * tone start. Not verified to meet the Channel Sounding tone
		 * requirements.
		 */
		len = DIV_ROUND_UP(len_us, CS_OCTET_US);
		t_start_us = t_air_us - CS_PREAMBLE_AA_US;
	} else {
		len = DIV_ROUND_UP(len_us - CS_PREAMBLE_AA_US, CS_OCTET_US);
		t_start_us = t_air_us;

		/* CS_SYNC trailer, alternating bits continuing the access
		 * address.
		 */
		tx_pkt[0] = ((step->access_address[3] & BIT(7)) != 0U) ? 0xAAU : 0x55U;
		if (action != 0U) {
			tx_pkt[0] = ((step->access_address_r[3] & BIT(7)) != 0U) ? 0xAAU : 0x55U;
		}
	}

	len = MIN(len, CS_PKT_LEN_MAX);
	radio_cs_pkt_configure(PHY_1M, len);
	radio_switch_complete_and_disable();
	radio_cs_tones_status_reset();

	if (is_tx) {
		if (is_tone) {
			fine_tune += RADIO_CS_FINE_TUNE_250KHZ;
		} else if ((step->mode == LLL_CS_MODE_0) && (action != 0U)) {
			/* TODO: The mode-0 tone is generated by changing the
			 * carrier offset during the zero payload that follows
			 * the CS_SYNC. Timer ISR latency is not characterized.
			 */
			fine_tune_tone = fine_tune + RADIO_CS_FINE_TUNE_250KHZ;
			(void)radio_tmr_isr_set(t_air_us + LLL_CS_T_SY_1M_US + LLL_CS_T_GD_US,
						isr_tmr_fine_tune, lll);
		}

		radio_cs_fine_tune_set(fine_tune);
		radio_pkt_tx_set(tx_pkt);
		radio_tmr_start_us(1U, t_start_us - radio_tx_ready_delay_get(PHY_1M, 0U) -
					   radio_tx_chain_delay_get(PHY_1M, 0U));
	} else {
		uint32_t window_us;
		uint32_t hcto;

		window_us = (lll->synced != 0U) ? CS_RX_MARGIN_US : lll->window_us;

		radio_cs_fine_tune_set(fine_tune);
		radio_pkt_rx_set(radio_pkt_scratch_get());

		if (is_tone) {
			/* TODO: The tone measurement is started from the radio
			 * timer ISR without an access address match. Not
			 * verified on hardware.
			 */
			radio_cs_tones_configure(2U + step->channel_index, PHY_1M,
						 (LLL_CS_T_PM_US - (2U * CS_TONE_GUARD_US)) *
						 CS_TONE_SAMPLES_PER_US, false);
			(void)radio_tmr_isr_set(t_air_us + LLL_CS_T_SW_US + CS_TONE_GUARD_US,
						isr_tmr_tones_start, lll);
			hcto = t_air_us + len_us;
		} else {
			if ((step->mode == LLL_CS_MODE_0) && (action != 0U)) {
				radio_cs_tones_configure(2U + step->channel_index, PHY_1M,
							 (LLL_CS_T_FM_US -
							  (2U * CS_TONE_GUARD_US)) *
							 CS_TONE_SAMPLES_PER_US, true);
				(void)radio_tmr_isr_set(t_air_us + LLL_CS_T_SY_1M_US +
							LLL_CS_T_GD_US + CS_TONE_GUARD_US,
							isr_tmr_tones_start, lll);
			}

			radio_rssi_measure();
			hcto = t_air_us + window_us + addr_us_get(PHY_1M) +
			       radio_rx_chain_delay_get(PHY_1M, 0U);
		}

		radio_tmr_start_us(0U, t_air_us - window_us - radio_rx_ready_delay_get(PHY_1M, 0U));
		radio_tmr_aa_capture();
		radio_tmr_hcto_configure(hcto);
	}

	lll->is_rx = is_tx ? 0U : 1U;

	radio_isr_set(isr_action, lll);
}

static int16_t time_diff_get(int32_t diff_us)
{
	/* Microseconds to 0.5 ns units */
	diff_us = CLAMP(diff_us, -16, 16);

	return (int16_t)(diff_us * 2000);
}

static void isr_action_rx(struct lll_cs *lll)
{
	const struct lll_cs_step *step = &lll->steps[lll->step_curr];
	struct lll_cs_step_result *res = &lll->results[lll->step_curr];
	uint32_t t_rx_us;
	int32_t diff_us;

	if (step->mode == LLL_CS_MODE_2) {
		/* The tone measurement completes after the last sample */
		(void)radio_cs_tones_wait();
		res->tone_quality = radio_cs_tones_pct_get(&res->iq.i, &res->iq.q);

		return;
	}

	if (radio_is_address() == 0U) {
		return;
	}

	t_rx_us = radio_tmr_aa_get() - addr_us_get(PHY_1M) - radio_rx_chain_delay_get(PHY_1M, 0U);

	res->aa_quality = LLL_CS_AA_OK;
	if (radio_rssi_is_ready() != 0U) {
		res->rssi = -(int8_t)radio_rssi_get();
	}

	if (lll->role == LLL_CS_ROLE_REFLECTOR) {
		/* Synchronize the step timing to the initiator CS_SYNC.
		 *
		 * TODO: The 1 us radio timer resolution limits the time of
		 * flight accuracy; the reflector transmits at the nominal
		 * interlude after the measured time of arrival and reports a
		 * zero turnaround time difference.
		 */
		lll->t_step_us = t_rx_us;
		lll->synced = 1U;

		if (step->mode == LLL_CS_MODE_1) {
			res->time_diff = 0;
		}

		return;
	}

	diff_us = (int32_t)(t_rx_us - lll->t_step_us -
			    lll_cs_action_offset_us(step->mode, 1U, lll->n_ap));

	if (step->mode == LLL_CS_MODE_1) {
		res->time_diff = time_diff_get(diff_us);
	} else if (radio_cs_tones_wait()) {
		int16_t ffo = radio_cs_tones_ffo_get();

		/* 62.5 ppb units to 0.01 ppm units */
		res->freq_offset = (int16_t)((ffo * 25) / 4);
		lll->ffo_sum += ffo;
		lll->ffo_count++;
	}
}

static void isr_action(void *param)
{
	struct lll_cs *lll = param;

	if (lll->is_rx != 0U) {
		isr_action_rx(lll);
		lll_isr_rx_status_reset();
	} else {
		lll_isr_tx_status_reset();
	}

	radio_cs_tones_status_reset();
	radio_rssi_status_reset();
	(void)radio_tmr_isr_clear(NULL);

	do {
		if (!action_next(lll)) {
			subevent_end(lll);

			return;
		}
	} while (action_skip(lll));

	action_setup(lll);
}

static void isr_abort(void *param)
{
	struct lll_cs *lll = param;

	(void)radio_tmr_isr_clear(NULL);

	lll->abort_reason = CS_ABORT_CONFLICT;

	subevent_end(lll);
}

static void isr_tmr_tones_start(void *param, uint8_t chain)
{
	ARG_UNUSED(param);
	ARG_UNUSED(chain);

	radio_cs_tones_start();
}

static void isr_tmr_fine_tune(void *param, uint8_t chain)
{
	ARG_UNUSED(param);
	ARG_UNUSED(chain);

	radio_cs_fine_tune_set(fine_tune_tone);
}

static void subevent_end(struct lll_cs *lll)
{
	uint8_t abort_reason;

	radio_cs_reset();

	abort_reason = lll->abort_reason;
	if ((abort_reason == 0U) && (lll->synced == 0U)) {
		abort_reason = CS_ABORT_NO_SYNC;
	}

	if (ull_cs_subevent_report(lll, lll->step_curr, abort_reason) != 0U) {
		lll->rx_pending = 1U;
	}

	isr_done(lll);
}

static void isr_done(void *param)
{
	struct lll_cs *lll = param;

	lll_isr_status_reset();

	if (lll->rx_pending) {
		lll->rx_pending = 0U;
		ull_rx_sched();
	}

	ull_cs_lll_done(lll);

	(void)ull_done_extra_type_set(EVENT_DONE_EXTRA_TYPE_NONE);

	lll_isr_cleanup(param);
}

static int init_reset(void)
{
	return 0;
}
