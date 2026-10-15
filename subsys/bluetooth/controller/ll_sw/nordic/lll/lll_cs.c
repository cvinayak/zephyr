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
#include <zephyr/kernel.h>
#include <zephyr/toolchain.h>
#include <zephyr/sys/byteorder.h>
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

/* Preamble and access address air time for CS_SYNC. */
#define CS_PREAMBLE_AA_1M_US 40U
#if defined(CONFIG_BT_CTLR_CS_SYNC_2M)
#define CS_PREAMBLE_AA_2M_US 24U
#endif

/* RX window after reflector has synchronized to initiator CS_SYNC. */
#define CS_RX_MARGIN_US 16U

/* Initiator RX margin for reflector responses (derived from reflector ToA). */
#define CS_INITIATOR_RX_MARGIN_US 24U

/* Air time of one octet on LE 1M. */
#define CS_OCTET_US 8U

/* Guard time at the start and end of a tone excluded from the measurement. */
#define CS_TONE_GUARD_US 1U

/* Delay from a tone on air to its IQ samples at the tone measurement unit. */
#define CS_TONE_IQ_DELAY_US 2U

/* Tone measurement sample rate, samples per microsecond. */
#define CS_TONE_SAMPLES_PER_US 2U

/* Margin after the planned subevent end before an own ACL connection event
 * may preempt the subevent.
 */
#define CS_OVERRUN_MARGIN_US 500U

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
static void cs_tones_start(struct lll_cs *lll, uint32_t start_us);
static void isr_tmr_carrier_end(void *param, uint8_t chain);
static void subevent_end(struct lll_cs *lll);
static void isr_done(void *param);

static uint8_t MALIGN(4) tx_pkt[CS_PKT_LEN_MAX];

static inline uint32_t cs_preamble_aa_us(uint8_t phy)
{
#if defined(CONFIG_BT_CTLR_CS_SYNC_2M)
	if (phy == PHY_2M) {
		return CS_PREAMBLE_AA_2M_US;
	}
#else
	ARG_UNUSED(phy);
#endif

	return CS_PREAMBLE_AA_1M_US;
}

static inline uint32_t cs_rx_window_us(const struct lll_cs *lll)
{
	if (lll->role == LLL_CS_ROLE_REFLECTOR && lll->synced == 0U) {
		/* Symmetric window around the expected initiator CS_SYNC */
		return lll->window_us;
	}

	if (lll->role == LLL_CS_ROLE_INITIATOR) {
		return CS_INITIATOR_RX_MARGIN_US;
	}

	return CS_RX_MARGIN_US;
}

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
	radio_phy_set(lll->phy, 0U);

	ticks_at_event = p->ticks_at_expire;
	ull = HDR_LLL2ULL(lll);
	ticks_at_event += lll_event_offset_get(ull);
	ticks_at_start = ticks_at_event + HAL_TICKER_US_TO_TICKS(EVENT_OVERHEAD_START_US);

	/* The first action of a subevent is the initiator CS_SYNC of step 0 */
	is_tx = (lll->role == LLL_CS_ROLE_INITIATOR);

	/* The CS ticker is a single shot ticker without a remainder started
	 * from the ACL ticker expiry, use the remainder of the ACL event.
	 */
	radio_cs_trx_enable_select(is_tx ? 1U : 0U);
	remainder_us = radio_tmr_start(is_tx ? 1U : 0U, ticks_at_start, lll->remainder);

	/* Air time of the ACL anchor point relative to the ACL radio timer
	 * start: the central enables Tx at the timer start, the peripheral
	 * enables Rx early by the jitter, ticker resolution margin and window
	 * widening of its receive window centered on the estimated anchor.
	 * The peripheral drift compensation centers the address event, which
	 * lags the air time by the Rx chain delay.
	 */
	if (lll->acl_periph != 0U) {
		anchor_us = radio_rx_ready_delay_get(lll->acl_phy, PHY_FLAGS_S8) +
			    EVENT_JITTER_US + EVENT_TICKER_RES_MARGIN_US + lll->acl_ww_us -
			    radio_rx_chain_delay_get(lll->acl_phy, PHY_FLAGS_S8);
	} else {
		anchor_us = radio_tx_ready_delay_get(lll->acl_phy, PHY_FLAGS_S8) +
			    radio_tx_chain_delay_get(lll->acl_phy, PHY_FLAGS_S8);
	}

	/* The subevent starts at the ACL anchor point plus the CS offset, and
	 * every step starts with T_FCS before the initiator CS_SYNC. t_step_us
	 * is the air time of the initiator CS_SYNC of the current step.
	 */
	lll->t_step_us = remainder_us + anchor_us + lll->start_offset_us + LLL_CS_T_FCS_US;
	lll->t_end_us = lll->t_step_us - LLL_CS_T_FCS_US + lll->duration_us +
			(lll->window_us << 1) + CS_OVERRUN_MARGIN_US;

	action_setup(lll);

	err = lll_prepare_done(lll);
	LL_ASSERT_ERR(!err);

	DEBUG_RADIO_START_O(1);
	return 0;
}

static int is_abort_cb(void *next, void *curr, lll_prepare_cb_t *resume_cb)
{
	struct lll_cs *lll = curr;

	ARG_UNUSED(resume_cb);

	if (next == curr) {
		return 0;
	}

	/* A CS subevent spans the ACL connection events that overlap it, these
	 * ACL connection events are cancelled, unless the subevent overran its
	 * planned end.
	 */
	if ((next != NULL) && (next == lll->acl_lll)) {
		uint32_t now_us;

		radio_tmr_sample();
		now_us = radio_tmr_sample_get();
		if ((int32_t)(now_us - lll->t_end_us) < 0) {
			return -EBUSY;
		}
	}

	return -ECANCELED;
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
	lll->t_step_us += lll_cs_step_len_us(step->mode, lll->n_ap, lll->t_sw_us, lll->phy) +
			  LLL_CS_T_FCS_US;
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
	uint32_t start_us __maybe_unused;
	uint32_t t_start_us;
	uint32_t t_air_us;
	uint32_t req_us;
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
	t_air_us = lll->t_step_us + lll_cs_action_offset_us(step->mode, action, lll->n_ap,
							    lll->t_sw_us, lll->phy);
	len_us = lll_cs_action_len_us(step->mode, action, lll->n_ap, lll->t_sw_us, lll->phy);

	radio_freq_chan_set(2U + step->channel_index);
	radio_aa_set((action == 0U) ? step->access_address : step->access_address_r);
	radio_phy_set(lll->phy, 0U);

	if (step->mode != LLL_CS_MODE_0) {
		fine_tune = fine_tune_compensation_get(lll, step->channel_index);
	} else {
		fine_tune = 0;
	}

	(void)memset(tx_pkt, 0U, sizeof(tx_pkt));

	if (is_tone) {
		/* Tx tones are the unmodulated carrier of the radio in TXIDLE,
		 * no packet is transmitted.
		 */
		len = DIV_ROUND_UP(len_us, CS_OCTET_US);
		t_start_us = t_air_us;
	} else {
		if (is_tx && (step->mode == LLL_CS_MODE_0) && (action != 0U)) {
			/* Only the CS_SYNC is a packet, the frequency
			 * measurement tone that follows is the unmodulated
			 * carrier of the radio in TXIDLE. A FREQFINETUNE change
			 * during the packet does not take effect.
			 */
			len = DIV_ROUND_UP(lll_cs_t_sy_us(lll->phy) -
					   cs_preamble_aa_us(lll->phy), CS_OCTET_US);
		} else {
			len = DIV_ROUND_UP(len_us - cs_preamble_aa_us(lll->phy), CS_OCTET_US);
		}
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
	radio_cs_pkt_configure(lll->phy, len);
	radio_switch_complete_and_disable();
	radio_tmr_cs_tones_start_clear();
	radio_cs_tones_status_reset();

	if (is_tx) {
		/* The carrier is ended by the timer ISR, its latency only
		 * extends the carrier past the measurement.
		 */
		if (is_tone) {
			radio_cs_tx_carrier_only();
			(void)radio_tmr_isr_set(t_air_us + len_us, isr_tmr_carrier_end, lll);
		} else if ((step->mode == LLL_CS_MODE_0) && (action != 0U)) {
			radio_cs_tx_carrier_hold();
			(void)radio_tmr_isr_set(t_air_us + len_us, isr_tmr_carrier_end, lll);
		}

		radio_cs_fine_tune_set(fine_tune);
		radio_pkt_tx_set(tx_pkt);
#if defined(CONFIG_BT_CTLR_PROFILE_ISR) || defined(CONFIG_BT_CTLR_SW_SWITCH_SINGLE_TIMER)
		radio_tmr_end_capture();
#endif
		req_us = t_start_us - radio_tx_ready_delay_get(lll->phy, 0U) -
			 radio_tx_chain_delay_get(lll->phy, 0U);
		radio_cs_trx_enable_select(1U);
		start_us = radio_tmr_start_us(1U, req_us);
	} else {
		uint32_t window_us;
		uint32_t hcto;

		window_us = cs_rx_window_us(lll);

		radio_cs_fine_tune_set(fine_tune);
		radio_pkt_rx_set(radio_pkt_scratch_get());

		if (is_tone) {
			/* TODO: The tone measurement is started at the expected
			 * tone time without an access address match. Not
			 * verified on hardware.
			 */
			radio_cs_tones_configure(2U + step->channel_index, lll->phy,
						 (LLL_CS_T_PM_US - (2U * CS_TONE_GUARD_US)) *
						 CS_TONE_SAMPLES_PER_US, false,
						 ull_cs_fae_for_channel(lll->fae_table,
									step->channel_index));
			cs_tones_start(lll, t_air_us + lll->t_sw_us + CS_TONE_GUARD_US +
					    CS_TONE_IQ_DELAY_US);
			hcto = t_air_us + len_us;
		} else {
			if ((step->mode == LLL_CS_MODE_0) && (action != 0U)) {
				radio_cs_tones_configure(2U + step->channel_index, lll->phy,
							 (LLL_CS_T_FM_US -
							  (2U * CS_TONE_GUARD_US)) *
							 CS_TONE_SAMPLES_PER_US, true,
							 ull_cs_fae_for_channel(lll->fae_table,
										step->channel_index));
				cs_tones_start(lll, t_air_us + lll_cs_t_sy_us(lll->phy) +
						    LLL_CS_T_GD_US + CS_TONE_GUARD_US +
						    CS_TONE_IQ_DELAY_US);
			}

			radio_rssi_measure();
			hcto = t_air_us + window_us + addr_us_get(lll->phy) +
			       radio_rx_chain_delay_get(lll->phy, 0U);
		}

		req_us = t_air_us - window_us - radio_rx_ready_delay_get(lll->phy, 0U);
		radio_cs_trx_enable_select(0U);
		start_us = radio_tmr_start_us(0U, req_us);

		/* A late Rx enable must still be followed by the HCTO, else the
		 * radio is never disabled and the subevent does not end.
		 */
		hcto = MAX(hcto, start_us + radio_rx_ready_delay_get(lll->phy, 0U) +
				 addr_us_get(lll->phy) + CS_RX_MARGIN_US);

		radio_tmr_aa_capture();
		radio_tmr_hcto_configure_abs(hcto);
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
	uint32_t t_air_us __maybe_unused;
	uint32_t is_done __maybe_unused;
	uint32_t t_rx_us;
	int32_t diff_us;

	if (step->mode == LLL_CS_MODE_2) {
		/* The tone measurement completes after the last sample */
		(void)radio_cs_tones_wait();
		res->tone_quality = radio_cs_tones_pct_get(&res->iq.i, &res->iq.q);

		return;
	}

	is_done = radio_is_done();

	if (radio_is_address() == 0U) {
		return;
	}

	t_rx_us = radio_tmr_aa_get() - addr_us_get(lll->phy) -
		  radio_rx_chain_delay_get(lll->phy, 0U);
	t_air_us = lll->t_step_us +
		   lll_cs_action_offset_us(step->mode, lll->action, lll->n_ap, lll->t_sw_us,
					   lll->phy);

	res->aa_quality = LLL_CS_AA_OK;
	if (radio_rssi_is_ready() != 0U) {
		res->rssi = -(int8_t)radio_rssi_get();
	}

	if (lll->role == LLL_CS_ROLE_REFLECTOR && lll->action == 0U &&
	    step->mode != LLL_CS_MODE_2) {
		/* Anchor step timing to the initiator CS_SYNC start (action 0).
		 * Reflector responses in action 1 are scheduled from this point.
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
			    lll_cs_action_offset_us(step->mode, 1U, lll->n_ap, lll->t_sw_us,
						    lll->phy));

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

static void cs_tones_start(struct lll_cs *lll, uint32_t start_us)
{
	ARG_UNUSED(lll);

	(void)radio_tmr_cs_tones_start_us(start_us);
}

static void isr_tmr_carrier_end(void *param, uint8_t chain)
{
	ARG_UNUSED(param);
	ARG_UNUSED(chain);

	radio_cs_tx_carrier_end();
}

static void subevent_end(struct lll_cs *lll)
{
	uint8_t abort_reason;

	radio_tmr_cs_tones_start_clear();
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
