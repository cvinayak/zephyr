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

/* Maximum CS_SYNC PDU length used to configure the radio packet for a step. The
 * CS_SYNC exchange itself is driven by the radio Channel Sounding hardware; the
 * standard packet path is only used to anchor the radio on/off timing of the
 * step, so a small bounded length is sufficient.
 */
#define LLL_CS_SYNC_MAX_LEN 16U

static int init_reset(void);
static int prepare_cb(struct lll_prepare_param *p);
static int is_abort_cb(void *next, void *curr, lll_prepare_cb_t *resume_cb);
static void abort_cb(struct lll_prepare_param *prepare_param, void *param);
static void isr_step(void *param);
static void isr_done(void *param);
static void cs_subevent_plan(struct lll_cs *lll);
static void cs_step_capture(struct lll_cs *lll);
static void cs_step_radio_setup(struct lll_cs *lll,
				const struct lll_cs_step *step);

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

	/* Bring up the high frequency clock for the radio event and register
	 * the CS prepare/abort callbacks with the LLL preemption pipeline,
	 * mirroring the ISO LLL prepare entry points.
	 */
	err = lll_hfclock_on();
	LL_ASSERT_ERR(err >= 0);

	err = lll_prepare(is_abort_cb, abort_cb, prepare_cb, 0U, param);
	LL_ASSERT_ERR((err == 0) || (err == -EINPROGRESS));
}

/* Plan the channel list and the ordered step list of the current subevent
 * (lll->subevent_curr) into the LLL context. Re-using the ULL planning helpers
 * keeps the channel selection and step ordering identical to the result
 * notification path. The helpers are pure (no allocation) and therefore safe to
 * call from the radio ISR when advancing to the next subevent.
 */
static void cs_subevent_plan(struct lll_cs *lll)
{
	uint32_t shuffle_seed;

	lll->channel_count = ull_cs_filtered_channels_get(lll->channel_map,
							  lll->channels,
							  sizeof(lll->channels));
	if (lll->channel_count == 0U) {
		lll->step_count = 0U;
		return;
	}

	/* Re-derive a unique channel order for this procedure and subevent. */
	shuffle_seed = ((uint32_t)lll->procedure_counter *
			(uint32_t)lll->subevents_per_event) +
		       (uint32_t)lll->subevent_curr + 1U;
	ull_cs_channels_shuffle(lll->channels, lll->channel_count, shuffle_seed);

	lll->step_count = ull_cs_subevent_steps_plan(lll->main_mode,
						     lll->sub_mode,
						     lll->main_mode_steps,
						     lll->main_mode_repetition,
						     lll->mode_0_steps,
						     lll->rtt_type,
						     lll->access_address,
						     lll->channels,
						     lll->channel_count,
						     lll->steps,
						     LLL_CS_MAX_STEPS);
}

/* Program the radio for a single Channel Sounding step. The radio Channel
 * Sounding hardware block is configured from the step descriptor and the
 * standard packet path is set up to transmit (Initiator) or receive
 * (Reflector) the CS_SYNC first, with an automatic in-phase period turnaround
 * to the reflected CS_SYNC.
 */
static void cs_step_radio_setup(struct lll_cs *lll,
				const struct lll_cs_step *step)
{
	uint8_t pkt_flags;
	void *pdu;

	/* Configure the radio Channel Sounding hardware (mode, channel, access
	 * address, RTT and tone settings) for the step.
	 */
	lll_cs_step_setup(step);

	/* For tone (phase based ranging) steps, point the radio IQ/phase (PCT)
	 * sample capture at the per-event scratch buffer so the captured tone
	 * sample can be read back after the step completes.
	 */
	if ((step->mode == LLL_CS_MODE_2) || (step->mode == LLL_CS_MODE_3)) {
		radio_cs_pct_buffer_set(lll->pct_scratch,
					sizeof(lll->pct_scratch));
	}

	radio_aa_set(step->access_address);

	pkt_flags = RADIO_PKT_CONF_FLAGS(RADIO_PKT_CONF_PDU_TYPE_DC, lll->phy,
					 RADIO_PKT_CONF_CTE_DISABLED);
	radio_pkt_configure(RADIO_PKT_CONF_LENGTH_8BIT, LLL_CS_SYNC_MAX_LEN,
			    pkt_flags);

	pdu = radio_pkt_scratch_get();

	if (lll->role == LLL_CS_ROLE_INITIATOR) {
		/* Initiator transmits the CS_SYNC and then receives the
		 * reflected CS_SYNC after the in-phase period turnaround.
		 */
		radio_pkt_tx_set(pdu);
		radio_switch_complete_and_rx(lll->phy);
	} else {
		/* Reflector receives the CS_SYNC and then transmits the
		 * reflected CS_SYNC after the in-phase period turnaround.
		 */
		radio_pkt_rx_set(pdu);
		radio_switch_complete_and_tx(lll->phy, PHY_FLAGS_S8, lll->phy,
					     PHY_FLAGS_S8);
	}
}

static int prepare_cb(struct lll_prepare_param *p)
{
	struct lll_cs *lll;
	struct ull_hdr *ull;
	uint32_t ticks_at_event;
	uint32_t ticks_at_start;
	uint32_t remainder;
	uint32_t start_us;
	uint8_t trx;

	DEBUG_RADIO_START_O(1);

	lll = p->param;

	/* Begin the CS event at its first subevent, first step. */
	lll->subevent_curr = 0U;
	lll->step_curr = 0U;
	lll->rx_pending = 0U;
	cs_subevent_plan(lll);
	if (lll->step_count == 0U) {
		/* Nothing to schedule for this event, complete immediately. */
		radio_isr_set(isr_done, lll);
		radio_disable();

		DEBUG_RADIO_START_O(1);
		return 0;
	}

	/* Set up the radio for the first step of the first subevent. */
	radio_reset();
	radio_phy_set(lll->phy, PHY_FLAGS_S8);
	radio_tmr_tifs_set(EVENT_IFS_US);

	cs_step_radio_setup(lll, &lll->steps[0]);

	radio_isr_set(isr_step, lll);

	/* Anchor the radio event at the ticker expiry plus the event start
	 * overhead, then start the radio. The Initiator transmits first and
	 * the Reflector receives first.
	 */
	ull = HDR_LLL2ULL(lll);
	ticks_at_event = p->ticks_at_expire;
	ticks_at_event += lll_event_offset_get(ull);

	ticks_at_start = ticks_at_event;
	ticks_at_start += HAL_TICKER_US_TO_TICKS(EVENT_OVERHEAD_START_US);

	remainder = p->remainder;

	trx = (lll->role == LLL_CS_ROLE_INITIATOR) ? 1U : 0U;
	start_us = radio_tmr_start(trx, ticks_at_start, remainder);

	/* Save the radio ready time so the subsequent steps and subevents are
	 * scheduled relative to the event anchor point.
	 */
	radio_tmr_ready_save(start_us);
	radio_tmr_end_capture();

	{
		int err;

		err = lll_prepare_done(lll);
		LL_ASSERT_ERR(!err);
	}

	DEBUG_RADIO_START_O(1);
	return 0;
}

static int is_abort_cb(void *next, void *curr, lll_prepare_cb_t *resume_cb)
{
	ARG_UNUSED(resume_cb);

	/* Let a different scheduled event preempt the CS event; continue when
	 * the same CS event is being evaluated.
	 */
	if (next != curr) {
		return -ECANCELED;
	}

	return 0;
}

static void abort_cb(struct lll_prepare_param *prepare_param, void *param)
{
	int err;

	/* If prepare_param is NULL the CS event is already ongoing and must be
	 * stopped; otherwise a queued prepare is being cancelled.
	 */
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

static void isr_step(void *param)
{
	struct lll_cs *lll;
	uint32_t next_us;
	uint8_t trx;

	/* Accumulate the elapsed time under single timer use and clear the
	 * radio status and events of the completed step.
	 */
	(void)radio_is_done();
	lll_isr_status_reset();

	lll = param;
	trx = (lll->role == LLL_CS_ROLE_INITIATOR) ? 1U : 0U;

	/* Capture the measurement results of the step that just completed. */
	cs_step_capture(lll);

	/* Advance to the next step of the current subevent. */
	lll->step_curr++;
	if (lll->step_curr < lll->step_count) {
		radio_tmr_tifs_set(EVENT_IFS_US);
		cs_step_radio_setup(lll, &lll->steps[lll->step_curr]);
		radio_isr_set(isr_step, lll);

		/* Schedule the step relative to the event anchor point using
		 * the negotiated subevent and step start-to-start spacing.
		 */
		next_us = radio_tmr_ready_restore();
		next_us += (uint32_t)lll->subevent_curr *
			   lll->subevent_interval_us;
		next_us += (uint32_t)lll->step_curr * lll->step_interval_us;
		(void)radio_tmr_start_us(trx, next_us);
		radio_tmr_end_capture();

		return;
	}

	/* Subevent complete, report the captured results towards the ULL and
	 * the Host. The last subevent of the event carries the procedure done
	 * complete status, the preceding ones a partial status.
	 */
	{
		uint8_t last_subevent =
			((lll->subevent_curr + 1U) >= lll->subevents_per_event);

		lll->rx_pending |= ull_cs_subevent_report(
			lll,
			BT_HCI_LE_CS_SUBEVENT_DONE_STATUS_COMPLETE,
			last_subevent ?
				BT_HCI_LE_CS_PROCEDURE_DONE_STATUS_COMPLETE :
				BT_HCI_LE_CS_PROCEDURE_DONE_STATUS_PARTIAL);
	}

	/* Advance to the next subevent of the event. */
	lll->subevent_curr++;
	if (lll->subevent_curr < lll->subevents_per_event) {
		lll->step_curr = 0U;
		cs_subevent_plan(lll);
		if (lll->step_count != 0U) {
			radio_tmr_tifs_set(EVENT_IFS_US);
			cs_step_radio_setup(lll, &lll->steps[0]);
			radio_isr_set(isr_step, lll);

			next_us = radio_tmr_ready_restore();
			next_us += (uint32_t)lll->subevent_curr *
				   lll->subevent_interval_us;
			(void)radio_tmr_start_us(trx, next_us);
			radio_tmr_end_capture();

			return;
		}
	}

	/* Event complete. */
	isr_done(lll);
}

/* Capture the radio measurement results of the Channel Sounding step that just
 * completed (lll->step_curr) into the LLL context. Mode-1 and mode-3 steps
 * carry the RTT timestamp; mode-2 and mode-3 steps carry the IQ/phase (PCT)
 * sample of the received tone.
 */
static void cs_step_capture(struct lll_cs *lll)
{
	const struct lll_cs_step *step = &lll->steps[lll->step_curr];

	if ((step->mode == LLL_CS_MODE_1) || (step->mode == LLL_CS_MODE_3)) {
		lll->step_rtt[lll->step_curr] = lll_cs_step_rtt_get();
	}

	if ((step->mode == LLL_CS_MODE_2) || (step->mode == LLL_CS_MODE_3)) {
		struct lll_cs_iq_sample sample = { 0 };

		(void)lll_cs_step_pct_get(&sample, 1U);
		lll->step_iq[lll->step_curr] = sample;
	}
}

static void isr_done(void *param)
{
	struct lll_cs *lll = param;

	lll_isr_status_reset();

	/* Schedule the ULL receive demux when at least one CS subevent result
	 * notification was enqueued during this event so that the Host gets the
	 * results. This is done once per event regardless of the number of
	 * subevents reported.
	 */
	if (lll->rx_pending) {
		lll->rx_pending = 0U;
		ull_rx_sched();
	}

	/* Mark the done event as not needing role specific processing; the CS
	 * subevent results are reported via the receive path above. This still
	 * releases the prepare reference taken in the ticker callback.
	 */
	(void)ull_done_extra_type_set(EVENT_DONE_EXTRA_TYPE_NONE);

	lll_isr_cleanup(param);
}

void lll_cs_step_setup(const struct lll_cs_step *step)
{
	/* Program the radio Channel Sounding hardware for the step. The order
	 * follows the Bluetooth Core specification, Vol 6, Part A, Section 5:
	 * the step mode determines which of the RTT and tone (phase based
	 * ranging) exchanges are present.
	 */
	radio_cs_mode_set(step->mode);
	radio_cs_chan_set(step->channel_index);
	radio_cs_aa_set(step->access_address);

	/* Mode 1 and mode 3 steps carry the RTT timestamp exchange. */
	if ((step->mode == LLL_CS_MODE_1) || (step->mode == LLL_CS_MODE_3)) {
		radio_cs_rtt_type_set(step->rtt_type);
	}

	/* Mode 2 and mode 3 steps carry the tone (phase based ranging)
	 * exchange driven over the configured antenna paths.
	 */
	if ((step->mode == LLL_CS_MODE_2) || (step->mode == LLL_CS_MODE_3)) {
		radio_cs_antenna_set(step->num_ant_paths,
				     step->antenna_selection);
		radio_cs_tone_set(step->num_tones, step->tone_duration_us);
	}
}

uint32_t lll_cs_step_rtt_get(void)
{
	return radio_cs_rtt_timestamp_get();
}

uint32_t lll_cs_step_pct_get(struct lll_cs_iq_sample *samples, uint32_t max)
{
	/* struct lll_cs_iq_sample and struct radio_cs_iq_sample share the same
	 * layout (two int16_t fields), so the radio captured samples can be
	 * copied directly into the caller provided array.
	 */
	return radio_cs_pct_get((struct radio_cs_iq_sample *)samples, max);
}

static int init_reset(void)
{
	if (radio_cs_is_supported()) {
		radio_cs_reset();
	}

	return 0;
}
