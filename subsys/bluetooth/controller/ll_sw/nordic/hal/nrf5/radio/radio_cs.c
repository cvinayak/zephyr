/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdbool.h>
#include <stdint.h>

#include <soc.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <hal/nrf_radio.h>

#include "hal/radio_cs.h"

#if !defined(RADIO_CSTONES_MODE_TPM_Msk)
#error "Channel Sounding requires a radio with the CSTONES tone measurement unit"
#endif

/* Mean tone magnitude below which the tone is considered not received.
 * TODO: Thresholds are not characterized on hardware.
 */
#define TONE_MAG_MIN 16U

/* Tone quality thresholds on the 12-bit magnitude standard deviation and the
 * absolute fractional frequency offset in ppb.
 */
#define TONE_MAGSTD_HIGH_MAX 18U
#define TONE_MAGSTD_MED_MAX  22U
#define TONE_FFO_HIGH_MAX    15000
#define TONE_FFO_MED_MAX     30000

/* Tone measurement sample count range, in samples at the 2 MHz sample rate. */
#define NUM_SAMPLES_MIN 16U
#define NUM_SAMPLES_MAX 160U

/* Maximum time to wait for the tone processing to complete once the radio
 * has stopped sampling.
 * TODO: Tone processing duration is not characterized on hardware.
 */
#define TONES_PROC_US_MAX 20U

void radio_cs_pkt_configure(uint8_t phy, uint8_t len)
{
	uint32_t plen;

	if (phy == BIT(1)) {
		plen = RADIO_PCNF0_PLEN_16bit;
	} else {
		plen = RADIO_PCNF0_PLEN_8bit;
	}

	NRF_RADIO->PCNF0 = (plen << RADIO_PCNF0_PLEN_Pos) & RADIO_PCNF0_PLEN_Msk;
	NRF_RADIO->PCNF1 =
		((((uint32_t)len) << RADIO_PCNF1_MAXLEN_Pos) & RADIO_PCNF1_MAXLEN_Msk) |
		((((uint32_t)len) << RADIO_PCNF1_STATLEN_Pos) & RADIO_PCNF1_STATLEN_Msk) |
		((3UL << RADIO_PCNF1_BALEN_Pos) & RADIO_PCNF1_BALEN_Msk) |
		((RADIO_PCNF1_ENDIAN_Little << RADIO_PCNF1_ENDIAN_Pos) & RADIO_PCNF1_ENDIAN_Msk);

	/* CS_SYNC packets carry no CRC */
	NRF_RADIO->CRCCNF = 0U;

	/* The RTT unit, needed by the tone measurement unit, disturbs the RSSI
	 * measurement; it is enabled by radio_cs_tones_configure() only.
	 */
	NRF_RADIO->RTT.CONFIG &= ~RADIO_RTT_CONFIG_EN_Msk;
}

void radio_cs_trx_enable_select(uint8_t trx)
{
	nrf_radio_subscribe_clear(NRF_RADIO, (trx != 0U) ? NRF_RADIO_TASK_RXEN :
							   NRF_RADIO_TASK_TXEN);
}

void radio_cs_fine_tune_set(int16_t steps)
{
	NRF_RADIO->FREQFINETUNE = ((uint32_t)(int32_t)steps) & RADIO_FREQFINETUNE_FREQFINETUNE_Msk;
}

void radio_cs_tx_carrier_hold(void)
{
	NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk;
}

void radio_cs_tx_carrier_only(void)
{
	NRF_RADIO->SHORTS = 0U;
}

void radio_cs_tx_carrier_end(void)
{
	NRF_RADIO->SHORTS = 0U;
	nrf_radio_task_trigger(NRF_RADIO, NRF_RADIO_TASK_DISABLE);
}

void radio_cs_tones_configure(uint8_t chan, uint8_t phy, uint8_t num_samples, bool ffo,
			      int8_t fae_peer)
{
	uint32_t rate;
	uint32_t mode;

	num_samples = CLAMP(num_samples, NUM_SAMPLES_MIN, NUM_SAMPLES_MAX);

	/* The tone measurement unit does not complete every measurement
	 * unless the RTT unit is enabled.
	 */
	NRF_RADIO->RTT.CONFIG |= RADIO_RTT_CONFIG_EN_Msk;

	/* The frequency offset measurement is also used for the tone quality */
	mode = RADIO_CSTONES_MODE_TFM_Msk;
	if (!ffo) {
		mode |= RADIO_CSTONES_MODE_TPM_Msk;
	}

	NRF_RADIO->CSTONES.MODE = mode;
	NRF_RADIO->CSTONES.NUMSAMPLES = num_samples;
	/* Averaging coefficient 2^16 / (num_samples / 16), rounded to nearest.
	 * Gives the reset value 0x199A for the reset value of 160 samples.
	 */
	NRF_RADIO->CSTONES.NUMSAMPLESCOEFF =
		((1UL << 20) + (num_samples / 2U)) / num_samples;
	NRF_RADIO->CSTONES.NEXTFREQUENCY = chan;
	NRF_RADIO->CSTONES.FAEPEER = (uint32_t)(int32_t)fae_peer &
				      RADIO_CSTONES_FAEPEER_FAEPEER_Msk;
	NRF_RADIO->CSTONES.PHASESHIFT = 0U;

	/* The input IQ samples are filtered and down sampled from the sample
	 * rate of the radio mode the samples are taken in.
	 */
	if (phy == BIT(1)) {
		rate = RADIO_CSTONES_DOWNSAMPLE_RATE_BLE2M;
	} else {
		rate = RADIO_CSTONES_DOWNSAMPLE_RATE_BLE1M;
	}

	NRF_RADIO->CSTONES.DOWNSAMPLE =
		((RADIO_CSTONES_DOWNSAMPLE_ENABLEFILTER_ON <<
		  RADIO_CSTONES_DOWNSAMPLE_ENABLEFILTER_Pos) &
		 RADIO_CSTONES_DOWNSAMPLE_ENABLEFILTER_Msk) |
		((rate << RADIO_CSTONES_DOWNSAMPLE_RATE_Pos) & RADIO_CSTONES_DOWNSAMPLE_RATE_Msk);
}

void radio_cs_tones_start(void)
{
	NRF_RADIO->TASKS_CSTONESSTART = 1U;
}

bool radio_cs_tones_is_done(void)
{
	return NRF_RADIO->EVENTS_CSTONESEND != 0U;
}

bool radio_cs_tones_wait(void)
{
	for (uint32_t us = 0U; us < TONES_PROC_US_MAX; us++) {
		if (radio_cs_tones_is_done()) {
			return true;
		}

		k_busy_wait(1U);
	}

	return radio_cs_tones_is_done();
}

void radio_cs_tones_status_reset(void)
{
	NRF_RADIO->EVENTS_CSTONESEND = 0U;
}

uint8_t radio_cs_tones_pct_get(int16_t *i, int16_t *q)
{
	uint32_t magphase;
	int32_t ffo_ppb;
	uint32_t pct16;
	uint16_t magstd;
	uint16_t mag;

	if (!radio_cs_tones_is_done()) {
		*i = 0;
		*q = 0;

		return RADIO_CS_TONE_QUALITY_UNAVAILABLE;
	}

	pct16 = NRF_RADIO->CSTONES.PCT16;
	magphase = NRF_RADIO->CSTONES.MAGPHASEMEAN;
	/* Standard deviation of the 16-bit magnitude, in 12-bit units */
	magstd = (uint16_t)(NRF_RADIO->CSTONES.MAGSTD & RADIO_CSTONES_MAGSTD_MAGSTD_Msk) >> 4;

	/* Reduce the 16-bit PCT to the 12-bit format of the HCI event */
	*i = (int16_t)(pct16 & 0xFFFFU) >> 4;
	*q = (int16_t)(pct16 >> 16) >> 4;

	mag = (uint16_t)((magphase & RADIO_CSTONES_MAGPHASEMEAN_MAG_Msk) >>
			 RADIO_CSTONES_MAGPHASEMEAN_MAG_Pos);

	/* 62.5 ppb units to ppb */
	ffo_ppb = ((int32_t)radio_cs_tones_ffo_get() * 125) / 2;
	ffo_ppb = (ffo_ppb < 0) ? -ffo_ppb : ffo_ppb;

	if (mag < TONE_MAG_MIN) {
		return RADIO_CS_TONE_QUALITY_UNAVAILABLE;
	} else if ((magstd <= TONE_MAGSTD_HIGH_MAX) && (ffo_ppb <= TONE_FFO_HIGH_MAX)) {
		return RADIO_CS_TONE_QUALITY_HIGH;
	} else if ((magstd <= TONE_MAGSTD_MED_MAX) && (ffo_ppb <= TONE_FFO_MED_MAX)) {
		return RADIO_CS_TONE_QUALITY_MEDIUM;
	}

	return RADIO_CS_TONE_QUALITY_LOW;
}

int16_t radio_cs_tones_ffo_get(void)
{
	uint32_t ffo;

	ffo = NRF_RADIO->CSTONES.FFOEST & RADIO_CSTONES_FFOEST_FFOEST_Msk;

	/* Sign extend the 12-bit two's complement value */
	return (int16_t)(ffo << 4) >> 4;
}

void radio_cs_tones_mag_get(uint16_t *mag, uint16_t *magstd)
{
	*mag = (uint16_t)((NRF_RADIO->CSTONES.MAGPHASEMEAN & RADIO_CSTONES_MAGPHASEMEAN_MAG_Msk) >>
			  RADIO_CSTONES_MAGPHASEMEAN_MAG_Pos);
	*magstd = (uint16_t)(NRF_RADIO->CSTONES.MAGSTD & RADIO_CSTONES_MAGSTD_MAGSTD_Msk) >> 4;
}

void radio_cs_reset(void)
{
	NRF_RADIO->FREQFINETUNE = 0U;
	NRF_RADIO->CSTONES.MODE = RADIO_CSTONES_MODE_ResetValue;
	NRF_RADIO->CSTONES.NUMSAMPLES = RADIO_CSTONES_NUMSAMPLES_ResetValue;
	NRF_RADIO->CSTONES.NUMSAMPLESCOEFF = RADIO_CSTONES_NUMSAMPLESCOEFF_ResetValue;
	NRF_RADIO->CSTONES.DOWNSAMPLE = RADIO_CSTONES_DOWNSAMPLE_ResetValue;
	NRF_RADIO->RTT.CONFIG &= ~RADIO_RTT_CONFIG_EN_Msk;
	NRF_RADIO->EVENTS_CSTONESEND = 0U;
}
