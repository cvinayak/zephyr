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

#include "hal/radio_cs.h"

#if !defined(RADIO_CSTONES_MODE_TPM_Msk)
#error "Channel Sounding requires a radio with the CSTONES tone measurement unit"
#endif

/* Mean tone magnitude below which the tone is considered not received.
 * TODO: Thresholds are not characterized on hardware.
 */
#define TONE_MAG_MIN 16U

/* Tone measurement sample count range, in samples at the 2 MHz sample rate. */
#define NUM_SAMPLES_MIN 16U
#define NUM_SAMPLES_MAX 160U

/* Number of samples in one 8 us period at the 2 MHz sample rate. */
#define NUM_SAMPLES_PER_8US 16U

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
}

void radio_cs_fine_tune_set(int16_t steps)
{
	NRF_RADIO->FREQFINETUNE = ((uint32_t)(int32_t)steps) & RADIO_FREQFINETUNE_FREQFINETUNE_Msk;
}

void radio_cs_tones_configure(uint8_t chan, uint8_t phy, uint8_t num_samples, bool ffo)
{
	uint32_t rate;
	uint32_t mode;

	/* Sample count in whole 8 us periods */
	num_samples = CLAMP(num_samples, NUM_SAMPLES_MIN, NUM_SAMPLES_MAX);
	num_samples -= num_samples % NUM_SAMPLES_PER_8US;

	mode = RADIO_CSTONES_MODE_TPM_Msk;
	if (ffo) {
		mode |= RADIO_CSTONES_MODE_TFM_Msk;
	}

	NRF_RADIO->CSTONES.MODE = mode;
	NRF_RADIO->CSTONES.NUMSAMPLES = num_samples;
	/* Averaging coefficient 2^16 / (num_samples / 16), rounded to nearest.
	 * Gives the reset value 0x199A for the reset value of 160 samples.
	 */
	NRF_RADIO->CSTONES.NUMSAMPLESCOEFF =
		((1UL << 20) + (num_samples / 2U)) / num_samples;
	NRF_RADIO->CSTONES.NEXTFREQUENCY = chan;
	/* TODO: Use the peer FAE table and a characterized phase shift. */
	NRF_RADIO->CSTONES.FAEPEER = 0U;
	NRF_RADIO->CSTONES.PHASESHIFT = 0U;

	/* The input IQ samples are not down sampled, the rate tells the tone
	 * measurement unit the radio mode the samples are taken in.
	 */
	if (phy == BIT(1)) {
		rate = RADIO_CSTONES_DOWNSAMPLE_RATE_BLE2M;
	} else {
		rate = RADIO_CSTONES_DOWNSAMPLE_RATE_BLE1M;
	}

	NRF_RADIO->CSTONES.DOWNSAMPLE =
		((RADIO_CSTONES_DOWNSAMPLE_ENABLEFILTER_OFF <<
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
	magstd = (uint16_t)(NRF_RADIO->CSTONES.MAGSTD & RADIO_CSTONES_MAGSTD_MAGSTD_Msk);

	/* Reduce the 16-bit PCT to the 12-bit format of the HCI event */
	*i = (int16_t)(pct16 & 0xFFFFU) >> 4;
	*q = (int16_t)(pct16 >> 16) >> 4;

	mag = (uint16_t)((magphase & RADIO_CSTONES_MAGPHASEMEAN_MAG_Msk) >>
			 RADIO_CSTONES_MAGPHASEMEAN_MAG_Pos);

	/* TODO: Tone quality thresholds are not characterized on hardware. */
	if (mag < TONE_MAG_MIN) {
		return RADIO_CS_TONE_QUALITY_UNAVAILABLE;
	} else if ((magstd * 8U) < mag) {
		return RADIO_CS_TONE_QUALITY_HIGH;
	} else if ((magstd * 4U) < mag) {
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

void radio_cs_reset(void)
{
	NRF_RADIO->FREQFINETUNE = 0U;
	NRF_RADIO->CSTONES.MODE = RADIO_CSTONES_MODE_ResetValue;
	NRF_RADIO->CSTONES.NUMSAMPLES = RADIO_CSTONES_NUMSAMPLES_ResetValue;
	NRF_RADIO->CSTONES.NUMSAMPLESCOEFF = RADIO_CSTONES_NUMSAMPLESCOEFF_ResetValue;
	NRF_RADIO->CSTONES.DOWNSAMPLE = RADIO_CSTONES_DOWNSAMPLE_ResetValue;
	NRF_RADIO->EVENTS_CSTONESEND = 0U;
}
