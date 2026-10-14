/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Channel Sounding radio interface.
 *
 * Implemented for the nRF54L Series radio that provides the CSTONES tone
 * measurement unit (RADIO.CSTONES) and fine frequency tuning
 * (RADIO.FREQFINETUNE).
 */

/* RADIO.FREQFINETUNE step size is 488.28125 Hz, i.e. 1 MHz / 2048. */
#define RADIO_CS_FINE_TUNE_HZ_PER_STEP_NUM 15625U
#define RADIO_CS_FINE_TUNE_HZ_PER_STEP_DEN 32U

/* RADIO.FREQFINETUNE steps that offset the carrier by +250 kHz. */
#define RADIO_CS_FINE_TUNE_250KHZ 512

/* Tone quality as reported in the LE CS Subevent Result event. */
#define RADIO_CS_TONE_QUALITY_HIGH        0U
#define RADIO_CS_TONE_QUALITY_MEDIUM      1U
#define RADIO_CS_TONE_QUALITY_LOW         2U
#define RADIO_CS_TONE_QUALITY_UNAVAILABLE 3U

/* Configure the radio packet format used for CS_SYNC and tone packets: no
 * S0/length/S1 fields, a static payload of len octets, 4 octet access address,
 * no whitening and no CRC.
 */
void radio_cs_pkt_configure(uint8_t phy, uint8_t len);

/* Set the RADIO.FREQFINETUNE carrier offset in 488.28125 Hz steps. */
void radio_cs_fine_tune_set(int16_t steps);

/* Configure the tone measurement unit to measure num_samples samples at the
 * 2 MHz sample rate, rounded down to whole 8 us periods, with the radio in the
 * phy mode. ffo enables the fractional frequency offset (mode-0) measurement in
 * addition to the tone phase measurement.
 */
void radio_cs_tones_configure(uint8_t chan, uint8_t phy, uint8_t num_samples, bool ffo);

/* Start a tone measurement on the configured channel. */
void radio_cs_tones_start(void);

/* Returns true if the tone measurement started by radio_cs_tones_start()
 * completed.
 */
bool radio_cs_tones_is_done(void);

/* Wait a bounded time for a started tone measurement to complete. Returns
 * true if the tone measurement completed.
 */
bool radio_cs_tones_wait(void);

/* Clear the tone measurement completion event. */
void radio_cs_tones_status_reset(void);

/* Get the measured phase correction term as 12-bit signed I and Q values and
 * the tone quality indicator.
 */
uint8_t radio_cs_tones_pct_get(int16_t *i, int16_t *q);

/* Get the measured fractional frequency offset in 62.5 ppb units. */
int16_t radio_cs_tones_ffo_get(void);

/* Restore the radio Channel Sounding configuration to reset values. */
void radio_cs_reset(void);
