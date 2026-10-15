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

/* Select the radio enable task triggered by the radio timer start, trx 1 for
 * Tx and 0 for Rx, before radio_tmr_start() or radio_tmr_start_us(). The Tx
 * and Rx enable on tick share a (D)PPI channel and the enable task of the
 * other direction may still subscribe to it, from a previous action or event
 * not using the software switch.
 */
void radio_cs_trx_enable_select(uint8_t trx);

/* Set the RADIO.FREQFINETUNE carrier offset in 488.28125 Hz steps. */
void radio_cs_fine_tune_set(int16_t steps);

/* Keep the radio in TXIDLE after the packet, transmitting an unmodulated
 * carrier until radio_cs_tx_carrier_end() is called. Call after the radio
 * switch is configured.
 */
void radio_cs_tx_carrier_hold(void);

/* Transmit only the unmodulated carrier from TXIDLE, without a packet, until
 * radio_cs_tx_carrier_end() is called. Call after the radio switch is
 * configured.
 */
void radio_cs_tx_carrier_only(void);

/* End the unmodulated carrier by disabling the radio. */
void radio_cs_tx_carrier_end(void);

/* Configure the tone measurement unit to measure num_samples samples at the
 * 2 MHz sample rate, rounded down to whole 8 us periods, with the radio in the
 * phy mode. The fractional frequency offset measurement is always enabled;
 * ffo selects it alone (mode-0), else the tone phase measurement is enabled in
 * addition (mode-2). fae_peer is the peer frequency actuation error in
 * 0.03125 ppm units, the unit of the FAE table, and only applies to the mode-0
 * measurement; pass 0 otherwise.
 */
void radio_cs_tones_configure(uint8_t chan, uint8_t phy, uint8_t num_samples, bool ffo,
			      int8_t fae_peer);

/* Start a tone measurement on the configured channel. */
void radio_cs_tones_start(void);

/* Start a tone measurement on the configured channel by hardware when the
 * radio timer reaches start_us. Returns the radio timer value the measurement
 * is started at, later than start_us if start_us has already passed.
 */
uint32_t radio_tmr_cs_tones_start_us(uint32_t start_us);

/* Cancel a tone measurement start setup by radio_tmr_cs_tones_start_us(). */
void radio_tmr_cs_tones_start_clear(void);

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

/* Get the measured mean tone magnitude and its standard deviation, the inputs
 * of the tone quality indicator.
 */
void radio_cs_tones_mag_get(uint16_t *mag, uint16_t *magstd);

/* Restore the radio Channel Sounding configuration to reset values. */
void radio_cs_reset(void);
