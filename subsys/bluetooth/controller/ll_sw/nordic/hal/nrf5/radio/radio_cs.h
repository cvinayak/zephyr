/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* HAL abstraction for the RADIO Channel Sounding (CS) hardware block.
 *
 * These interfaces wrap the nRF RADIO Channel Sounding registers used by the
 * LLL to run the CS step radio events: tone (phase based ranging) generation
 * and extraction, Round Trip Time (RTT) timestamping of the CS_SYNC packets,
 * and IQ/phase (PCT) sampling of the received tones. Refer to the nRF54L15
 * Product Specification, RADIO chapter "Channel sounding" for the register
 * details and to the Bluetooth Core specification, Vol 6, Part A, Section 5
 * for the protocol requirements.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Channel Sounding step modes. */
#define RADIO_CS_MODE_0 0U /* Frequency offset and timing recovery. */
#define RADIO_CS_MODE_1 1U /* RTT (time of flight) exchange. */
#define RADIO_CS_MODE_2 2U /* Tone (phase based ranging) exchange. */
#define RADIO_CS_MODE_3 3U /* Combined RTT and tone exchange. */

/* RTT types, matching the Bluetooth Core specification values. */
#define RADIO_CS_RTT_TYPE_AA_ONLY        0U
#define RADIO_CS_RTT_TYPE_SOUNDING_32BIT 1U
#define RADIO_CS_RTT_TYPE_SOUNDING_96BIT 2U
#define RADIO_CS_RTT_TYPE_RANDOM_32BIT   3U
#define RADIO_CS_RTT_TYPE_RANDOM_64BIT   4U
#define RADIO_CS_RTT_TYPE_RANDOM_96BIT   5U
#define RADIO_CS_RTT_TYPE_RANDOM_128BIT  6U

/* In-phase/quadrature (PCT) sample extracted from a received CS tone. */
struct radio_cs_iq_sample {
	int16_t i;
	int16_t q;
};

/* Returns true if the RADIO peripheral on the target exposes the Channel
 * Sounding register block, i.e. CS radio events can run on hardware.
 */
bool radio_cs_is_supported(void);

/* Reset the RADIO Channel Sounding configuration to a known idle state. */
void radio_cs_reset(void);

/* Configure the RADIO for a Channel Sounding step of the given mode
 * (RADIO_CS_MODE_0 .. RADIO_CS_MODE_3).
 */
void radio_cs_mode_set(uint8_t mode);

/* Set the access address used for the CS_SYNC packet of the current step. The
 * initiator and reflector use distinct access addresses negotiated during the
 * CS configuration procedure.
 */
void radio_cs_aa_set(const uint8_t * const access_address);

/* Select the RF channel for the current CS step from a CS channel index
 * (0..78). The index is mapped to (2402 + index) MHz.
 */
void radio_cs_chan_set(uint8_t channel_index);

/* Configure the tone extension used for phase based ranging: the number of
 * tones (antenna paths plus the optional tone extension slot) and the duration
 * of a single tone (T_PM) in microseconds.
 */
void radio_cs_tone_set(uint8_t num_tones, uint8_t tone_duration_us);

/* Configure the antenna paths used for the tone slots and the antenna
 * selection (permutation) index for the step.
 */
void radio_cs_antenna_set(uint8_t num_ant_paths, uint8_t antenna_selection);

/* Configure the RTT payload type (RADIO_CS_RTT_TYPE_*). */
void radio_cs_rtt_type_set(uint8_t rtt_type);

/* Provide the buffer used by the radio to store the captured IQ/phase (PCT)
 * samples of the received tones during the current step.
 */
void radio_cs_pct_buffer_set(void *buffer, size_t len);

/* Retrieve the RTT timestamp, in radio time-base ticks, captured for the last
 * received CS_SYNC packet of the current step.
 */
uint32_t radio_cs_rtt_timestamp_get(void);

/* Copy up to max captured PCT samples for the current step into samples and
 * return the number of samples copied.
 */
uint32_t radio_cs_pct_get(struct radio_cs_iq_sample *samples, uint32_t max);
