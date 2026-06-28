/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <soc.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/util_macro.h>
#include <zephyr/sys/byteorder.h>

#include "hal/ccm.h"
#include "hal/radio.h"
#include "hal/radio_cs.h"

/* The register level access to the RADIO Channel Sounding block is only
 * compiled in when the MDK for the target exposes the CS register definitions.
 * On targets/MDK revisions without the CS register block the abstraction keeps
 * a software shadow of the configuration so that the LLL state machine can be
 * exercised without driving the hardware. Refer to the nRF54L15 Product
 * Specification, RADIO chapter "Channel sounding" for the register details.
 */
#define HAL_CS_HW HAL_RADIO_NRF54LX_CS_REG_PRESENT

/* Software shadow of the current Channel Sounding step configuration. It is
 * used both for the no-hardware fallback and to keep the values that need to be
 * combined into a single hardware register write.
 */
static struct {
	uint8_t access_address[4];
	uint8_t mode;
	uint8_t channel_index;
	uint8_t num_tones;
	uint8_t tone_duration_us;
	uint8_t num_ant_paths;
	uint8_t antenna_selection;
	uint8_t rtt_type;
	void    *pct_buffer;
	size_t  pct_buffer_len;
} cs_cfg;

bool radio_cs_is_supported(void)
{
	return (bool)HAL_CS_HW;
}

void radio_cs_reset(void)
{
	(void)memset(&cs_cfg, 0, sizeof(cs_cfg));

#if HAL_CS_HW
	/* Return the RADIO Channel Sounding registers to their reset value and
	 * disable the tone generation/extraction path.
	 */
	NRF_RADIO->CS.MODE = 0U;
	NRF_RADIO->CS.TONES = 0U;
	NRF_RADIO->CS.ANTENNA = 0U;
	NRF_RADIO->CS.RTT = 0U;
	NRF_RADIO->CS.PCTPTR = 0U;
	NRF_RADIO->CS.PCTMAXCNT = 0U;
#endif /* HAL_CS_HW */
}

void radio_cs_mode_set(uint8_t mode)
{
	cs_cfg.mode = mode;

#if HAL_CS_HW
	/* Select the Channel Sounding step mode. Mode 0 is used for the
	 * frequency offset and timing recovery, mode 1 carries the RTT
	 * exchange, mode 2 carries the tone (phase based ranging) exchange and
	 * mode 3 carries both.
	 */
	NRF_RADIO->CS.MODE = ((uint32_t)mode << RADIO_CS_MODE_MODE_Pos) &
			     RADIO_CS_MODE_MODE_Msk;
#endif /* HAL_CS_HW */
}

void radio_cs_aa_set(const uint8_t * const access_address)
{
	memcpy(cs_cfg.access_address, access_address, sizeof(cs_cfg.access_address));

	/* The CS_SYNC packet uses the standard RADIO address matching path with
	 * a single logical address. Program the access address using the same
	 * BASE0/PREFIX0 layout used for connection events.
	 */
	NRF_RADIO->TXADDRESS = ((0UL << RADIO_TXADDRESS_TXADDRESS_Pos) &
				RADIO_TXADDRESS_TXADDRESS_Msk);
	NRF_RADIO->RXADDRESSES = (RADIO_RXADDRESSES_ADDR0_Enabled <<
				  RADIO_RXADDRESSES_ADDR0_Pos);
	NRF_RADIO->PREFIX0 = access_address[3];
	NRF_RADIO->BASE0 = sys_get_le24(access_address) << 8;
}

void radio_cs_chan_set(uint8_t channel_index)
{
	cs_cfg.channel_index = channel_index;

	/* Map the Channel Sounding channel index (0..78) to the RADIO FREQUENCY
	 * register value, i.e. the offset in MHz from 2400 MHz.
	 */
	NRF_RADIO->FREQUENCY = HAL_RADIO_NRF54LX_CS_CHAN_TO_FREQ(channel_index);
}

void radio_cs_tone_set(uint8_t num_tones, uint8_t tone_duration_us)
{
	cs_cfg.num_tones = num_tones;
	cs_cfg.tone_duration_us = tone_duration_us;

#if HAL_CS_HW
	/* Configure the number of tones and the duration of a single tone
	 * (T_PM) used during the tone extension slots of the step.
	 */
	NRF_RADIO->CS.TONES =
		(((uint32_t)num_tones << RADIO_CS_TONES_NUMTONES_Pos) &
		 RADIO_CS_TONES_NUMTONES_Msk) |
		(((uint32_t)tone_duration_us << RADIO_CS_TONES_DURATION_Pos) &
		 RADIO_CS_TONES_DURATION_Msk);
#endif /* HAL_CS_HW */
}

void radio_cs_antenna_set(uint8_t num_ant_paths, uint8_t antenna_selection)
{
	cs_cfg.num_ant_paths = num_ant_paths;
	cs_cfg.antenna_selection = antenna_selection;

#if HAL_CS_HW
	/* Configure the antenna paths used for the tone slots together with the
	 * antenna permutation index for the step.
	 */
	NRF_RADIO->CS.ANTENNA =
		(((uint32_t)num_ant_paths << RADIO_CS_ANTENNA_PATHS_Pos) &
		 RADIO_CS_ANTENNA_PATHS_Msk) |
		(((uint32_t)antenna_selection << RADIO_CS_ANTENNA_SEL_Pos) &
		 RADIO_CS_ANTENNA_SEL_Msk);
#endif /* HAL_CS_HW */
}

void radio_cs_rtt_type_set(uint8_t rtt_type)
{
	cs_cfg.rtt_type = rtt_type;

#if HAL_CS_HW
	/* Select the RTT payload type used to timestamp the CS_SYNC exchange. */
	NRF_RADIO->CS.RTT = ((uint32_t)rtt_type << RADIO_CS_RTT_TYPE_Pos) &
			    RADIO_CS_RTT_TYPE_Msk;
#endif /* HAL_CS_HW */
}

void radio_cs_pct_buffer_set(void *buffer, size_t len)
{
	cs_cfg.pct_buffer = buffer;
	cs_cfg.pct_buffer_len = len;

#if HAL_CS_HW
	/* Point the radio EasyDMA at the buffer used to store the captured
	 * IQ/phase (PCT) samples of the received tones.
	 */
	NRF_RADIO->CS.PCTPTR = (uint32_t)buffer;
	NRF_RADIO->CS.PCTMAXCNT =
		(uint32_t)(len / sizeof(struct radio_cs_iq_sample));
#endif /* HAL_CS_HW */
}

uint32_t radio_cs_rtt_timestamp_get(void)
{
#if HAL_CS_HW
	/* The radio captures the time of arrival of the received CS_SYNC packet
	 * relative to the step start, expressed in radio time-base ticks.
	 */
	return NRF_RADIO->CS.RTTSTATUS;
#else /* !HAL_CS_HW */
	return 0U;
#endif /* !HAL_CS_HW */
}

uint32_t radio_cs_pct_get(struct radio_cs_iq_sample *samples, uint32_t max)
{
	uint32_t count;

	if ((samples == NULL) || (max == 0U) || (cs_cfg.pct_buffer == NULL)) {
		return 0U;
	}

#if HAL_CS_HW
	count = NRF_RADIO->CS.PCTAMOUNT;
#else /* !HAL_CS_HW */
	count = 0U;
#endif /* !HAL_CS_HW */

	if (count > max) {
		count = max;
	}

	if (count > 0U) {
		(void)memcpy(samples, cs_cfg.pct_buffer,
			     count * sizeof(struct radio_cs_iq_sample));
	}

	return count;
}
