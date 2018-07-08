/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdbool.h>

#include <toolchain.h>
#include <zephyr/types.h>
#include <misc/util.h>
#include <drivers/clock_control/nrf5_clock_control.h>

#include "util/memq.h"

#include "hal/ccm.h"
#include "hal/radio.h"

#include "pdu.h"

#include "ull_types.h"

#include "lll.h"
#include "lll_conn.h"

#include "lll_internal.h"
#include "lll_tim_internal.h"

#include "common/log.h"
#include <soc.h>
#include "hal/debug.h"

static int init_reset(void);
static void isr_done(void *param);
static void isr_cleanup(void *param);
static void isr_race(void *param);

static u16_t const sca_ppm_lut[] = {500, 250, 150, 100, 75, 50, 30, 20};

int lll_conn_init(void)
{
	int err;

	err = init_reset();
	if (err) {
		return err;
	}

	return 0;
}

int lll_conn_reset(void)
{
	int err;

	err = init_reset();
	if (err) {
		return err;
	}

	return 0;
}

u32_t lll_conn_ppm_local_get(void)
{
	return sca_ppm_lut[CLOCK_CONTROL_NRF5_K32SRC_ACCURACY];
}

u32_t lll_conn_ppm_get(u8_t sca)
{
	return sca_ppm_lut[sca];
}

int lll_conn_is_abort_cb(void *next, int prio, void *curr,
			 lll_prepare_cb_t *resume_cb, int *resume_prio)
{
	return -ECANCELED;
}

void lll_conn_abort_cb(struct lll_prepare_param *prepare_param, void *param)
{
	int err;

	/* NOTE: This is not a prepare being cancelled */
	if (!prepare_param) {
		/* Perform event abort here.
		 * After event has been cleanly aborted, clean up resources
		 * and dispatch event done.
		 */
		radio_isr_set(lll_conn_isr_abort, param);
		radio_disable();
		return;
	}

	/* NOTE: Else clean the top half preparations of the aborted event
	 * currently in preparation pipeline.
	 */
	err = lll_clk_off();
	LL_ASSERT(!err || err == -EBUSY);

	lll_done(param);
}

void lll_conn_isr_rx(void *param)
{
	u8_t trx_done;
	u8_t crc_ok;
	u8_t rssi_ready;

	/* Read radio status and events */
	trx_done = radio_is_done();
	if (trx_done) {

#if defined(CONFIG_BT_CTLR_PROFILE_ISR)
		/* sample the packet timer here, use it to calculate ISR latency
		 * and generate the profiling event at the end of the ISR.
		 */
		radio_tmr_sample();
#endif /* CONFIG_BT_CTLR_PROFILE_ISR */

		crc_ok = radio_crc_is_valid();
		rssi_ready = radio_rssi_is_ready();
	} else {
		crc_ok = rssi_ready = 0;
	}

	/* Clear radio status and events */
	radio_status_reset();
	radio_tmr_status_reset();
	radio_rssi_status_reset();

#if defined(CONFIG_BT_CTLR_GPIO_PA_PIN) || \
	defined(CONFIG_BT_CTLR_GPIO_LNA_PIN)
	radio_gpio_pa_lna_disable();
#endif /* CONFIG_BT_CTLR_GPIO_PA_PIN || CONFIG_BT_CTLR_GPIO_LNA_PIN */

	if (!trx_done) {
		radio_isr_set(isr_done, param);
		radio_disable();

		return;
	}

	/* TODO: coming soon... */
}

void lll_conn_isr_tx(void *param)
{
	struct lll_conn *lll = (void *)param;
	u32_t hcto;

	/* TODO: MOVE to a common interface, isr_lll_radio_status? */
	/* Clear radio status and events */
	radio_status_reset();
	radio_tmr_status_reset();
	radio_filter_status_reset();
	radio_ar_status_reset();
	radio_rssi_status_reset();

#if defined(CONFIG_BT_CTLR_GPIO_PA_PIN) || \
	defined(CONFIG_BT_CTLR_GPIO_LNA_PIN)
	radio_gpio_pa_lna_disable();
#endif /* CONFIG_BT_CTLR_GPIO_PA_PIN || CONFIG_BT_CTLR_GPIO_LNA_PIN */
	/* TODO: MOVE ^^ */

	radio_isr_set(lll_conn_isr_rx, param);
	radio_tmr_tifs_set(TIFS_US);
#if defined(CONFIG_BT_CTLR_PHY)
	radio_switch_complete_and_tx(lll->phy_rx, 0,
				     lll->phy_tx,
				     lll->phy_flags);
#else /* !CONFIG_BT_CTLR_PHY */
	radio_switch_complete_and_tx(0, 0, 0, 0);
#endif /* !CONFIG_BT_CTLR_PHY */

	lll_conn_rx_pkt_set(lll);

	/* assert if radio packet ptr is not set and radio started rx */
	LL_ASSERT(!radio_is_ready());

	/* +/- 2us active clock jitter, +1 us hcto compensation */
	hcto = radio_tmr_tifs_base_get() + TIFS_US + 4 + 1;
#if defined(CONFIG_BT_CTLR_PHY)
	hcto += radio_rx_chain_delay_get(lll->phy_rx, 1);
	hcto += addr_us_get(lll->phy_rx);
	hcto -= radio_tx_chain_delay_get(lll->phy_tx, lll->phy_flags);
#else /* !CONFIG_BT_CTLR_PHY */
	hcto += radio_rx_chain_delay_get(0, 0);
	hcto += addr_us_get(0);
	hcto -= radio_tx_chain_delay_get(0, 0);
#endif /* !CONFIG_BT_CTLR_PHY */

	radio_tmr_hcto_configure(hcto);

	/* capture end of CONNECT_IND PDU, used for calculating first
	 * slave event.
	 */
	radio_tmr_end_capture();

#if defined(CONFIG_BT_CTLR_SCAN_REQ_RSSI)
	radio_rssi_measure();
#endif /* CONFIG_BT_CTLR_SCAN_REQ_RSSI */

#if defined(CONFIG_BT_CTLR_GPIO_LNA_PIN)
	radio_gpio_lna_setup();
#if defined(CONFIG_BT_CTLR_PHY)
	radio_gpio_pa_lna_enable(radio_tmr_tifs_base_get() + TIFS_US - 4 -
				 radio_tx_chain_delay_get(lll->phy_tx,
							  lll->phy_flags) -
				 CONFIG_BT_CTLR_GPIO_LNA_OFFSET);
#else /* !CONFIG_BT_CTLR_PHY */
	radio_gpio_pa_lna_enable(radio_tmr_tifs_base_get() + TIFS_US - 4 -
				 radio_tx_chain_delay_get(0, 0) -
				 CONFIG_BT_CTLR_GPIO_LNA_OFFSET);
#endif /* !CONFIG_BT_CTLR_PHY */
#endif /* CONFIG_BT_CTLR_GPIO_LNA_PIN */
}

void lll_conn_isr_abort(void *param)
{
	isr_cleanup(param);
}

void lll_conn_rx_pkt_set(struct lll_conn *lll)
{
	struct node_rx_pdu *node_rx;
	u16_t max_rx_octets;
	u8_t phy;

	node_rx = ull_pdu_rx_alloc_peek(1);
	LL_ASSERT(node_rx);

#if defined(CONFIG_BT_CTLR_DATA_LENGTH)
	max_rx_octets = lll->max_rx_octets;
#else /* !CONFIG_BT_CTLR_DATA_LENGTH */
	max_rx_octets = PDU_DC_PAYLOAD_SIZE_MIN;
#endif /* !CONFIG_BT_CTLR_DATA_LENGTH */

#if defined(CONFIG_BT_CTLR_PHY)
	phy = lll->phy_rx;
#else /* !CONFIG_BT_CTLR_PHY */
	phy = 0;
#endif /* !CONFIG_BT_CTLR_PHY */

	radio_phy_set(phy, 0);

	if (lll->enc_rx) {
		radio_pkt_configure(8, (max_rx_octets + 4), (phy << 1) | 0x01);

		radio_pkt_rx_set(radio_ccm_rx_pkt_set(&lll->ccm_rx, phy,
						      node_rx->pdu));
	} else {
		radio_pkt_configure(8, max_rx_octets, (phy << 1) | 0x01);

		radio_pkt_rx_set(node_rx->pdu);
	}
}

static int init_reset(void)
{
	return 0;
}

static void isr_done(void *param)
{
	/* TODO: MOVE to a common interface, isr_lll_radio_status? */
	/* Clear radio status and events */
	radio_status_reset();
	radio_tmr_status_reset();
	radio_filter_status_reset();
	radio_ar_status_reset();
	radio_rssi_status_reset();

#if defined(CONFIG_BT_CTLR_GPIO_PA_PIN) || \
	defined(CONFIG_BT_CTLR_GPIO_LNA_PIN)
	radio_gpio_pa_lna_disable();
#endif /* CONFIG_BT_CTLR_GPIO_PA_PIN || CONFIG_BT_CTLR_GPIO_LNA_PIN */
	/* TODO: MOVE ^^ */

	/* Local initiated terminate happened */
	if (!param) {
		goto isr_done_cleanup;
	}

isr_done_cleanup:
	isr_cleanup(param);
}

static void isr_cleanup(void *param)
{
	int err;

	radio_isr_set(isr_race, param);
	radio_tmr_stop();

	err = lll_clk_off();
	LL_ASSERT(!err || err == -EBUSY);

	lll_done(NULL);
}

static void isr_race(void *param)
{
	/* NOTE: lll_disable could have a race with ... */
	radio_status_reset();
}
