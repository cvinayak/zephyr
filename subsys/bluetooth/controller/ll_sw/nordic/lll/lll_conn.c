/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdbool.h>
#include <stddef.h>

#include <toolchain.h>
#include <zephyr/types.h>
#include <misc/util.h>
#include <drivers/clock_control/nrf5_clock_control.h>

#include "util/memq.h"
#include "util/mfifo.h"

#include "hal/ccm.h"
#include "hal/radio.h"

#include "pdu.h"

#include "lll.h"
#include "lll_conn.h"

#include "lll_internal.h"
#include "lll_tim_internal.h"
#include "lll_prof_internal.h"

#include "common/log.h"
#include <soc.h>
#include "hal/debug.h"

static int init_reset(void);
static void isr_done(void *param);
static void isr_cleanup(void *param);
static void isr_race(void *param);
static u32_t isr_rx_pdu(struct lll_conn *lll, struct pdu_data *pdu_data_rx,
			struct node_tx **tx_release, u8_t *is_rx_enqueue);
static struct pdu_data *empty_tx_enqueue(struct lll_conn *lll);

static u16_t const sca_ppm_lut[] = {500, 250, 150, 100, 75, 50, 30, 20};
static u8_t crc_expire;
static u16_t trx_cnt;

static MFIFO_DEFINE(conn_ack, sizeof(struct lll_tx),
		    CONFIG_BT_CTLR_TX_BUFFERS);

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

	MFIFO_INIT(conn_ack);

	err = init_reset();
	if (err) {
		return err;
	}

	return 0;
}

u8_t lll_conn_sca_local_get(void)
{
	return CLOCK_CONTROL_NRF5_K32SRC_ACCURACY;
}

u32_t lll_conn_ppm_local_get(void)
{
	return sca_ppm_lut[CLOCK_CONTROL_NRF5_K32SRC_ACCURACY];
}

u32_t lll_conn_ppm_get(u8_t sca)
{
	return sca_ppm_lut[sca];
}

void lll_conn_prepare_reset(void)
{
	trx_cnt = 0;
	crc_expire = 0;
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
		radio_isr_set(isr_done, param);
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
	struct lll_conn *lll = param;
	struct pdu_data *pdu_data_rx;
	struct pdu_data *pdu_data_tx;
	struct node_rx_pdu *node_rx;
	struct node_tx *tx_release;
	u8_t is_empty_pdu_tx_retry;
	u8_t is_crc_backoff = 0;
	u8_t is_rx_enqueue = 0;
	u8_t rssi_ready;
	u8_t trx_done;
	u8_t is_done;
	u8_t crc_ok;

#if defined(CONFIG_BT_CTLR_PROFILE_ISR)
	lll_prof_latency_capture();
#endif /* CONFIG_BT_CTLR_PROFILE_ISR */

	/* Read radio status and events */
	trx_done = radio_is_done();
	if (trx_done) {
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

	trx_cnt++;

	node_rx = ull_pdu_rx_alloc_peek(1);
	LL_ASSERT(node_rx);

	pdu_data_rx = (void *)node_rx->pdu;

	if (crc_ok) {
		u32_t err;

		err = isr_rx_pdu(lll, pdu_data_rx, &tx_release, &is_rx_enqueue);
		if (err) {
			goto lll_conn_isr_rx_exit;
		}

		/* Reset CRC expiry counter */
		crc_expire = 0;

		/* Reset supervision counter */
		lll->supervision_expire = 0;
	} else {
		/* Start CRC error countdown, if not already started */
		if (crc_expire == 0) {
			crc_expire = 2;
		}

		/* CRC error countdown */
		crc_expire--;
		is_crc_backoff = (crc_expire == 0);
	}

	/* prepare tx packet */
	is_empty_pdu_tx_retry = lll->empty;
	lll_conn_pdu_tx_prep(lll, &pdu_data_tx);

	/* Decide on event continuation and hence Radio Shorts to use */
	is_done = is_crc_backoff || ((crc_ok) && (pdu_data_rx->md == 0) &&
				     (pdu_data_tx->len == 0));

	if (is_done) {
		radio_isr_set(isr_done, param);

		if (0) {
#if defined(CONFIG_BT_CENTRAL)
		/* Event done for master */
		} else if (!lll->role) {
			radio_disable();

			/* assert if radio packet ptr is not set and radio
			 * started tx.
			 */
			LL_ASSERT(!radio_is_ready());

			/* Restore state if last transmitted was empty PDU */
			lll->empty = is_empty_pdu_tx_retry;

			goto lll_conn_isr_rx_exit;
#elif defined(CONFIG_BT_PERIPHERAL)
		/* Event done for slave */
		} else {
			radio_switch_complete_and_disable();
#endif /* CONFIG_BT_PERIPHERAL */
		}
	} else {
		radio_isr_set(lll_conn_isr_tx, param);
		radio_tmr_tifs_set(TIFS_US);

#if defined(CONFIG_BT_CTLR_PHY)
		radio_switch_complete_and_rx(lll->phy_rx);
#else /* !CONFIG_BT_CTLR_PHY */
		radio_switch_complete_and_rx(0);
#endif /* !CONFIG_BT_CTLR_PHY */

		/* capture end of Tx-ed PDU, used to calculate HCTO. */
		radio_tmr_end_capture();
	}

	/* Fill sn and nesn */
	pdu_data_tx->sn = lll->sn;
	pdu_data_tx->nesn = lll->nesn;

	/* setup the radio tx packet buffer */
	lll_conn_tx_pkt_set(lll, pdu_data_tx);

#if defined(CONFIG_BT_CTLR_GPIO_PA_PIN)

#if defined(CONFIG_BT_CTLR_PROFILE_ISR)
	/* PA enable is overwriting packet end used in ISR profiling, hence
	 * back it up for later use.
	 */
	lll_prof_radio_end_backup();
#endif /* CONFIG_BT_CTLR_PROFILE_ISR */

	radio_gpio_pa_setup();

#if defined(CONFIG_BT_CTLR_PHY)
	radio_gpio_pa_lna_enable(radio_tmr_tifs_base_get() + TIFS_US -
				 radio_rx_chain_delay_get(lll->phy_rx, 1) -
				 CONFIG_BT_CTLR_GPIO_PA_OFFSET);
#else /* !CONFIG_BT_CTLR_PHY */
	radio_gpio_pa_lna_enable(radio_tmr_tifs_base_get() + TIFS_US -
				 radio_rx_chain_delay_get(0, 0) -
				 CONFIG_BT_CTLR_GPIO_PA_OFFSET);
#endif /* !CONFIG_BT_CTLR_PHY */
#endif /* CONFIG_BT_CTLR_GPIO_PA_PIN */

	/* assert if radio packet ptr is not set and radio started tx */
	LL_ASSERT(!radio_is_ready());

lll_conn_isr_rx_exit:
	/* Save the AA captured for the first Rx in connection event */
	if (!radio_tmr_aa_restore()) {
		radio_tmr_aa_save(radio_tmr_aa_get());
	}

#if defined(CONFIG_BT_CTLR_PROFILE_ISR)
	lll_prof_cputime_capture();
#endif /* CONFIG_BT_CTLR_PROFILE_ISR */

	/* TODO: */
	#if 0
	/* release tx node and generate event for num complete */
	if (tx_release) {
		pdu_node_tx_release(_radio.conn_curr->handle, tx_release);
	}

	/* enqueue any rx packet/event towards application */
	if (rx_enqueue) {
		/* set data flow control lock on currently rx-ed connection */
		rx_fc_lock(_radio.conn_curr->handle);

		/* set the connection handle and enqueue */
		node_rx->hdr.handle = _radio.conn_curr->handle;
		packet_rx_enqueue();
	}
	#endif

#if defined(CONFIG_BT_CTLR_CONN_RSSI)
	/* Collect RSSI for connection */
	if (rssi_ready) {
		u8_t rssi = radio_rssi_get();

		_radio.conn_curr->rssi_latest = rssi;

		if (((_radio.conn_curr->rssi_reported - rssi) & 0xFF) >
		    RADIO_RSSI_THRESHOLD) {
			if (_radio.conn_curr->rssi_sample_count) {
				_radio.conn_curr->rssi_sample_count--;
			}
		} else {
			_radio.conn_curr->rssi_sample_count =
				RADIO_RSSI_SAMPLE_COUNT;
		}
	}
#else /* !CONFIG_BT_CTLR_CONN_RSSI */
	ARG_UNUSED(rssi_ready);
#endif /* !CONFIG_BT_CTLR_CONN_RSSI */

#if defined(CONFIG_BT_CTLR_PROFILE_ISR)
	lll_prof_send();
#endif /* CONFIG_BT_CTLR_PROFILE_ISR */

	return;
}

void lll_conn_isr_tx(void *param)
{
	struct lll_conn *lll = (void *)param;
	u32_t hcto;

	/* TODO: MOVE to a common interface, isr_lll_radio_status? */
	/* Clear radio status and events */
	radio_status_reset();
	radio_tmr_status_reset();

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

#if defined(CONFIG_BT_CENTRAL) && defined(CONFIG_BT_CTLR_CONN_RSSI)
	if (!lll->role) {
		radio_rssi_measure();
	}
#endif /* iCONFIG_BT_CENTRAL && CONFIG_BT_CTLR_CONN_RSSI */

#if defined(CONFIG_BT_CTLR_PROFILE_ISR) || \
	defined(CONFIG_BT_CTLR_GPIO_PA_PIN)
	radio_tmr_end_capture();
#endif /* CONFIG_BT_CTLR_PROFILE_ISR */

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

void lll_conn_tx_pkt_set(struct lll_conn *lll, struct pdu_data *pdu_data_tx)
{
	u16_t max_tx_octets;
	u8_t phy, flags;

#if defined(CONFIG_BT_CTLR_DATA_LENGTH)
	max_tx_octets = lll->max_tx_octets;
#else /* !CONFIG_BT_CTLR_DATA_LENGTH */
	max_tx_octets = PDU_DC_PAYLOAD_SIZE_MIN;
#endif /* !CONFIG_BT_CTLR_DATA_LENGTH */

#if defined(CONFIG_BT_CTLR_PHY)
	phy = lll->phy_tx;
	flags = lll->phy_flags;
#else /* !CONFIG_BT_CTLR_PHY */
	phy = 0;
	flags = 0;
#endif /* !CONFIG_BT_CTLR_PHY */

	radio_phy_set(phy, flags);

	if (lll->enc_tx) {
		radio_pkt_configure(8, (max_tx_octets + 4), (phy << 1) | 0x01);

		radio_pkt_tx_set(radio_ccm_tx_pkt_set(&lll->ccm_tx,
						      pdu_data_tx));
	} else {
		radio_pkt_configure(8, max_tx_octets, (phy << 1) | 0x01);

		radio_pkt_tx_set(pdu_data_tx);
	}
}

void lll_conn_pdu_tx_prep(struct lll_conn *lll, struct pdu_data **pdu_data_tx)
{
	struct pdu_data *p;

	/* TODO: */
	p = empty_tx_enqueue(lll);

	*pdu_data_tx = p;
}

u8_t lll_conn_ack_last_idx_get(void)
{
	return mfifo_conn_ack.l;
}

memq_link_t *lll_conn_ack_peek(u16_t *handle, struct node_tx **node_tx)
{
	struct lll_tx *tx;

	tx = MFIFO_DEQUEUE_GET(conn_ack);
	if (!tx) {
		return NULL;
	}

	*handle = tx->handle;
	*node_tx = tx->node;

	return (*node_tx)->link;
}

memq_link_t *lll_conn_ack_by_last_peek(u8_t last, u16_t *handle,
				      struct node_tx **node_tx)
{
	struct lll_tx *tx;

	tx = mfifo_dequeue_get(mfifo_conn_ack.m, mfifo_conn_ack.s,
			       mfifo_conn_ack.f, last);
	if (!tx) {
		return NULL;
	}

	*handle = tx->handle;
	*node_tx = tx->node;

	return (*node_tx)->link;
}

void *lll_conn_ack_dequeue(void)
{
	return MFIFO_DEQUEUE(conn_ack);
}

static int init_reset(void)
{
	return 0;
}

static void isr_done(void *param)
{
	struct event_done_extra *e;

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

	e = ull_event_done_extra_get();
	e->type = EVENT_DONE_EXTRA_TYPE_CONN;
	e->trx_cnt = trx_cnt;
	if (trx_cnt) {
		struct lll_conn *lll = param;

		if (IS_ENABLED(CONFIG_BT_PERIPHERAL) && lll->role) {
			u32_t preamble_to_addr_us;

#if defined(CONFIG_BT_CTLR_PHY)
			preamble_to_addr_us =
				addr_us_get(lll->phy_rx);
#else /* !CONFIG_BT_CTLR_PHY */
			preamble_to_addr_us =
				addr_us_get(0);
#endif /* !CONFIG_BT_CTLR_PHY */

			e->slave.start_to_address_actual_us =
				radio_tmr_aa_restore() - radio_tmr_ready_get();
			e->slave.window_widening_event_us =
				lll->slave.window_widening_event_us;
			e->slave.preamble_to_addr_us = preamble_to_addr_us;

			/* Reset window widening, as anchor point sync-ed */
			lll->slave.window_widening_event_us = 0;
			lll->slave.window_size_event_us = 0;
		}

		/* TODO: event sync-ed */
	}

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

static u32_t isr_rx_pdu(struct lll_conn *lll, struct pdu_data *pdu_data_rx,
			struct node_tx **tx_release, u8_t *is_rx_enqueue)
{
	/* Ack for tx-ed data */
	if (pdu_data_rx->nesn != lll->sn) {
		/* Increment serial number */
		lll->sn++;

		/* First ack (and redundantly any other ack) enable use of
		 * slave latency.
		 */
		if (IS_ENABLED(CONFIG_BT_PERIPHERAL) && lll->role) {
			lll->slave.latency_enabled = 1;
		}

		if (!lll->empty) {
			/* TODO: */
			LL_ASSERT(0);
		} else {
			lll->empty = 0;
		}
	}

	/* process received data */
	if ((pdu_data_rx->sn == lll->nesn) &&
	    /* check so that we will NEVER use the rx buffer reserved for empty
	     * packet and internal control enqueue
	     */
	    (ull_pdu_rx_alloc_peek(3) != 0)) {
		u8_t ccm_rx_increment = 0;
		u8_t nack = 0;

		if (pdu_data_rx->len != 0) {
#if 0
			/* If required, wait for CCM to finish
			 */
			if (_radio.conn_curr->enc_rx) {
				u32_t done;

				done = radio_ccm_is_done();
				LL_ASSERT(done);

				ccm_rx_increment = 1;
			}

			/* MIC Failure Check or data rx during pause */
			if ((_radio.conn_curr->enc_rx &&
			     !radio_ccm_mic_is_valid()) ||
			    (_radio.conn_curr->pause_rx &&
			     isr_rx_conn_enc_unexpected(_radio.conn_curr,
							pdu_data_rx))) {
				_radio.state = STATE_CLOSE;
				radio_disable();

				/* assert if radio packet ptr is not set and
				 * radio started tx
				 */
				LL_ASSERT(!radio_is_ready());

				terminate_ind_rx_enqueue(_radio.conn_curr,
							 0x3d);

				connection_release(_radio.conn_curr);
				_radio.conn_curr = NULL;

				return 1; /* terminated */
			}

#if defined(CONFIG_BT_CTLR_LE_PING)
			/* stop authenticated payload (pre) timeout */
			_radio.conn_curr->appto_expire = 0;
			_radio.conn_curr->apto_expire = 0;
#endif /* CONFIG_BT_CTLR_LE_PING */

			switch (pdu_data_rx->ll_id) {
			case PDU_DATA_LLID_DATA_CONTINUE:
			case PDU_DATA_LLID_DATA_START:
				/* enqueue data packet */
				*rx_enqueue = 1;
				break;

			case PDU_DATA_LLID_CTRL:
				nack = isr_rx_conn_pkt_ctrl(node_rx,
							    rx_enqueue);
				break;
			case PDU_DATA_LLID_RESV:
			default:
				/* Invalid LL id, drop it. */
				break;
			}
#endif

#if defined(CONFIG_BT_CTLR_LE_PING)
		} else if (lll->enc_rx || lll->pause_rx) {
			u16_t appto_reload_new;

			/* check for change in apto */
			appto_reload_new = (lll->apto_reload >
					    (lll->latency + 6)) ?
					   (lll->apto_reload -
					    (lll->latency + 6)) :
					   lll->apto_reload;
			if (lll->appto_reload != appto_reload_new) {
				lll->appto_reload = appto_reload_new;
				lll->apto_expire = 0;
			}

			/* start authenticated payload (pre) timeout */
			if (lll->apto_expire == 0) {
				lll->appto_expire = lll->appto_reload;
				lll->apto_expire = lll->apto_reload;
			}
#endif /* CONFIG_BT_CTLR_LE_PING */

		}

		if (!nack) {
			lll->nesn++;

			if (ccm_rx_increment) {
				lll->ccm_rx.counter++;
			}
		}
	}

	return 0;
}

static struct pdu_data *empty_tx_enqueue(struct lll_conn *lll)
{
	struct pdu_data *p;

	lll->empty = 1;

	p = (void *)radio_pkt_empty_get();
	p->ll_id = PDU_DATA_LLID_DATA_CONTINUE;
	p->len = 0;
	if (lll->pkt_tx_head) {
		p->md = 1;
	} else {
		p->md = 0;
	}

	return p;
}
