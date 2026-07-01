/*
 * Copyright (c) 2026 The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SUBSYS_BLUETOOTH_CONTROLLER_LL_SW_NORDIC_LLL_CONN_ISO_FLUSH_H_
#define ZEPHYR_SUBSYS_BLUETOOTH_CONTROLLER_LL_SW_NORDIC_LLL_CONN_ISO_FLUSH_H_

#include <stdint.h>

#include "lll_conn_iso.h"

static inline uint8_t lll_flush_subevent_limit(uint8_t nse, uint8_t bn, uint64_t payload_count)
{
	return nse - ((nse / bn) * (bn - 1U - (payload_count % bn)));
}

static inline void lll_flush_iso_rx_data_lost(struct lll_conn_iso_stream *cis_lll,
					      uint32_t timestamp)
{
	struct lll_conn_iso_group *cig_lll;
	struct node_rx_iso_meta *iso_meta;
	struct node_rx_pdu *node_rx;
	struct pdu_cis *pdu_rx;

	/* Only report lost ISO data when an Rx data path is set up to consume
	 * it, e.g. to drive Packet Loss Concealment (PLC) in the LC3 codec.
	 */
	if (!cis_lll->datapath_ready_rx) {
		return;
	}

	/* Two free Rx buffers are required: one is consumed to report the lost
	 * ISO data and the other ensures a buffer remains available for the
	 * radio DMA to receive in subsequent subevents/events.
	 */
	node_rx = ull_iso_pdu_rx_alloc_peek(2U);
	if (!node_rx) {
		return;
	}

	ull_iso_pdu_rx_alloc();

	pdu_rx = (void *)node_rx->pdu;
	pdu_rx->ll_id = PDU_CIS_LLID_START_CONTINUE;
	pdu_rx->len = 0U;

	node_rx->hdr.type = NODE_RX_TYPE_ISO_PDU;
	node_rx->hdr.handle = cis_lll->handle;

	iso_meta = &node_rx->rx_iso_meta;
	iso_meta->payload_number = cis_lll->rx.payload_count + cis_lll->rx.bn_curr - 1U;
	iso_meta->timestamp = timestamp;
	cig_lll = ull_conn_iso_lll_group_get_by_stream(cis_lll);
	iso_meta->timestamp -= (cis_lll->event_count -
				(cis_lll->rx.payload_count / cis_lll->rx.bn)) *
			       cig_lll->iso_interval_us;
	iso_meta->timestamp %=
		HAL_TICKER_TICKS_TO_US_64BIT(BIT64(HAL_TICKER_CNTR_MSBIT + 1U));
	iso_meta->status = ISOAL_PDU_STATUS_LOST_DATA;

	iso_rx_put(node_rx->hdr.link, node_rx);
	iso_rx_sched();
}

static void iso_rx_data_lost(struct lll_conn_iso_stream *cis_lll);

static inline void lll_flush_rx_advance(struct lll_conn_iso_stream *cis_lll)
{
	iso_rx_data_lost(cis_lll);
	cis_lll->nesn++;
	cis_lll->rx.bn_curr++;
	if (cis_lll->rx.bn_curr > cis_lll->rx.bn) {
		cis_lll->rx.payload_count += cis_lll->rx.bn;
		cis_lll->rx.bn_curr = 1U;
	}
}

static inline void lll_flush_tx_advance(struct lll_conn_iso_stream *cis_lll)
{
	cis_lll->sn++;
	cis_lll->tx.bn_curr++;
	if (cis_lll->tx.bn_curr > cis_lll->tx.bn) {
		cis_lll->tx.payload_count += cis_lll->tx.bn;
		cis_lll->tx.bn_curr = 1U;
	}
}

#endif /* ZEPHYR_SUBSYS_BLUETOOTH_CONTROLLER_LL_SW_NORDIC_LLL_CONN_ISO_FLUSH_H_ */
