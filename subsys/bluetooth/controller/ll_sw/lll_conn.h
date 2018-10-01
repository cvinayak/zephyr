/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* NOTE: Definitions used between ULL and LLL implementations */

#define LLL_CONN_RSSI_SAMPLE_COUNT 10
#define LLL_CONN_RSSI_THRESHOLD    4

struct lll_tx {
	u16_t handle;
	void *node;
};

struct node_tx {
	union {
		void        *next;
		void        *pool;
		memq_link_t *link;
	};

	u8_t pdu[];
};

enum llcp {
	LLCP_NONE,
	LLCP_CONN_UPD,
	LLCP_CHAN_MAP,

#if defined(CONFIG_BT_CTLR_LE_ENC)
	LLCP_ENCRYPTION,
#endif /* CONFIG_BT_CTLR_LE_ENC */

	LLCP_FEATURE_EXCHANGE,
	LLCP_VERSION_EXCHANGE,
	/* LLCP_TERMINATE, */
	LLCP_CONNECTION_PARAM_REQ,

#if defined(CONFIG_BT_CTLR_LE_PING)
	LLCP_PING,
#endif /* CONFIG_BT_CTLR_LE_PING */

#if defined(CONFIG_BT_CTLR_PHY)
	LLCP_PHY_UPD,
#endif /* CONFIG_BT_CTLR_PHY */
};

struct lll_conn {
	struct lll_hdr hdr;

	u8_t access_addr[4];
	u8_t crc_init[3];

	u16_t handle;
	u16_t interval;
	u16_t latency;

	/* FIXME: BEGIN: Move to ULL? */
	u16_t latency_prepare;
	u16_t latency_event;

	u16_t event_counter;
	u8_t data_chan_map[5];
	u8_t data_chan_count:6;
	u8_t data_chan_sel:1;
	u8_t role:1;

	union {
		struct {
			u8_t data_chan_hop;
			u8_t data_chan_use;
		};

		u16_t data_chan_id;
	};

	union {
		struct {
			u8_t  latency_enabled:1;
			u8_t  latency_cancel:1;
			u8_t  sca:3;
			u32_t window_widening_periodic_us;
			u32_t window_widening_max_us;
			u32_t window_widening_prepare_us;
			u32_t window_widening_event_us;
			u32_t window_size_prepare_us;
			u32_t window_size_event_us;
		} slave;
	};
	/* FIXME: END: Move to ULL? */

	MEMQ_DECLARE(tx);
	memq_link_t link_tx;
	memq_link_t *link_tx_free;
	u8_t  packet_tx_head_len;
	u8_t  packet_tx_head_offset;

	struct ccm ccm_rx;
	struct ccm ccm_tx;

	u8_t sn:1;
	u8_t nesn:1;
	u8_t enc_rx:1;
	u8_t enc_tx:1;
	u8_t empty:1;

#if defined(CONFIG_BT_CTLR_CONN_RSSI)
	u8_t  rssi_latest;
	u8_t  rssi_reported;
	u8_t  rssi_sample_count;
#endif /* CONFIG_BT_CTLR_CONN_RSSI */
};

int lll_conn_init(void);
int lll_conn_reset(void);
u8_t lll_conn_sca_local_get(void);
u32_t lll_conn_ppm_local_get(void);
u32_t lll_conn_ppm_get(u8_t sca);
void lll_conn_prepare_reset(void);
int lll_conn_is_abort_cb(void *next, int prio, void *curr,
			 lll_prepare_cb_t *resume_cb, int *resume_prio);
void lll_conn_abort_cb(struct lll_prepare_param *prepare_param, void *param);
void lll_conn_isr_rx(void *param);
void lll_conn_isr_tx(void *param);
void lll_conn_isr_abort(void *param);
void lll_conn_rx_pkt_set(struct lll_conn *lll);
void lll_conn_tx_pkt_set(struct lll_conn *lll, struct pdu_data *pdu_data_tx);
void lll_conn_pdu_tx_prep(struct lll_conn *lll, struct pdu_data **pdu_data_tx);
u8_t lll_conn_ack_last_idx_get(void);
memq_link_t *lll_conn_ack_peek(u8_t *ack_last, u16_t *handle,
			       struct node_tx **node_tx);
memq_link_t *lll_conn_ack_by_last_peek(u8_t last, u16_t *handle,
				       struct node_tx **node_tx);
void *lll_conn_ack_dequeue(void);
void lll_conn_tx_flush(void *param);
