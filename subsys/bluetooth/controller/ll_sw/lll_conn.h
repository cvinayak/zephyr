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

	u8_t  access_addr[4];
	u8_t  crc_init[3];
	u8_t  data_chan_map[5];
	u8_t  chm_update;

	u8_t  data_chan_count:6;
	u8_t  data_chan_sel:1;
	u8_t  role:1;

	union {
		struct {
			u8_t data_chan_hop;
			u8_t data_chan_use;
		};

		u16_t data_chan_id;
	};

	u16_t handle;
	u16_t event_counter;
	u16_t interval;
	u16_t latency;
	u16_t latency_prepare;
	u16_t latency_event;

#if defined(CONFIG_BT_CTLR_DATA_LENGTH)
	u16_t default_tx_octets;
	u16_t max_tx_octets;
	u16_t max_rx_octets;

#if defined(CONFIG_BT_CTLR_PHY)
	u16_t default_tx_time;
	u16_t max_tx_time;
	u16_t max_rx_time;
#endif /* CONFIG_BT_CTLR_PHY */
#endif /* CONFIG_BT_CTLR_DATA_LENGTH */

#if defined(CONFIG_BT_CTLR_PHY)
	u8_t phy_pref_tx:3;
	u8_t phy_tx:3;
	u8_t phy_pref_flags:1;
	u8_t phy_flags:1;
	u8_t phy_tx_time:3;

	u8_t phy_pref_rx:3;
	u8_t phy_rx:3;
#endif /* CONFIG_BT_CTLR_PHY */

	u16_t connect_expire;
	u16_t supervision_reload;
	u16_t supervision_expire;
	u16_t procedure_reload;
	u16_t procedure_expire;

#if defined(CONFIG_BT_CTLR_LE_PING)
	u16_t appto_reload;
	u16_t appto_expire;
	u16_t apto_reload;
	u16_t apto_expire;
#endif /* CONFIG_BT_CTLR_LE_PING */

	union {
		struct {
			u8_t reserved:5;
			u8_t fex_valid:1;
		} common;

		struct {
			u8_t terminate_ack:1;
			u8_t rfu:4;
			u8_t fex_valid:1;
		} master;

		struct {
			u8_t  latency_enabled:1;
			u8_t  latency_cancel:1;
			u8_t  sca:3;
			u8_t  fex_valid:1;
			u32_t window_widening_periodic_us;
			u32_t window_widening_max_us;
			u32_t window_widening_prepare_us;
			u32_t window_widening_event_us;
			u32_t window_size_prepare_us;
			u32_t window_size_event_us;
			u32_t force;
			u32_t ticks_to_offset;
		} slave;
	};

	u8_t  llcp_req;
	u8_t  llcp_ack;
	enum  llcp llcp_type;
	union {
		struct {
			enum {
				LLCP_CUI_STATE_INPROG,
				LLCP_CUI_STATE_USE,
				LLCP_CUI_STATE_SELECT
			} state:2 __packed;
			u8_t  is_internal:1;
			u16_t interval;
			u16_t latency;
			u16_t timeout;
			u16_t instant;
			u32_t win_offset_us;
			u8_t  win_size;
			u16_t *pdu_win_offset;
			u32_t ticks_anchor;
		} conn_upd;
		struct {
			u8_t  initiate;
			u8_t  chm[5];
			u16_t instant;
		} chan_map;

#if defined(CONFIG_BT_CTLR_PHY)
		struct {
			u8_t initiate:1;
			u8_t cmd:1;
			u8_t tx:3;
			u8_t rx:3;
			u16_t instant;
		} phy_upd_ind;
#endif /* CONFIG_BT_CTLR_PHY */

		struct {
			u8_t  initiate;
			u8_t  error_code;
			u8_t  rand[8];
			u8_t  ediv[2];
			u8_t  ltk[16];
			u8_t  skd[16];
		} encryption;
	} llcp;

	u32_t llcp_features;

	struct {
		u8_t  tx:1;
		u8_t  rx:1;
		u8_t  version_number;
		u16_t company_id;
		u16_t sub_version_number;
	} llcp_version;

	struct {
		u8_t req;
		u8_t ack;
		u8_t reason_own;
		u8_t reason_peer;
		struct {
			struct node_rx_hdr hdr;
			u8_t reason;
		} node_rx;
	} llcp_terminate;

#if defined(CONFIG_BT_CTLR_CONN_PARAM_REQ)
	struct {
		u8_t  req;
		u8_t  ack;
		enum {
			LLCP_CPR_STATE_REQ,
			LLCP_CPR_STATE_RSP,
			LLCP_CPR_STATE_APP_REQ,
			LLCP_CPR_STATE_APP_WAIT,
			LLCP_CPR_STATE_RSP_WAIT,
			LLCP_CPR_STATE_UPD
		} state:3 __packed;
		u8_t  cmd:1;
		u8_t  disabled:1;
		u8_t  status;
		u16_t interval;
		u16_t latency;
		u16_t timeout;
		u8_t  preferred_periodicity;
		u16_t reference_conn_event_count;
		u16_t offset0;
		u16_t offset1;
		u16_t offset2;
		u16_t offset3;
		u16_t offset4;
		u16_t offset5;
		u16_t *pdu_win_offset0;
		u32_t ticks_ref;
		u32_t ticks_to_offset_next;
	} llcp_conn_param;
#endif /* CONFIG_BT_CTLR_CONN_PARAM_REQ */

#if defined(CONFIG_BT_CTLR_DATA_LENGTH)
	struct {
		u8_t  req;
		u8_t  ack;
		u8_t  state:2;
#define LLCP_LENGTH_STATE_REQ        0
#define LLCP_LENGTH_STATE_ACK_WAIT   1
#define LLCP_LENGTH_STATE_RSP_WAIT   2
#define LLCP_LENGTH_STATE_RESIZE     3
		u16_t rx_octets;
		u16_t tx_octets;
#if defined(CONFIG_BT_CTLR_PHY)
		u16_t rx_time;
		u16_t tx_time;
#endif /* CONFIG_BT_CTLR_PHY */
	} llcp_length;
#endif /* CONFIG_BT_CTLR_DATA_LENGTH */

#if defined(CONFIG_BT_CTLR_PHY)
	struct {
		u8_t req;
		u8_t ack;
		u8_t state:2;
#define LLCP_PHY_STATE_REQ      0
#define LLCP_PHY_STATE_ACK_WAIT 1
#define LLCP_PHY_STATE_RSP_WAIT 2
#define LLCP_PHY_STATE_UPD      3
		u8_t tx:3;
		u8_t rx:3;
		u8_t flags:1;
		u8_t cmd:1;
	} llcp_phy;
#endif /* CONFIG_BT_CTLR_PHY */

	u8_t  sn:1;
	u8_t  nesn:1;
	u8_t  pause_rx:1;
	u8_t  pause_tx:1;
	u8_t  enc_rx:1;
	u8_t  enc_tx:1;
	u8_t  refresh:1;
	u8_t  empty:1;

	struct ccm ccm_rx;
	struct ccm ccm_tx;

	MEMQ_DECLARE(tx);
	memq_link_t link_tx;
	memq_link_t *link_tx_free;
	u8_t  packet_tx_head_len;
	u8_t  packet_tx_head_offset;

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
