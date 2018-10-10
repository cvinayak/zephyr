/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* NOTE: Definitions used between Thread and ULL/LLL implementations */

struct ll_conn {
	struct evt_hdr  evt;
	struct ull_hdr  ull;
	struct lll_conn lll;

	u16_t connect_expire;
	u16_t supervision_reload;
	u16_t supervision_expire;
	u16_t procedure_reload;
	u16_t procedure_expire;

	u8_t llcp_req;
	u8_t llcp_ack;
	u8_t llcp_type;

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

	u8_t  pause_tx:1;

	struct node_tx *tx_head;
	struct node_tx *tx_ctrl;
	struct node_tx *tx_ctrl_last;
	struct node_tx *tx_data;
	struct node_tx *tx_data_last;
};

struct node_rx_cc {
	u8_t  status;
	u8_t  role;
	u8_t  peer_addr_type;
	u8_t  peer_addr[BDADDR_SIZE];
#if defined(CONFIG_BT_CTLR_PRIVACY)
	u8_t  peer_rpa[BDADDR_SIZE];
	u8_t  own_addr_type;
	u8_t  own_addr[BDADDR_SIZE];
#endif /* CONFIG_BT_CTLR_PRIVACY */
	u16_t interval;
	u16_t latency;
	u16_t timeout;
	u8_t  sca;
};

struct node_rx_cu {
	u8_t  status;
	u16_t interval;
	u16_t latency;
	u16_t timeout;
};

struct node_rx_cs {
	u8_t csa;
};

struct node_rx_pu {
	u8_t status;
	u8_t tx;
	u8_t rx;
};
