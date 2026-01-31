/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/hci_types.h>

#include "util/util.h"
#include "util/memq.h"
#include "util/dbuf.h"

#include "hal/ccm.h"

#include "pdu.h"

#include "lll.h"
#include "lll_conn.h"

#include "ull_tx_queue.h"

#include "ull_conn_types.h"
#include "ull_internal.h"
#include "ull_conn_internal.h"

#include "ull_llcp.h"
#include "ull_llcp_internal.h"

#include "ull_cs_types.h"
#include "ull_cs_internal.h"

#include <zephyr/bluetooth/hci.h>

#include "hal/debug.h"

#define CS_PROC_STATE_IDLE 0

void llcp_cs_setup(struct ll_conn *conn)
{
	if (IS_ENABLED(CONFIG_BT_CTLR_CHANNEL_SOUNDING)) {
		memset(&conn->llcp.cs, 0, sizeof(conn->llcp.cs));
	}
}

void llcp_cs_tx_req(struct ll_conn *conn)
{
}

void llcp_cs_tx_rsp(struct ll_conn *conn)
{
}

void llcp_cs_tx_ind(struct ll_conn *conn)
{
}

void llcp_cs_rx(struct ll_conn *conn, struct pdu_data *pdu)
{
	uint8_t opcode = pdu->llctrl.opcode;

	switch (opcode) {
	case PDU_DATA_LLCTRL_TYPE_CS_REQ:
		break;
	case PDU_DATA_LLCTRL_TYPE_CS_RSP:
		break;
	case PDU_DATA_LLCTRL_TYPE_CS_IND:
		break;
	case PDU_DATA_LLCTRL_TYPE_CS_TERMINATE_REQ:
		break;
	case PDU_DATA_LLCTRL_TYPE_CS_TERMINATE_RSP:
		break;
	case PDU_DATA_LLCTRL_TYPE_CS_FAE_REQ:
		break;
	case PDU_DATA_LLCTRL_TYPE_CS_FAE_RSP:
		break;
	default:
		break;
	}
}
