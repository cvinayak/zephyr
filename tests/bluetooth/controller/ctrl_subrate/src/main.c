/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <zephyr/ztest.h>

#define ULL_LLCP_UNITTEST

#include <zephyr/bluetooth/hci.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/slist.h>
#include <zephyr/sys/util.h>
#include "hal/ccm.h"

#include "util/util.h"
#include "util/mem.h"
#include "util/memq.h"
#include "util/dbuf.h"

#include "pdu_df.h"
#include "lll/pdu_vendor.h"
#include "pdu.h"
#include "ll.h"
#include "ll_settings.h"

#include "lll.h"
#include "lll/lll_df_types.h"
#include "lll_conn.h"
#include "lll_conn_iso.h"

#include "ull_tx_queue.h"

#include "isoal.h"
#include "ull_iso_types.h"
#include "ull_conn_iso_types.h"
#include "ull_conn_types.h"
#include "ull_llcp.h"
#include "ull_conn_internal.h"
#include "ull_llcp_internal.h"

#include "helper_pdu.h"
#include "helper_util.h"

/* Default subrating parameters */
#define SUBRATE_FACTOR_MIN  1U
#define SUBRATE_FACTOR_MAX  4U
#define MAX_LATENCY         0U
#define CONTINUATION_NUMBER 0U
#define SUPERVISION_TIMEOUT 100U /* multiple of 10 ms */

/* Default LL_SUBRATE_REQ PDU */
struct pdu_data_llctrl_subrate_req subrate_req = {
	.subrate_factor_min = sys_cpu_to_le16(SUBRATE_FACTOR_MIN),
	.subrate_factor_max = sys_cpu_to_le16(SUBRATE_FACTOR_MAX),
	.max_latency = sys_cpu_to_le16(MAX_LATENCY),
	.continuation_number = sys_cpu_to_le16(CONTINUATION_NUMBER),
	.supervision_timeout = sys_cpu_to_le16(SUPERVISION_TIMEOUT)
};

/* Default LL_SUBRATE_IND PDU */
struct pdu_data_llctrl_subrate_ind subrate_ind = {
	.subrate_factor = sys_cpu_to_le16(SUBRATE_FACTOR_MIN),
	.subrate_base_event = sys_cpu_to_le16(6U),
	.latency = sys_cpu_to_le16(MAX_LATENCY),
	.continuation_number = sys_cpu_to_le16(CONTINUATION_NUMBER),
	.supervision_timeout = sys_cpu_to_le16(SUPERVISION_TIMEOUT)
};

static struct ll_conn conn;

static void subrate_setup(void *data)
{
	test_setup(&conn);

	/* Initialize lll conn parameters */
	struct lll_conn *lll = &conn.lll;

	lll->interval = 6U; /* 7.5ms */
	lll->latency = 0U;
	conn.supervision_timeout = 100U;
	lll->event_counter = 0U;

#if defined(CONFIG_BT_CTLR_SUBRATING)
	/* Initialize subrate parameters */
	conn.subrate_factor = 1U;
	conn.subrate_base_event = 0U;
	conn.continuation_number = 0U;
#endif /* CONFIG_BT_CTLR_SUBRATING */
}

static bool is_instant_reached(struct ll_conn *llconn, uint16_t instant)
{
	return ((event_counter(llconn) - instant) & 0xFFFF) <= 0x7FFF;
}

/*
 * Central-initiated Connection Subrating procedure.
 * Central requests subrating, peripheral accepts.
 *
 * +-----+                    +-------+                    +-----+
 * | UT  |                    | LL_C  |                    | LT  |
 * +-----+                    +-------+                    +-----+
 *    |                           |                           |
 *    | LE Subrate Request        |                           |
 *    |-------------------------->|                           |
 *    |                           | LL_SUBRATE_REQ            |
 *    |                           |-------------------------->|
 *    |                           |                           |
 *    |                           |        LL_SUBRATE_IND     |
 *    |                           |<--------------------------|
 *    |                           |                           |
 *    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *    |                           |                           |
 *    |      LE Subrate Change    |                           |
 *    |                  Complete |                           |
 *    |<--------------------------|                           |
 *    |                           |                           |
 */
ZTEST(central_loc, test_subrate_central_loc_accept)
{
	uint8_t err;
	struct node_tx *tx;
	struct node_rx_pdu *ntf;
	uint16_t base_event;

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_CENTRAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Initiate a Subrate Request Procedure */
	err = ll_subrate_req(conn.lll.handle, SUBRATE_FACTOR_MIN, SUBRATE_FACTOR_MAX,
			    MAX_LATENCY, CONTINUATION_NUMBER, SUPERVISION_TIMEOUT);
	zassert_equal(err, BT_HCI_ERR_SUCCESS);

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should have one LL Control PDU */
	lt_rx(LL_SUBRATE_REQ, &conn, &tx, &subrate_req);
	lt_rx_q_is_empty(&conn);

	/* Done */
	event_done(&conn);

	/* Release Tx */
	ull_cp_release_tx(&conn, tx);

	/* Prepare */
	event_prepare(&conn);

	/* Rx */
	subrate_ind.subrate_base_event = sys_cpu_to_le16(event_counter(&conn) + 6U);
	lt_tx(LL_SUBRATE_IND, &conn, &subrate_ind);

	/* Done */
	event_done(&conn);

	/* Save base event */
	base_event = sys_le16_to_cpu(subrate_ind.subrate_base_event);

	/* Wait for instant */
	while (!is_instant_reached(&conn, base_event)) {
		/* Prepare */
		event_prepare(&conn);

		/* Tx Queue should NOT have a LL Control PDU */
		lt_rx_q_is_empty(&conn);

		/* Done */
		event_done(&conn);
	}

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should NOT have a LL Control PDU */
	lt_rx_q_is_empty(&conn);

	/* Done */
	event_done(&conn);

	/* There should be one notification */
	ut_rx_pdu(LL_SUBRATE_IND, &ntf, &subrate_ind);
	ut_rx_q_is_empty();

	/* Verify subrate parameters applied */
	zassert_equal(conn.subrate_factor, SUBRATE_FACTOR_MIN);
	zassert_equal(conn.subrate_base_event, base_event);
	zassert_equal(conn.continuation_number, CONTINUATION_NUMBER);

	/* Release Ntf */
	release_ntf(ntf);

	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(),
		      "Free CTX buffers %d", llcp_ctx_buffers_free());
}

/*
 * Peripheral-initiated Connection Subrating procedure.
 * Peripheral requests subrating, central accepts.
 *
 * +-----+                    +-------+                    +-----+
 * | UT  |                    | LL_P  |                    | LT  |
 * +-----+                    +-------+                    +-----+
 *    |                           |                           |
 *    | LE Subrate Request        |                           |
 *    |-------------------------->|                           |
 *    |                           | LL_SUBRATE_REQ            |
 *    |                           |-------------------------->|
 *    |                           |                           |
 *    |                           |        LL_SUBRATE_IND     |
 *    |                           |<--------------------------|
 *    |                           |                           |
 *    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *    |                           |                           |
 *    |      LE Subrate Change    |                           |
 *    |                  Complete |                           |
 *    |<--------------------------|                           |
 *    |                           |                           |
 */
ZTEST(periph_loc, test_subrate_periph_loc_accept)
{
	uint8_t err;
	struct node_tx *tx;
	struct node_rx_pdu *ntf;
	uint16_t base_event;

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_PERIPHERAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Initiate a Subrate Request Procedure */
	err = ll_subrate_req(conn.lll.handle, SUBRATE_FACTOR_MIN, SUBRATE_FACTOR_MAX,
			    MAX_LATENCY, CONTINUATION_NUMBER, SUPERVISION_TIMEOUT);
	zassert_equal(err, BT_HCI_ERR_SUCCESS);

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should have one LL Control PDU */
	lt_rx(LL_SUBRATE_REQ, &conn, &tx, &subrate_req);
	lt_rx_q_is_empty(&conn);

	/* Done */
	event_done(&conn);

	/* Release Tx */
	ull_cp_release_tx(&conn, tx);

	/* Prepare */
	event_prepare(&conn);

	/* Rx */
	subrate_ind.subrate_base_event = sys_cpu_to_le16(event_counter(&conn) + 6U);
	lt_tx(LL_SUBRATE_IND, &conn, &subrate_ind);

	/* Done */
	event_done(&conn);

	/* Save base event */
	base_event = sys_le16_to_cpu(subrate_ind.subrate_base_event);

	/* Wait for instant */
	while (!is_instant_reached(&conn, base_event)) {
		/* Prepare */
		event_prepare(&conn);

		/* Tx Queue should NOT have a LL Control PDU */
		lt_rx_q_is_empty(&conn);

		/* Done */
		event_done(&conn);
	}

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should NOT have a LL Control PDU */
	lt_rx_q_is_empty(&conn);

	/* Done */
	event_done(&conn);

	/* There should be one notification */
	ut_rx_pdu(LL_SUBRATE_IND, &ntf, &subrate_ind);
	ut_rx_q_is_empty();

	/* Verify subrate parameters applied */
	zassert_equal(conn.subrate_factor, SUBRATE_FACTOR_MIN);
	zassert_equal(conn.subrate_base_event, base_event);
	zassert_equal(conn.continuation_number, CONTINUATION_NUMBER);

	/* Release Ntf */
	release_ntf(ntf);

	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(),
		      "Free CTX buffers %d", llcp_ctx_buffers_free());
}

/*
 * Central receives subrating request from peripheral.
 * Central responds with LL_SUBRATE_IND.
 *
 * +-----+                    +-------+                    +-----+
 * | UT  |                    | LL_C  |                    | LT  |
 * +-----+                    +-------+                    +-----+
 *    |                           |                           |
 *    |                           |      LL_SUBRATE_REQ       |
 *    |                           |<--------------------------|
 *    |                           |                           |
 *    |                           | LL_SUBRATE_IND            |
 *    |                           |-------------------------->|
 *    |                           |                           |
 *    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *    |                           |                           |
 *    |      LE Subrate Change    |                           |
 *    |                  Complete |                           |
 *    |<--------------------------|                           |
 *    |                           |                           |
 */
ZTEST(central_rem, test_subrate_central_rem_accept)
{
	struct node_tx *tx;
	struct node_rx_pdu *ntf;
	uint16_t base_event;

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_CENTRAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Prepare */
	event_prepare(&conn);

	/* Rx */
	lt_tx(LL_SUBRATE_REQ, &conn, &subrate_req);

	/* Done */
	event_done(&conn);

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should have one LL Control PDU */
	subrate_ind.subrate_base_event = sys_cpu_to_le16(event_counter(&conn) + 6U);
	lt_rx(LL_SUBRATE_IND, &conn, &tx, &subrate_ind);
	lt_rx_q_is_empty(&conn);

	/* Done */
	event_done(&conn);

	/* Save base event */
	base_event = sys_le16_to_cpu(subrate_ind.subrate_base_event);

	/* Release Tx */
	ull_cp_release_tx(&conn, tx);

	/* Wait for instant */
	while (!is_instant_reached(&conn, base_event)) {
		/* Prepare */
		event_prepare(&conn);

		/* Tx Queue should NOT have a LL Control PDU */
		lt_rx_q_is_empty(&conn);

		/* Done */
		event_done(&conn);
	}

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should NOT have a LL Control PDU */
	lt_rx_q_is_empty(&conn);

	/* Done */
	event_done(&conn);

	/* There should be one notification */
	ut_rx_pdu(LL_SUBRATE_IND, &ntf, &subrate_ind);
	ut_rx_q_is_empty();

	/* Verify subrate parameters applied */
	zassert_equal(conn.subrate_factor, SUBRATE_FACTOR_MIN);
	zassert_equal(conn.subrate_base_event, base_event);
	zassert_equal(conn.continuation_number, CONTINUATION_NUMBER);

	/* Release Ntf */
	release_ntf(ntf);

	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(),
		      "Free CTX buffers %d", llcp_ctx_buffers_free());
}

/*
 * Peripheral receives subrating request from central.
 * Peripheral responds with LL_SUBRATE_IND.
 *
 * +-----+                    +-------+                    +-----+
 * | UT  |                    | LL_P  |                    | LT  |
 * +-----+                    +-------+                    +-----+
 *    |                           |                           |
 *    |                           |      LL_SUBRATE_REQ       |
 *    |                           |<--------------------------|
 *    |                           |                           |
 *    |                           | LL_SUBRATE_IND            |
 *    |                           |-------------------------->|
 *    |                           |                           |
 *    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *    |                           |                           |
 *    |      LE Subrate Change    |                           |
 *    |                  Complete |                           |
 *    |<--------------------------|                           |
 *    |                           |                           |
 */
ZTEST(periph_rem, test_subrate_periph_rem_accept)
{
	struct node_tx *tx;
	struct node_rx_pdu *ntf;
	uint16_t base_event;

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_PERIPHERAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Prepare */
	event_prepare(&conn);

	/* Rx */
	lt_tx(LL_SUBRATE_REQ, &conn, &subrate_req);

	/* Done */
	event_done(&conn);

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should have one LL Control PDU */
	subrate_ind.subrate_base_event = sys_cpu_to_le16(event_counter(&conn) + 6U);
	lt_rx(LL_SUBRATE_IND, &conn, &tx, &subrate_ind);
	lt_rx_q_is_empty(&conn);

	/* Done */
	event_done(&conn);

	/* Save base event */
	base_event = sys_le16_to_cpu(subrate_ind.subrate_base_event);

	/* Release Tx */
	ull_cp_release_tx(&conn, tx);

	/* Wait for instant */
	while (!is_instant_reached(&conn, base_event)) {
		/* Prepare */
		event_prepare(&conn);

		/* Tx Queue should NOT have a LL Control PDU */
		lt_rx_q_is_empty(&conn);

		/* Done */
		event_done(&conn);
	}

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should NOT have a LL Control PDU */
	lt_rx_q_is_empty(&conn);

	/* Done */
	event_done(&conn);

	/* There should be one notification */
	ut_rx_pdu(LL_SUBRATE_IND, &ntf, &subrate_ind);
	ut_rx_q_is_empty();

	/* Verify subrate parameters applied */
	zassert_equal(conn.subrate_factor, SUBRATE_FACTOR_MIN);
	zassert_equal(conn.subrate_base_event, base_event);
	zassert_equal(conn.continuation_number, CONTINUATION_NUMBER);

	/* Release Ntf */
	release_ntf(ntf);

	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(),
		      "Free CTX buffers %d", llcp_ctx_buffers_free());
}

ZTEST_SUITE(central_loc, NULL, NULL, subrate_setup, NULL, NULL);
ZTEST_SUITE(central_rem, NULL, NULL, subrate_setup, NULL, NULL);
ZTEST_SUITE(periph_loc, NULL, NULL, subrate_setup, NULL, NULL);
ZTEST_SUITE(periph_rem, NULL, NULL, subrate_setup, NULL, NULL);
