/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stddef.h>
#include <zephyr.h>

#include "hal/ccm.h"

#include "util/mem.h"
#include "util/memq.h"

#include "ull_types.h"

#include "pdu.h"
#include "lll.h"
#include "lll_conn.h"
#include "ull_conn_types.h"

static int _init_reset(void);

static struct ll_conn _conn[CONFIG_BT_MAX_CONN];
static void *_conn_free;

struct ll_conn *ll_conn_acquire(void)
{
	return mem_acquire(&_conn_free);
}

void ll_conn_release(struct ll_conn *conn)
{
	mem_release(conn, &_conn_free);
}


void *ll_tx_mem_acquire(void)
{
	return NULL;
}

void ll_tx_mem_release(void *node_tx)
{
}

u8_t ll_tx_mem_enqueue(u16_t handle, void *node_tx)
{
	return 0;
}

u8_t ll_terminate_ind_send(u16_t handle, u8_t reason)
{
	return 0;
}

u8_t ll_version_ind_send(u16_t handle)
{
	return 0;
}

u8_t ll_feature_req_send(u16_t handle)
{
	return 0;
}

u8_t ll_chm_get(u16_t handle, u8_t *chm)
{
	return 0;
}

u8_t ll_conn_update(u16_t handle, u8_t cmd, u8_t status, u16_t interval,
		     u16_t latency, u16_t timeout)
{
	return 0;
}

#if defined(CONFIG_BT_CTLR_LE_ENC)
#if defined(CONFIG_BT_PERIPHERAL)
u8_t ll_start_enc_req_send(u16_t handle, u8_t error_code,
			    u8_t const *const ltk)
{
	return 0;
}
#endif /* CONFIG_BT_PERIPHERAL */
#endif /* CONFIG_BT_CTLR_LE_ENC */

int ull_conn_init(void)
{
	int err;

	err = _init_reset();
	if (err) {
		return err;
	}

	return 0;
}

int ull_conn_reset(void)
{
	int err;

	err = _init_reset();
	if (err) {
		return err;
	}

	return 0;
}

static int _init_reset(void)
{
	mem_init(_conn, sizeof(struct ll_conn),
		 sizeof(_conn)/sizeof(struct ll_conn), &_conn_free);

	return 0;
}
