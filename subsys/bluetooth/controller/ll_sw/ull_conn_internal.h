/*
 * Copyright (c) 2017-2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* NOTE: Definitions used internal to ULL implementations */

int ull_conn_init(void);
int ull_conn_reset(void);
struct ll_conn *ll_conn_acquire(void);
void ll_conn_release(struct ll_conn *conn);
