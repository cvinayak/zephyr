/*
 * Copyright (c) 2017-2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* NOTE: Definitions used internal to ULL implementations */

/* Macro to convert time in us to connection interval units */
#define RADIO_CONN_EVENTS(x, y) ((u16_t)(((x) + (y) - 1) / (y)))

struct ll_conn *ll_conn_acquire(void);
void ll_conn_release(struct ll_conn *conn);
u16_t ll_conn_handle_get(struct ll_conn *conn);
struct ll_conn *ll_conn_get(u16_t handle);
int ull_conn_init(void);
int ull_conn_reset(void);
u32_t ull_conn_ppm_local_get(void);
u32_t ull_conn_ppm_get(u8_t sca);
void ull_conn_setup(memq_link_t *link, struct node_rx_hdr *rx);
void ull_conn_done(struct node_rx_event_done *done);
void ull_conn_tx_demux(u8_t count);
