/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ISO_RX_STATS_H_
#define ISO_RX_STATS_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/autoconf.h>
#include <zephyr/sys/util.h>

#define BIG_SYNC_COUNT     MIN(CONFIG_BT_ISO_MAX_BIG, CONFIG_BT_PER_ADV_SYNC_MAX)
#define BIS_ISO_CHAN_COUNT MIN(2U, (CONFIG_BT_ISO_MAX_CHAN / BIG_SYNC_COUNT))
#define ISO_CHAN_COUNT     (BIG_SYNC_COUNT * BIS_ISO_CHAN_COUNT)

/* Per BIS reception statistics */
struct iso_rx_stats {
	uint32_t rx_cnt;
	uint32_t lost_cnt;
	uint32_t seq_gap_cnt;
	uint16_t seq_num_last;
	bool     connected;
	bool     seq_num_valid;
};

extern struct iso_rx_stats iso_rx_stats[ISO_CHAN_COUNT];

#endif /* ISO_RX_STATS_H_ */
