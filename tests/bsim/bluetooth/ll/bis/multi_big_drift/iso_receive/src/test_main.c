/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/sys/clock.h>
#include <zephyr/sys/util.h>

#include "bs_types.h"
#include "bs_tracing.h"
#include "bstests.h"

#include "iso_rx_stats.h"

#define TICK_PERIOD_US       (1U * USEC_PER_SEC)
#define REPORT_PERIOD_TICKS  10U
#define SDU_INTERVAL_US      10000U
#define RX_CNT_MARGIN        2U
#define DURATION_S_DEFAULT   60U

extern enum bst_result_t bst_result;

#define FAIL(...)                                       \
	do {                                            \
		bst_result = Failed;                    \
		bs_trace_error_time_line(__VA_ARGS__);  \
	} while (0)

#define PASS(...)                                       \
	do {                                            \
		bst_result = Passed;                    \
		bs_trace_info_time(1, __VA_ARGS__);     \
	} while (0)

static uint32_t duration_s = DURATION_S_DEFAULT;
static struct iso_rx_stats stats_base[ISO_CHAN_COUNT];
static bs_time_t synced_time_us;
static bool synced;
static uint32_t tick_count;

static bool all_connected(void)
{
	for (size_t i = 0U; i < ISO_CHAN_COUNT; i++) {
		if (!iso_rx_stats[i].connected) {
			return false;
		}
	}

	return true;
}

static void stats_report(bs_time_t now_us, uint8_t level)
{
	for (size_t i = 0U; i < ISO_CHAN_COUNT; i++) {
		const struct iso_rx_stats *s = &iso_rx_stats[i];
		const struct iso_rx_stats *b = &stats_base[i];

		bs_trace_info_time(level,
				   "BIG %zu BIS %zu: connected %u rx %u lost %u seq_gap %u "
				   "(since sync: rx %u lost %u seq_gap %u, %" PRIu64 " us)\n",
				   i / BIS_ISO_CHAN_COUNT, i % BIS_ISO_CHAN_COUNT, s->connected,
				   s->rx_cnt, s->lost_cnt, s->seq_gap_cnt, s->rx_cnt - b->rx_cnt,
				   s->lost_cnt - b->lost_cnt, s->seq_gap_cnt - b->seq_gap_cnt,
				   synced ? (now_us - synced_time_us) : 0U);
	}
}

static void test_args(int argc, char *argv[])
{
	for (int i = 0; i < argc; i++) {
		if ((strcmp(argv[i], "duration_s") == 0) && ((i + 1) < argc)) {
			duration_s = strtoul(argv[++i], NULL, 0);
		}
	}
}

static void test_init(void)
{
	bst_ticker_set_period(TICK_PERIOD_US);
	bst_result = In_progress;
}

static void test_tick(bs_time_t now_us)
{
	bool pass = true;
	uint32_t rx_min;

	tick_count++;

	if (!synced) {
		if (all_connected()) {
			synced = true;
			synced_time_us = now_us;
			(void)memcpy(stats_base, iso_rx_stats, sizeof(stats_base));
			bs_trace_info_time(1, "All %u BIS synchronized, measuring for %u s\n",
					   ISO_CHAN_COUNT, duration_s);
		} else if (now_us > (bs_time_t)(duration_s * 2U) * USEC_PER_SEC) {
			stats_report(now_us, 0U);
			FAIL("iso_receive FAILED: not all BIS synchronized\n");
		}

		return;
	}

	if ((tick_count % REPORT_PERIOD_TICKS) == 0U) {
		stats_report(now_us, 2U);
	}

	if ((now_us - synced_time_us) < ((bs_time_t)duration_s * USEC_PER_SEC)) {
		return;
	}

	stats_report(now_us, 0U);

	rx_min = ((now_us - synced_time_us) / SDU_INTERVAL_US) - RX_CNT_MARGIN;
	for (size_t i = 0U; i < ISO_CHAN_COUNT; i++) {
		const struct iso_rx_stats *s = &iso_rx_stats[i];
		const struct iso_rx_stats *b = &stats_base[i];

		if (!s->connected || (s->lost_cnt != b->lost_cnt) ||
		    (s->seq_gap_cnt != b->seq_gap_cnt) || ((s->rx_cnt - b->rx_cnt) < rx_min)) {
			pass = false;
		}
	}

	if (pass) {
		PASS("iso_receive PASSED\n");
		bs_trace_exit("Done, disconnecting from simulation\n");
	} else {
		FAIL("iso_receive FAILED: SDUs missed (expected >= %u per BIS)\n", rx_min);
	}
}

static const struct bst_test_instance test_def[] = {
	{
		.test_id = "iso_receive",
		.test_descr = "iso_receive sample synchronized to multiple drifting iso_broadcast "
			      "samples; after all BIS are synchronized, checks that no SDU is "
			      "lost for duration_s seconds (default " STRINGIFY(DURATION_S_DEFAULT)
			      ")",
		.test_args_f = test_args,
		.test_pre_init_f = test_init,
		.test_tick_f = test_tick,
	},
	BSTEST_END_MARKER
};

static struct bst_test_list *test_iso_receive_install(struct bst_test_list *tests)
{
	return bst_add_tests(tests, test_def);
}

bst_test_install_t test_installers[] = {
	test_iso_receive_install,
	NULL
};
