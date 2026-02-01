/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Comprehensive unit tests for ULL prepare pipeline (PR #79444)
 *
 * ⚠️ IMPORTANT: These tests are written for the FUTURE implementation
 * described in PR #79444, which replaces MFIFO with an ordered linked list.
 *
 * Current implementation uses MFIFO; these tests will need adaptation
 * when PR #79444 is merged.
 *
 * This test suite validates the ordered linked list implementation that
 * replaces the FIFO-based prepare pipeline in PR #79444.
 *
 * Key changes tested:
 * - Data structure change from MFIFO to ordered linked list
 * - Iterator interface change from uint8_t* to void**
 * - Ordered insertion based on ticks_at_expire
 * - Resume events always placed at tail
 */

#include <string.h>
#include <zephyr/types.h>
#include <zephyr/ztest.h>
#include <zephyr/kernel.h>

#include "util/util.h"
#include "util/mem.h"
#include "util/memq.h"
#include "util/mayfly.h"

#include "lll.h"
#include "ull_internal.h"

/* External test suite declarations */
extern struct ztest_suite_node test_ull_prepare_basic_suite_node;
extern struct ztest_suite_node test_ull_prepare_ordering_suite_node;
extern struct ztest_suite_node test_ull_prepare_iterator_suite_node;
extern struct ztest_suite_node test_ull_prepare_edge_cases_suite_node;

void main(void)
{
	ztest_run_all(NULL, false, 1, 1);
}
