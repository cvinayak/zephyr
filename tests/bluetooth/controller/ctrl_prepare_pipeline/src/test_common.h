/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_TESTS_BT_CTRL_PREPARE_PIPELINE_H_
#define ZEPHYR_TESTS_BT_CTRL_PREPARE_PIPELINE_H_

/**
 * @brief Initialize the prepare pipeline for testing
 *
 * Must be called before any prepare pipeline operations.
 */
void ull_prepare_pipeline_init(void);

/**
 * @brief Cleanup the prepare pipeline after testing
 *
 * Frees all allocated events and resets the pipeline.
 */
void ull_prepare_pipeline_cleanup(void);

#endif /* ZEPHYR_TESTS_BT_CTRL_PREPARE_PIPELINE_H_ */
