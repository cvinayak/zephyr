/*
 * Copyright (c) 2016 Nordic Semiconductor ASA
 * Copyright (c) 2016 Vinayak Kariappa Chettimada
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "hal/cntr_vendor_hal.h"

void cntr_init(void);
uint32_t cntr_start(void);
uint32_t cntr_stop(void);
uint32_t cntr_cnt_get(void);
void cntr_cmp_set(uint8_t cmp, uint32_t value);

#if defined(CONFIG_BT_CTLR_PREEMPT_TIMEOUT_CNTR)
void cntr_preempt_cmp_set(uint32_t value);
void cntr_preempt_cmp_clear(void);
#endif /* CONFIG_BT_CTLR_PREEMPT_TIMEOUT_CNTR */
