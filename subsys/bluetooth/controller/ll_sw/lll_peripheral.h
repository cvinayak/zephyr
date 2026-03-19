/*
 * Copyright (c) 2018-2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef BT_CTLR_LLL_ISR_CODE_RAM_ATTR
#if defined(CONFIG_BT_CTLR_LLL_ISR_CODE_IN_RAM)
#define BT_CTLR_LLL_ISR_CODE_RAM_ATTR __ramfunc
#else
#define BT_CTLR_LLL_ISR_CODE_RAM_ATTR
#endif
#endif /* BT_CTLR_LLL_ISR_CODE_RAM_ATTR */

int lll_periph_init(void);
int lll_periph_reset(void);
BT_CTLR_LLL_ISR_CODE_RAM_ATTR void lll_periph_prepare(void *param);
