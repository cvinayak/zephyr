/*
 * Copyright (c) 2018-2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef LLL_ISR_CODE_RAM_ATTR
#if defined(CONFIG_BT_CTLR_ISR_CODE_IN_RAM)
#define LLL_ISR_CODE_RAM_ATTR __ramfunc
#else
#define LLL_ISR_CODE_RAM_ATTR
#endif
#endif /* LLL_ISR_CODE_RAM_ATTR */

int lll_central_init(void);
int lll_central_reset(void);
LLL_ISR_CODE_RAM_ATTR void lll_central_prepare(void *param);
