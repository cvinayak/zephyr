/*
 * Copyright (c) 2021 Demant
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

int lll_peripheral_iso_init(void);
int lll_peripheral_iso_reset(void);
LLL_ISR_CODE_RAM_ATTR void lll_peripheral_iso_prepare(void *param);
