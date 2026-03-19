/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
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

int lll_adv_sync_init(void);
int lll_adv_sync_reset(void);
BT_CTLR_LLL_ISR_CODE_RAM_ATTR void lll_adv_sync_prepare(void *param);

extern uint16_t ull_adv_sync_lll_handle_get(const struct lll_adv_sync *lll);

extern void ull_adv_sync_lll_syncinfo_fill(struct pdu_adv *pdu, struct lll_adv_aux *lll_aux);
