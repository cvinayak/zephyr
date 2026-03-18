/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
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

int lll_adv_iso_init(void);
int lll_adv_iso_reset(void);
LLL_ISR_CODE_RAM_ATTR void lll_adv_iso_create_prepare(void *param);
LLL_ISR_CODE_RAM_ATTR void lll_adv_iso_prepare(void *param);

extern struct lll_adv_iso_stream *ull_adv_iso_lll_stream_get(uint16_t handle);

extern void ull_adv_iso_lll_biginfo_fill(struct pdu_adv *pdu, struct lll_adv_sync *lll_sync);
