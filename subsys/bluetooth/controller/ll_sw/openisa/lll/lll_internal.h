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

#ifndef BT_CTLR_ULL_HIGH_CODE_RAM_ATTR
#if defined(CONFIG_BT_CTLR_ULL_HIGH_CODE_IN_RAM)
#define BT_CTLR_ULL_HIGH_CODE_RAM_ATTR __ramfunc
#else
#define BT_CTLR_ULL_HIGH_CODE_RAM_ATTR
#endif
#endif /* BT_CTLR_ULL_HIGH_CODE_RAM_ATTR */

#ifndef BT_CTLR_ULL_LOW_CODE_RAM_ATTR
#if defined(CONFIG_BT_CTLR_ULL_LOW_CODE_IN_RAM)
#define BT_CTLR_ULL_LOW_CODE_RAM_ATTR __ramfunc
#else
#define BT_CTLR_ULL_LOW_CODE_RAM_ATTR
#endif
#endif /* BT_CTLR_ULL_LOW_CODE_RAM_ATTR */

int lll_prepare_done(void *param);
BT_CTLR_LLL_ISR_CODE_RAM_ATTR int lll_done(void *param);
bool lll_is_done(void *param);
int lll_is_abort_cb(void *next, void *curr, lll_prepare_cb_t *resume_cb);
void lll_abort_cb(struct lll_prepare_param *prepare_param, void *param);

int lll_clk_on(void);
int lll_clk_on_wait(void);
int lll_clk_off(void);
uint32_t lll_event_offset_get(struct ull_hdr *ull);
uint32_t lll_preempt_calc(struct ull_hdr *ull, uint8_t ticker_id,
			  uint32_t ticks_at_event);
void lll_chan_set(uint32_t chan);
BT_CTLR_LLL_ISR_CODE_RAM_ATTR void lll_isr_status_reset(void);
