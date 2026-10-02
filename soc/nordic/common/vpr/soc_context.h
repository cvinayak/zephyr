/*
 * Copyright (C) 2024 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SOC_RISCV_NORDIC_NRF_COMMON_VPR_SOC_CONTEXT_H_
#define SOC_RISCV_NORDIC_NRF_COMMON_VPR_SOC_CONTEXT_H_

#ifdef CONFIG_NORDIC_VPR_HW_STACKING
#define SOC_ESF_MEMBERS						\
	unsigned long minttresh;				\
	unsigned long sp_align;

#define SOC_ESF_INIT						\
	0,							\
	0
#else
#define SOC_ESF_MEMBERS						\
	unsigned long minttresh;

#define SOC_ESF_INIT						\
	0
#endif /* CONFIG_NORDIC_VPR_HW_STACKING */

#endif /* SOC_RISCV_NORDIC_NRF_COMMON_VPR_SOC_CONTEXT_H_ */
