/*
 * Copyright (C) 2024 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SOC_RISCV_NORDIC_NRF_COMMON_VPR_SOC_OFFSETS_H_
#define SOC_RISCV_NORDIC_NRF_COMMON_VPR_SOC_OFFSETS_H_

#ifdef CONFIG_NORDIC_VPR_HW_STACKING
#define GEN_SOC_OFFSET_SYMS()					\
	GEN_OFFSET_SYM(soc_esf_t, minttresh);			\
	GEN_OFFSET_SYM(soc_esf_t, sp_align)
#else
#define GEN_SOC_OFFSET_SYMS()					\
	GEN_OFFSET_SYM(soc_esf_t, minttresh)
#endif /* CONFIG_NORDIC_VPR_HW_STACKING */

#endif /* SOC_RISCV_NORDIC_NRF_COMMON_VPR_SOC_OFFSETS_H_ */
