/*
 * Copyright (c) 2026 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/bluetooth/hci_types.h>

/* LL timing indices for the mandatory CS step timings implemented in lll_cs.h
 * (T_IP1/T_IP2 145 us, T_FCS 150 us, T_PM 40 us). See Core Spec Vol 6 Part H.
 */
#define ULL_CS_LL_T_IP1_MANDATORY_INDEX  7U
#define ULL_CS_LL_T_IP2_MANDATORY_INDEX  7U
#define ULL_CS_LL_T_FCS_MANDATORY_INDEX  9U
#define ULL_CS_LL_T_PM_MANDATORY_INDEX   2U

/* CS channel selection: #3b (default) and optional #3c. */
#define ULL_CS_CHSEL_ALG_3B 0U
#define ULL_CS_CHSEL_ALG_3C 1U

/* Time from CS radio event start to the first step anchor. */
#define ULL_CS_LEAD_US 1000U

/* Minimum offset from ACL anchor to CS event (matches LLCP scheduling). */
#define ULL_CS_OFFSET_MIN_US 5000U

/* HCI LE Set CS Procedure Parameters minimum subevent length. */
#define ULL_CS_MIN_SUBEVENT_LEN_US 1250U

/* LL mode_types bit for Mode-2 (tone / PBR); see Core Spec Vol 6 Part B. */
#define ULL_CS_LL_MODE_TYPE_2_MASK BIT(1)
#if defined(CONFIG_BT_CTLR_CS_STEP_MODE3)
#define ULL_CS_LL_MODE_TYPE_3_MASK BIT(0)
#else
#define ULL_CS_LL_MODE_TYPE_3_MASK 0U
#endif

/* Capability bitmasks for the mandatory step timings in LL_CS_CONFIG_* (index N -> BIT(N)). */
#define ULL_CS_CAP_T_IP1_MANDATORY BIT(ULL_CS_LL_T_IP1_MANDATORY_INDEX)
#define ULL_CS_CAP_T_IP2_MANDATORY BIT(ULL_CS_LL_T_IP2_MANDATORY_INDEX)
#define ULL_CS_CAP_T_FCS_MANDATORY BIT(ULL_CS_LL_T_FCS_MANDATORY_INDEX)
#define ULL_CS_CAP_T_PM_MANDATORY  BIT(ULL_CS_LL_T_PM_MANDATORY_INDEX)
