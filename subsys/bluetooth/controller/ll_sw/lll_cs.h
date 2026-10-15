/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* This header relies on lll.h (struct lll_hdr, struct lll_prepare_param) being
 * included first by the translation unit, following the controller LLL header
 * convention used by e.g. lll_sync_iso.h. lll.h is intentionally not included
 * here because it does not provide an include guard.
 */

/* Maximum number of Channel Sounding steps in a CS subevent, as defined by
 * the Bluetooth Core specification, Vol 6, Part B, Section 4.5.
 */
#define LLL_CS_MAX_STEPS 160U

/* Maximum number of Channel Sounding channel indices (0..78 minus the seven
 * indices that are not allowed for Channel Sounding) usable in a procedure.
 */
#define LLL_CS_MAX_CHANNELS 72U

/* Maximum number of antenna paths. */
#define LLL_CS_N_AP_MAX 4U

/* Channel Sounding roles, as defined by the Bluetooth Core specification,
 * Vol 6, Part B, "Channel Sounding".
 */
#define LLL_CS_ROLE_INITIATOR 0U
#define LLL_CS_ROLE_REFLECTOR 1U

/* Channel Sounding step modes, as defined by the Bluetooth Core specification,
 * Vol 6, Part A, Section 5 "Channel Sounding".
 */
#define LLL_CS_MODE_0 0U /* Frequency offset and timing recovery. */
#define LLL_CS_MODE_1 1U /* RTT (time of flight) exchange. */
#define LLL_CS_MODE_2 2U /* Tone (phase based ranging) exchange. */
#define LLL_CS_MODE_3 3U /* Combined RTT and tone exchange. */

/* Channel Sounding step timings in microseconds, Bluetooth Core specification,
 * Vol 6, Part H, Section 4.3. Only the mandatory interlude, frequency change and
 * tone durations are supported.
 */
#define LLL_CS_T_IP1_US   145U /* Interlude between CS_SYNC packets. */
#define LLL_CS_T_IP2_US   145U /* Interlude between tones. */
#define LLL_CS_T_FCS_US   150U /* Frequency change between steps. */
#define LLL_CS_T_PM_US    40U  /* Tone (phase measurement) duration. */
#define LLL_CS_T_SW_US    10U  /* Supported antenna switch time. */
#define LLL_CS_T_RD_US    5U   /* Ramp down time. */
#define LLL_CS_T_GD_US    10U  /* Guard time before a mode-0 tone. */
#define LLL_CS_T_FM_US    80U  /* Mode-0 frequency measurement tone. */
#define LLL_CS_T_SY_1M_US 44U  /* CS_SYNC duration, LE 1M, AA only. */
#if defined(CONFIG_BT_CTLR_CS_SYNC_2M)
#define LLL_CS_T_SY_2M_US 22U  /* CS_SYNC duration, LE 2M, AA only. */
#endif

/* Step results not available marker for time differences. */
#define LLL_CS_TIME_DIFF_NA ((int16_t)0x8000)

/* Step results not available marker for RSSI. */
#define LLL_CS_RSSI_NA 0x7F

/* Packet quality: access address check result. */
#define LLL_CS_AA_OK        0U
#define LLL_CS_AA_NOT_FOUND 2U

/* Duration of the tone exchange of one device in a mode-2 step, i.e. N_AP
 * antenna path tones plus the tone extension slot, each preceded by the
 * selected antenna switch time t_sw.
 */
static inline uint32_t lll_cs_tone_len_us(uint8_t n_ap, uint8_t t_sw)
{
	return ((uint32_t)t_sw + LLL_CS_T_PM_US) * ((uint32_t)n_ap + 1U);
}

static inline uint32_t lll_cs_t_sy_us(uint8_t phy)
{
#if defined(CONFIG_BT_CTLR_CS_SYNC_2M)
	if (phy == PHY_2M) {
		return LLL_CS_T_SY_2M_US;
	}
#else
	ARG_UNUSED(phy);
#endif

	return LLL_CS_T_SY_1M_US;
}

/* Air time offset, relative to the start of the step, of a step action.
 * Action 0 is the initiator transmission and action 1 the reflector
 * transmission.
 */
static inline uint32_t lll_cs_action_offset_us(uint8_t mode, uint8_t action, uint8_t n_ap,
					       uint8_t t_sw, uint8_t phy)
{
	if (action == 0U) {
		return 0U;
	}

	if (mode == LLL_CS_MODE_2) {
		return lll_cs_tone_len_us(n_ap, t_sw) + LLL_CS_T_RD_US + LLL_CS_T_IP2_US;
	}

	return lll_cs_t_sy_us(phy) + LLL_CS_T_RD_US + LLL_CS_T_IP1_US;
}

/* Air time duration of a step action. */
static inline uint32_t lll_cs_action_len_us(uint8_t mode, uint8_t action, uint8_t n_ap,
					    uint8_t t_sw, uint8_t phy)
{
	if (mode == LLL_CS_MODE_2) {
		return lll_cs_tone_len_us(n_ap, t_sw);
	}

	if ((mode == LLL_CS_MODE_0) && (action == 1U)) {
		return lll_cs_t_sy_us(phy) + LLL_CS_T_GD_US + LLL_CS_T_FM_US;
	}

	return lll_cs_t_sy_us(phy);
}

/* Duration of a step, excluding the frequency change period that follows. */
static inline uint32_t lll_cs_step_len_us(uint8_t mode, uint8_t n_ap, uint8_t t_sw, uint8_t phy)
{
	return lll_cs_action_offset_us(mode, 1U, n_ap, t_sw, phy) +
	       lll_cs_action_len_us(mode, 1U, n_ap, t_sw, phy) + LLL_CS_T_RD_US;
}

/* Description of a single Channel Sounding step to be executed by the LLL
 * radio event, filled by the ULL CS step planning.
 */
struct lll_cs_step {
	uint8_t access_address[4];   /* Initiator CS_SYNC access address. */
	uint8_t access_address_r[4]; /* Reflector CS_SYNC access address. */
	uint8_t mode;                /* CS step mode (LLL_CS_MODE_0..2). */
	uint8_t channel_index;       /* CS channel index (0..78). */
};

/* Phase correction term extracted from a received CS tone. */
struct lll_cs_iq_sample {
	int16_t i;
	int16_t q;
};

/* Measurement results of a single Channel Sounding step. */
struct lll_cs_step_result {
	struct lll_cs_iq_sample iq; /* Mode-2 tone PCT, 12-bit I and Q. */
	int16_t time_diff;          /* Mode-1 ToA-ToD/ToD-ToA, 0.5 ns units. */
	int16_t freq_offset;        /* Mode-0 initiator FFO, 0.01 ppm units. */
	int8_t  rssi;               /* CS_SYNC RSSI in dBm. */
	uint8_t aa_quality;         /* LLL_CS_AA_OK or LLL_CS_AA_NOT_FOUND. */
	uint8_t tone_quality;       /* Mode-2 tone quality indicator. */
};

/* Channel Sounding LLL event context.
 *
 * One per connection (or for the CS Test mode). A single LLL radio event
 * executes one CS subevent, which in this implementation is a complete CS
 * procedure. The ULL plans the steps of the subevent (mode, channel and
 * access addresses) before the event, and the LLL executes them and reports
 * the per-step results.
 *
 * The struct lll_hdr must be the first member so that HDR_LLL2ULL() can recover
 * the owning struct ull_hdr.
 */
struct lll_cs {
	struct lll_hdr hdr;

	uint16_t handle;              /* Connection (or CS Test) handle. */
	uint8_t config_id;            /* CS configuration identifier. */

	/* Negotiated configuration. */
	uint8_t access_address[4];    /* Unused, kept for CS Test. */
	uint8_t role;                 /* LLL_CS_ROLE_INITIATOR/REFLECTOR. */
	uint8_t phy;                  /* CS_SYNC PHY (PHY_1M). */
	uint8_t main_mode;            /* Main mode step type. */
	uint8_t sub_mode;             /* Sub mode step type, or "none". */
	uint8_t main_mode_steps;      /* Maximum number of main mode steps. */
	uint8_t main_mode_repetition; /* Main mode steps repeated. */
	uint8_t mode_0_steps;         /* Number of mode-0 steps. */
	uint8_t rtt_type;             /* RTT payload type. */
	uint8_t channel_map[10];      /* Allowed CS channels bitmap. */
	uint8_t n_ap;                 /* Number of antenna paths. */
	uint8_t t_sw_us;              /* Selected antenna switch time. */
	const int8_t *fae_table;      /* Peer FAE table, or NULL. */

	/* Scheduling parameters. */
	uint8_t  subevents_per_event;
	uint32_t subevent_interval_us;
	uint32_t subevent_len_us;
	uint16_t procedure_count_max;

	/* Per subevent parameters set by the ULL before the radio event. */
	uint16_t procedure_counter;    /* CS procedure counter. */
	uint16_t start_acl_conn_event; /* ACL event counter anchoring the event. */
	uint32_t remainder;            /* Anchor remainder of the CS event. */
	uint32_t start_offset_us;      /* Step 0 start after the event start. */
	uint32_t window_us;            /* Reflector first CS_SYNC widening. */
	uint32_t acl_ww_us;            /* Peripheral ACL window widening. */
	uint8_t  acl_phy;              /* ACL PHY of the anchoring event. */
	uint8_t  acl_periph:1;         /* Local device is the ACL peripheral. */
	const void *acl_lll;           /* LLL context of the ACL connection. */
	uint32_t duration_us;          /* Planned duration of the steps. */

	/* Planned steps of the subevent. */
	uint8_t step_count;
	struct lll_cs_step steps[LLL_CS_MAX_STEPS];

	/* Per-step measurement results captured during the subevent. */
	struct lll_cs_step_result results[LLL_CS_MAX_STEPS];

	/* Radio event execution state. */
	uint8_t  step_curr;     /* Step being executed. */
	uint8_t  action;        /* Step action, 0: initiator Tx, 1: reflector Tx. */
	uint8_t  synced:1;      /* Reflector timing synchronized to initiator. */
	uint8_t  rx_pending:1;  /* Subevent result enqueued towards ULL. */
	uint8_t  is_rx:1;       /* Current action is a reception. */
	uint8_t  abort_reason;  /* Subevent abort reason, 0 if not aborted. */
	uint32_t t_step_us;     /* Air time of the current step start. */
	uint32_t t_end_us;      /* Latest planned end of the subevent. */
	int32_t  ffo_sum;       /* Sum of mode-0 FFO estimates, 62.5 ppb. */
	uint8_t  ffo_count;     /* Number of mode-0 FFO estimates. */
};

int lll_cs_init(void);
int lll_cs_reset(void);
void lll_cs_prepare(void *param);
