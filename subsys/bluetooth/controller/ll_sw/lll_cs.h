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

/* Maximum number of Channel Sounding steps planned for a single subevent. This
 * mirrors NODE_RX_CS_MAX_STEPS used by the ULL result notification path.
 */
#define LLL_CS_MAX_STEPS 15U

/* Maximum number of Channel Sounding channel indices (0..78 minus the seven
 * indices that are not allowed for Channel Sounding) usable in a procedure.
 */
#define LLL_CS_MAX_CHANNELS 72U

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

/* Description of a single Channel Sounding step to be executed by the LLL
 * radio event. The ULL scheduling fills this descriptor for each step of a CS
 * subevent and the LLL programs the radio Channel Sounding hardware from it.
 */
struct lll_cs_step {
	uint8_t access_address[4]; /* CS_SYNC access address for the step. */
	uint8_t mode;              /* CS step mode (LLL_CS_MODE_0..3). */
	uint8_t channel_index;     /* CS channel index (0..78). */
	uint8_t rtt_type;          /* RTT payload type. */
	uint8_t num_tones;         /* Number of tones for the tone extension. */
	uint8_t tone_duration_us;  /* Single tone duration T_PM in us. */
	uint8_t num_ant_paths;     /* Antenna paths used for the tone slots. */
	uint8_t antenna_selection; /* Antenna permutation index. */
};

/* In-phase/quadrature (PCT) sample extracted from a received CS tone. */
struct lll_cs_iq_sample {
	int16_t i;
	int16_t q;
};

/* Channel Sounding LLL event context.
 *
 * One per connection (or for the CS Test mode) that runs the radio Channel
 * Sounding event. The context is filled by the ULL from the end-to-end control
 * procedure negotiation (LL_CS_REQ/RSP/IND) and from the agreed configuration,
 * and drives the LLL radio event that walks the
 * event -> subevent -> step hierarchy described by the Bluetooth Core
 * specification, Vol 6, Part B, "Channel Sounding".
 *
 * The struct lll_hdr must be the first member so that HDR_LLL2ULL() can recover
 * the owning struct ull_hdr, exactly like the ISO LLL contexts.
 */
struct lll_cs {
	struct lll_hdr hdr;

	/* Identification used when reporting the CS subevent results towards
	 * the ULL and the Host. The handle is the ACL connection handle (or the
	 * CS Test connection handle) and the configuration identifier selects
	 * the CS configuration the results belong to.
	 */
	uint16_t handle;              /* Connection (or CS Test) handle. */
	uint8_t config_id;            /* CS configuration identifier. */

	/* Negotiated configuration used to plan and schedule the CS event. */
	uint8_t access_address[4];    /* CS_SYNC access address. */
	uint8_t role;                 /* LLL_CS_ROLE_INITIATOR/REFLECTOR. */
	uint8_t phy;                  /* CS_SYNC PHY (PHY_1M/PHY_2M). */
	uint8_t main_mode;            /* Main mode step type. */
	uint8_t sub_mode;             /* Sub mode step type, or "none". */
	uint8_t main_mode_steps;      /* Number of main mode steps. */
	uint8_t main_mode_repetition; /* Sub mode insertion period. */
	uint8_t mode_0_steps;         /* Number of mode-0 steps. */
	uint8_t rtt_type;             /* RTT payload type. */
	uint8_t channel_map[10];      /* Allowed CS channels bitmap. */

	/* Scheduling parameters of the event -> subevent -> step hierarchy. */
	uint8_t  subevents_per_event;  /* Subevents in one CS event. */
	uint32_t subevent_interval_us; /* Start-to-start subevent spacing. */
	uint32_t subevent_len_us;      /* Duration of a single subevent. */
	uint32_t step_interval_us;     /* Start-to-start step spacing. */

	/* Procedure progress across CS events. */
	uint16_t procedure_counter;
	uint16_t procedure_count_max;

	/* Per-event iteration state, updated by the radio ISR as it steps
	 * through the subevents and steps of the current CS event.
	 */
	uint8_t  subevent_curr;        /* Subevent being executed (0-based). */
	uint8_t  step_curr;            /* Step being executed (0-based). */
	uint8_t  step_count;           /* Steps planned for current subevent. */
	uint8_t  channel_count;        /* Channels in the current subevent. */
	uint8_t  channels[LLL_CS_MAX_CHANNELS];
	struct lll_cs_step steps[LLL_CS_MAX_STEPS];

	/* Per-step measurement results captured from the radio Channel Sounding
	 * hardware as the current subevent is executed. step_rtt holds the RTT
	 * timestamp captured for mode-1/mode-3 steps and step_iq holds the
	 * IQ/phase (PCT) sample captured for mode-2/mode-3 steps. They are
	 * serialized into the CS subevent result notification reported towards
	 * the ULL and the Host once the subevent completes.
	 */
	uint32_t step_rtt[LLL_CS_MAX_STEPS];
	struct lll_cs_iq_sample step_iq[LLL_CS_MAX_STEPS];

	/* Set when at least one CS subevent result notification has been
	 * enqueued during the current CS event so that the radio ISR done
	 * handler schedules the ULL receive demux exactly once.
	 */
	uint8_t  rx_pending;
};

int lll_cs_init(void);
int lll_cs_reset(void);
void lll_cs_prepare(void *param);
