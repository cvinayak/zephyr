/*
 * Copyright (c) 2026 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SUBSYS_BLUETOOTH_CONTROLLER_LL_SW_ULL_CS_DRBG_H_
#define ZEPHYR_SUBSYS_BLUETOOTH_CONTROLLER_LL_SW_ULL_CS_DRBG_H_

/* Channel Sounding deterministic random bit generator (CS DRBG), refer to the
 * Bluetooth Core specification, Vol 6, Part E, Section 3 "Deterministic
 * random bit generator" and Vol 6, Part H, "Channel Sounding".
 *
 * All multi-octet values are stored least significant octet (LSO) first.
 */

/* CS DRBG transaction identifiers (CSTransactionID) */
#define ULL_CS_DRBG_TID_HOP_CHAN_NON_MODE_0 0x00U
#define ULL_CS_DRBG_TID_HOP_CHAN_MODE_0     0x01U
#define ULL_CS_DRBG_TID_SUBEVENT_SUB_MODE   0x02U
#define ULL_CS_DRBG_TID_T_PM_TONE_PRESENCE  0x03U
#define ULL_CS_DRBG_TID_ANTENNA_PERMUTATION 0x04U
#define ULL_CS_DRBG_TID_ACCESS_ADDRESS      0x05U
#define ULL_CS_DRBG_TID_MARKER_POSITION     0x06U
#define ULL_CS_DRBG_TID_MARKER_SIGNAL       0x07U
#define ULL_CS_DRBG_TID_RANDOM_SEQUENCE     0x08U
#define ULL_CS_DRBG_TID_BACKTRACKING        0x09U
#define ULL_CS_DRBG_TID_COUNT               0x0AU

/* Sizes of the CS security vectors exchanged in LL_CS_SEC_REQ/RSP */
#define ULL_CS_DRBG_IV_SIZE 16U /* CS_IV = CS_IV_P || CS_IV_C */
#define ULL_CS_DRBG_IN_SIZE 8U  /* CS_IN = CS_IN_P || CS_IN_C */
#define ULL_CS_DRBG_PV_SIZE 16U /* CS_PV = CS_PV_P || CS_PV_C */

struct ull_cs_drbg_cache {
	uint16_t step_counter;   /* CS step counter the cached bits belong to */
	uint8_t  counter;        /* CSTransactionCounter */
	uint8_t  avail;          /* Number of unused octets in bytes[] */
	uint8_t  bytes[16];      /* Last 128-bit CS_DRBG output */
};

struct ull_cs_drbg {
	uint8_t key[16];         /* Temporal key K_DRBG */
	uint8_t v[16];           /* Nonce vector V_DRBG */
	struct ull_cs_drbg_cache cache[ULL_CS_DRBG_TID_COUNT];
	uint8_t tone_presence_bits;
	uint8_t tone_presence_avail;
};

/* Instantiate the CS DRBG (h9) from the CS security vectors. */
void ull_cs_drbg_init(struct ull_cs_drbg *drbg, const uint8_t iv[ULL_CS_DRBG_IV_SIZE],
		      const uint8_t in[ULL_CS_DRBG_IN_SIZE],
		      const uint8_t pv[ULL_CS_DRBG_PV_SIZE]);

/* Discard all cached random bits, keeping K_DRBG and V_DRBG. */
void ull_cs_drbg_cache_clear(struct ull_cs_drbg *drbg);

/* Generate len random octets for a CS step counter and transaction ID. */
void ull_cs_drbg_rand(struct ull_cs_drbg *drbg, uint16_t step_counter, uint8_t tid,
		      uint8_t *out, uint8_t len);

/* Random number generation function hr1, returns a value in [0, r - 1]. */
uint8_t ull_cs_drbg_hr1(struct ull_cs_drbg *drbg, uint16_t step_counter, uint8_t tid,
			uint8_t r);

/* Channel shuffling function cr1 used by channel selection algorithm #3b. */
void ull_cs_drbg_cr1(struct ull_cs_drbg *drbg, uint16_t step_counter, uint8_t tid,
		     const uint8_t *channels, uint8_t *shuffled, uint8_t count);

/* Generate the CS_SYNC access addresses of a CS step. */
void ull_cs_drbg_aa_get(struct ull_cs_drbg *drbg, uint16_t step_counter,
			uint32_t *initiator_aa, uint32_t *reflector_aa);

/* Random number of main mode steps in [min_steps, max_steps]. */
uint8_t ull_cs_drbg_main_mode_steps_get(struct ull_cs_drbg *drbg, uint16_t step_counter,
					uint8_t min_steps, uint8_t max_steps);

/* Random antenna path permutation index for n_ap antenna paths (1..4). */
uint8_t ull_cs_drbg_antenna_permutation_get(struct ull_cs_drbg *drbg,
					    uint16_t step_counter, uint8_t n_ap);

/* Random presence of the tone extension slot of a mode-2/mode-3 step. */
uint8_t ull_cs_drbg_tone_presence_get(struct ull_cs_drbg *drbg, uint16_t step_counter);

#endif /* ZEPHYR_SUBSYS_BLUETOOTH_CONTROLLER_LL_SW_ULL_CS_DRBG_H_ */
