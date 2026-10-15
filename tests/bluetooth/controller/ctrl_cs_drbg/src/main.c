/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <string.h>

#include <zephyr/ztest.h>

#include "hal/ecb.h"

#include "ull_cs_drbg.h"

/* The sample data in the Bluetooth Core specification is written most
 * significant octet first; convert it to the LSO first representation used
 * by the controller.
 */
static void mso_to_lso(uint8_t *dst, const uint8_t *src, uint8_t len)
{
	for (uint8_t i = 0U; i < len; i++) {
		dst[i] = src[len - 1U - i];
	}
}

/* Bluetooth Core specification, Vol 6, Part C, Section 7, CS DRBG sample data:
 * CS_IV = CS_IV_P || CS_IV_C, CS_IN = CS_IN_P || CS_IN_C and
 * CS_PV = CS_PV_P || CS_PV_C.
 */
static void drbg_sample_init(struct ull_cs_drbg *drbg)
{
	static const uint8_t iv_mso[16] = {
		0xE1, 0x0B, 0xC2, 0x8A, 0x0B, 0xFD, 0xDF, 0xE9,
		0x3E, 0x7F, 0x51, 0x86, 0xE0, 0xCA, 0x0B, 0x3B,
	};
	static const uint8_t in_mso[8] = {
		0x9F, 0xF4, 0x77, 0xC1, 0x86, 0x73, 0x84, 0x0D,
	};
	static const uint8_t pv_mso[16] = {
		0xC9, 0x80, 0xDE, 0xDF, 0x98, 0x82, 0xED, 0x44,
		0x64, 0xA6, 0x74, 0x96, 0x78, 0x68, 0xF1, 0x43,
	};
	uint8_t iv[16];
	uint8_t in[8];
	uint8_t pv[16];

	mso_to_lso(iv, iv_mso, sizeof(iv));
	mso_to_lso(in, in_mso, sizeof(in));
	mso_to_lso(pv, pv_mso, sizeof(pv));

	ull_cs_drbg_init(drbg, iv, in, pv);
}

/* Vol 6, Part C, Section 1.1, Encrypt Command sample data */
ZTEST(cs_drbg, test_cs_drbg_e)
{
	static const uint8_t key_mso[16] = {
		0x4C, 0x68, 0x38, 0x41, 0x39, 0xF5, 0x74, 0xD8,
		0x36, 0xBC, 0xF3, 0x4E, 0x9D, 0xFB, 0x01, 0xBF,
	};
	static const uint8_t data_mso[16] = {
		0x02, 0x13, 0x24, 0x35, 0x46, 0x57, 0x68, 0x79,
		0xAC, 0xBD, 0xCE, 0xDF, 0xE0, 0xF1, 0x02, 0x13,
	};
	static const uint8_t expected_mso[16] = {
		0x99, 0xAD, 0x1B, 0x52, 0x26, 0xA3, 0x7E, 0x3E,
		0x05, 0x8E, 0x3B, 0x8E, 0x27, 0xC2, 0xC6, 0x66,
	};
	uint8_t key[16];
	uint8_t data[16];
	uint8_t expected[16];
	uint8_t out[16];

	mso_to_lso(key, key_mso, sizeof(key));
	mso_to_lso(data, data_mso, sizeof(data));
	mso_to_lso(expected, expected_mso, sizeof(expected));

	ecb_encrypt(key, data, out, NULL);
	zassert_mem_equal(out, expected, sizeof(out));
}

/* Instantiation h9 (f7, f8 and f9) producing K_DRBG and V_DRBG */
ZTEST(cs_drbg, test_cs_drbg_init)
{
	static const uint8_t key_mso[16] = {
		0xEE, 0xE0, 0x4D, 0x7C, 0x76, 0x11, 0x3A, 0x5C,
		0xEC, 0x99, 0x2A, 0xE3, 0x20, 0xC2, 0x4D, 0x27,
	};
	static const uint8_t v_mso[16] = {
		0xDF, 0x90, 0x56, 0x47, 0xC1, 0x06, 0x6E, 0x6F,
		0x52, 0xC0, 0x3E, 0xDF, 0xB8, 0x2B, 0x69, 0x28,
	};
	struct ull_cs_drbg drbg;
	uint8_t key[16];
	uint8_t v[16];

	mso_to_lso(key, key_mso, sizeof(key));
	mso_to_lso(v, v_mso, sizeof(v));

	drbg_sample_init(&drbg);

	zassert_mem_equal(drbg.key, key, sizeof(key));
	zassert_mem_equal(drbg.v, v, sizeof(v));
}

ZTEST(cs_drbg, test_cs_drbg_rand)
{
	static const uint8_t expected_mso[16] = {
		0x79, 0x74, 0x1F, 0xD1, 0x8F, 0x57, 0x7B, 0x45,
		0xD0, 0x9A, 0x66, 0x5A, 0x7F, 0x1F, 0x28, 0x58,
	};
	struct ull_cs_drbg drbg;
	uint8_t expected[16];
	uint8_t out[16];

	mso_to_lso(expected, expected_mso, sizeof(expected));

	drbg_sample_init(&drbg);

	ull_cs_drbg_rand(&drbg, 0U, ULL_CS_DRBG_TID_HOP_CHAN_NON_MODE_0, out, sizeof(out));
	zassert_mem_equal(out, expected, sizeof(out));
}

/* Channel selection algorithm #3b, channel shuffling with cr1 */
ZTEST(cs_drbg, test_cs_drbg_cr1)
{
	static const uint8_t channels[19] = {
		2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
	};
	static const uint8_t expected_mode_0[19] = {
		11, 7, 14, 18, 9, 19, 10, 8, 5, 2, 4, 15, 16, 13, 12, 6, 17, 20, 3,
	};
	static const uint8_t expected_non_mode_0[19] = {
		6, 12, 5, 10, 3, 2, 18, 17, 16, 8, 11, 7, 19, 4, 13, 20, 9, 15, 14,
	};
	struct ull_cs_drbg drbg;
	uint8_t shuffled[19];

	drbg_sample_init(&drbg);

	(void)memset(shuffled, 0, sizeof(shuffled));
	ull_cs_drbg_cr1(&drbg, 0U, ULL_CS_DRBG_TID_HOP_CHAN_MODE_0, channels, shuffled,
			sizeof(channels));
	zassert_mem_equal(shuffled, expected_mode_0, sizeof(shuffled));

	(void)memset(shuffled, 0, sizeof(shuffled));
	ull_cs_drbg_cr1(&drbg, 3U, ULL_CS_DRBG_TID_HOP_CHAN_NON_MODE_0, channels, shuffled,
			sizeof(channels));
	zassert_mem_equal(shuffled, expected_non_mode_0, sizeof(shuffled));
}

/* CS_SYNC access address generation */
ZTEST(cs_drbg, test_cs_drbg_aa)
{
	static const struct {
		uint16_t step;
		uint32_t initiator_aa;
		uint32_t reflector_aa;
	} vectors[] = {
		{ 0U, 0x6C376AB8UL, 0xF079BC3AUL },
		{ 1U, 0x011CAE4EUL, 0xD06ACDDAUL },
		{ 2U, 0x64061214UL, 0x28942F38UL },
		{ 14U, 0xF7219786UL, 0x57176470UL },
	};
	struct ull_cs_drbg drbg;

	drbg_sample_init(&drbg);

	for (uint8_t i = 0U; i < ARRAY_SIZE(vectors); i++) {
		uint32_t initiator_aa;
		uint32_t reflector_aa;

		ull_cs_drbg_aa_get(&drbg, vectors[i].step, &initiator_aa, &reflector_aa);
		zassert_equal(initiator_aa, vectors[i].initiator_aa, "step %u",
			      vectors[i].step);
		zassert_equal(reflector_aa, vectors[i].reflector_aa, "step %u",
			      vectors[i].step);
	}
}

/* Helpers built on hr1 must stay within their documented ranges */
ZTEST(cs_drbg, test_cs_drbg_ranges)
{
	struct ull_cs_drbg drbg;

	drbg_sample_init(&drbg);

	for (uint16_t step = 0U; step < 64U; step++) {
		uint8_t val;

		val = ull_cs_drbg_main_mode_steps_get(&drbg, step, 2U, 5U);
		zassert_between_inclusive(val, 2U, 5U);

		val = ull_cs_drbg_antenna_permutation_get(&drbg, step, 4U);
		zassert_true(val < 24U);

		val = ull_cs_drbg_antenna_permutation_get(&drbg, step, 1U);
		zassert_equal(val, 0U);

		val = ull_cs_drbg_tone_presence_get(&drbg, step);
		zassert_true(val <= 1U);
	}

	zassert_equal(ull_cs_drbg_main_mode_steps_get(&drbg, 0U, 3U, 3U), 3U);
}

ZTEST_SUITE(cs_drbg, NULL, NULL, NULL, NULL, NULL);
