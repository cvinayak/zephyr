/*
 * Copyright (c) 2026 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <string.h>

#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include "hal/ecb.h"

#include "ull_cs_drbg.h"

#define BLOCK_SIZE 16U

/* Security function e: AES-128 with LSO first key, input and output. */
static void cs_e(const uint8_t key[BLOCK_SIZE], const uint8_t in[BLOCK_SIZE],
		 uint8_t out[BLOCK_SIZE])
{
	ecb_encrypt(key, in, out, NULL);
}

/* DRBG chain function f7: CBC-MAC like chaining over the input bit string,
 * processing 128-bit blocks starting from the most significant one.
 */
static void cs_f7(const uint8_t k[BLOCK_SIZE], const uint8_t *in, uint8_t len,
		  uint8_t out[BLOCK_SIZE])
{
	(void)memset(out, 0, BLOCK_SIZE);

	for (uint8_t offset = len; offset >= BLOCK_SIZE; offset -= BLOCK_SIZE) {
		const uint8_t *block = &in[offset - BLOCK_SIZE];

		for (uint8_t i = 0U; i < BLOCK_SIZE; i++) {
			out[i] ^= block[i];
		}

		cs_e(k, out, out);
	}
}

/* DRBG derivation function f8: derive 256-bit seed material from the 320-bit
 * input bit string CS_IV || CS_IN || CS_PV.
 */
static void cs_f8(const uint8_t input[40], uint8_t sm[32])
{
	/* K = 0x000102030405060708090A0B0C0D0E0F */
	static const uint8_t k[BLOCK_SIZE] = {
		0x0FU, 0x0EU, 0x0DU, 0x0CU, 0x0BU, 0x0AU, 0x09U, 0x08U,
		0x07U, 0x06U, 0x05U, 0x04U, 0x03U, 0x02U, 0x01U, 0x00U,
	};
	/* buf = V || S, with S occupying the 64 least significant octets */
	uint8_t buf[80] = {0U};
	uint8_t k2[BLOCK_SIZE];
	uint8_t x[BLOCK_SIZE];

	/* S = 0x0000002800000020 || input_bit_string || 0x80 || 0x00 * 15 */
	buf[15] = 0x80U;
	(void)memcpy(&buf[16], input, 40U);
	sys_put_le32(0x00000020UL, &buf[56]);
	sys_put_le32(0x00000028UL, &buf[60]);

	/* V = 0, K2 = f7(K, V || S) */
	cs_f7(k, buf, sizeof(buf), k2);

	/* V = 0x00000001000000000000000000000000, X = f7(K, V || S) */
	buf[76] = 0x01U;
	cs_f7(k, buf, sizeof(buf), x);

	/* SM = e(K2, X) || e(K2, e(K2, X)) */
	cs_e(k2, x, &sm[BLOCK_SIZE]);
	cs_e(k2, &sm[BLOCK_SIZE], &sm[0]);
}

/* DRBG update function f9: refresh K and V using the seed material. */
static void cs_f9(const uint8_t sm[32], uint8_t k[BLOCK_SIZE], uint8_t v[BLOCK_SIZE])
{
	uint8_t x[32];

	/* V = V[127:8] || ((V[7:0] + 1) mod 2^8) */
	v[0]++;
	cs_e(k, v, &x[BLOCK_SIZE]);

	v[0]++;
	cs_e(k, v, &x[0]);

	for (uint8_t i = 0U; i < sizeof(x); i++) {
		x[i] ^= sm[i];
	}

	(void)memcpy(v, &x[0], BLOCK_SIZE);
	(void)memcpy(k, &x[BLOCK_SIZE], BLOCK_SIZE);
}

void ull_cs_drbg_init(struct ull_cs_drbg *drbg, const uint8_t iv[ULL_CS_DRBG_IV_SIZE],
		      const uint8_t in[ULL_CS_DRBG_IN_SIZE],
		      const uint8_t pv[ULL_CS_DRBG_PV_SIZE])
{
	uint8_t input[ULL_CS_DRBG_PV_SIZE + ULL_CS_DRBG_IN_SIZE + ULL_CS_DRBG_IV_SIZE];
	uint8_t sm[32];

	/* Instantiation function h9, input bit string is CS_IV || CS_IN || CS_PV
	 * with CS_PV in the least significant octets.
	 */
	(void)memcpy(&input[0], pv, ULL_CS_DRBG_PV_SIZE);
	(void)memcpy(&input[ULL_CS_DRBG_PV_SIZE], in, ULL_CS_DRBG_IN_SIZE);
	(void)memcpy(&input[ULL_CS_DRBG_PV_SIZE + ULL_CS_DRBG_IN_SIZE], iv,
		     ULL_CS_DRBG_IV_SIZE);

	cs_f8(input, sm);

	(void)memset(drbg, 0, sizeof(*drbg));
	cs_f9(sm, drbg->key, drbg->v);
}

void ull_cs_drbg_cache_clear(struct ull_cs_drbg *drbg)
{
	(void)memset(drbg->cache, 0, sizeof(drbg->cache));
	drbg->tone_presence_bits = 0U;
	drbg->tone_presence_avail = 0U;
}

void ull_cs_drbg_rand(struct ull_cs_drbg *drbg, uint16_t step_counter, uint8_t tid,
		      uint8_t *out, uint8_t len)
{
	struct ull_cs_drbg_cache *cache = &drbg->cache[tid];

	/* The CSTransactionCounter restarts, and unused random bits are
	 * discarded, whenever the CS step counter of the nonce changes.
	 */
	if (cache->step_counter != step_counter) {
		cache->step_counter = step_counter;
		cache->counter = 0U;
		cache->avail = 0U;
	}

	while (len > 0U) {
		uint8_t chunk;

		if (cache->avail == 0U) {
			uint8_t nonce[BLOCK_SIZE];

			/* Nonce = V_DRBG with the CSTransactionCounter,
			 * CSTransactionID and CS step counter fields applied.
			 */
			(void)memcpy(nonce, drbg->v, sizeof(nonce));
			nonce[0] += cache->counter;
			nonce[1] += tid;
			sys_put_le16(sys_get_le16(&nonce[2]) + step_counter, &nonce[2]);

			cs_e(drbg->key, nonce, cache->bytes);

			cache->counter++;
			cache->avail = BLOCK_SIZE;
		}

		/* Cached octets are consumed starting from the most
		 * significant octet.
		 */
		chunk = MIN(len, cache->avail);
		cache->avail -= chunk;
		(void)memcpy(out, &cache->bytes[cache->avail], chunk);

		out += chunk;
		len -= chunk;
	}
}

uint8_t ull_cs_drbg_hr1(struct ull_cs_drbg *drbg, uint16_t step_counter, uint8_t tid,
			uint8_t r)
{
	uint32_t t;
	uint8_t rand;

	if (r <= 1U) {
		return 0U;
	}

	ull_cs_drbg_rand(drbg, step_counter, tid, &rand, 1U);
	t = (uint32_t)r * rand;

	if ((t & 0xFFU) < (256U % r)) {
		ull_cs_drbg_rand(drbg, step_counter, tid, &rand, 1U);

		return (uint8_t)((((uint32_t)rand * r * 256U) + t) >> 16);
	}

	return (uint8_t)(t >> 8);
}

void ull_cs_drbg_cr1(struct ull_cs_drbg *drbg, uint16_t step_counter, uint8_t tid,
		     const uint8_t *channels, uint8_t *shuffled, uint8_t count)
{
	/* Inside-out Fisher-Yates shuffle driven by hr1 */
	for (uint8_t i = 0U; i < count; i++) {
		uint8_t j;

		j = ull_cs_drbg_hr1(drbg, step_counter, tid, i + 1U);
		if (j != i) {
			shuffled[i] = shuffled[j];
		}

		shuffled[j] = channels[i];
	}
}

/* Autocorrelation score of a candidate CS access address, lower is better. */
static uint32_t cs_aa_score(uint32_t aa)
{
	uint32_t score = 0U;

	for (uint8_t k = 1U; k <= 3U; k++) {
		int32_t c = 0;

		for (uint8_t i = 0U; i < (32U - k); i++) {
			c += (int32_t)(((aa >> i) ^ (aa >> (i + k))) & 1U);
		}

		c = (2 * c) - (int32_t)(32U - k);
		score += (uint32_t)((c < 0) ? -c : c);
	}

	return score;
}

void ull_cs_drbg_aa_get(struct ull_cs_drbg *drbg, uint16_t step_counter,
			uint32_t *initiator_aa, uint32_t *reflector_aa)
{
	uint8_t buf[16];
	uint32_t s0;
	uint32_t s1;
	uint32_t s2;
	uint32_t s3;

	ull_cs_drbg_rand(drbg, step_counter, ULL_CS_DRBG_TID_ACCESS_ADDRESS, buf,
			 sizeof(buf));

	/* buf[15] is the first generated octet */
	s0 = sys_get_le32(&buf[12]);
	s1 = sys_get_le32(&buf[8]);
	s2 = sys_get_le32(&buf[4]);
	s3 = sys_get_le32(&buf[0]);

	*initiator_aa = (cs_aa_score(s0) < cs_aa_score(s1)) ? s0 : s1;
	*reflector_aa = (cs_aa_score(s2) < cs_aa_score(s3)) ? s2 : s3;
}

uint8_t ull_cs_drbg_main_mode_steps_get(struct ull_cs_drbg *drbg, uint16_t step_counter,
					uint8_t min_steps, uint8_t max_steps)
{
	uint8_t r;

	if (max_steps <= min_steps) {
		return min_steps;
	}

	r = max_steps - min_steps + 1U;

	return min_steps + ull_cs_drbg_hr1(drbg, step_counter,
					   ULL_CS_DRBG_TID_SUBEVENT_SUB_MODE, r);
}

uint8_t ull_cs_drbg_antenna_permutation_get(struct ull_cs_drbg *drbg,
					    uint16_t step_counter, uint8_t n_ap)
{
	uint8_t n_ap_factorial = 1U;

	if (n_ap <= 1U) {
		return 0U;
	}

	n_ap = MIN(n_ap, 4U);
	for (uint8_t i = 2U; i <= n_ap; i++) {
		n_ap_factorial *= i;
	}

	return ull_cs_drbg_hr1(drbg, step_counter, ULL_CS_DRBG_TID_ANTENNA_PERMUTATION,
			       n_ap_factorial);
}

uint8_t ull_cs_drbg_tone_presence_get(struct ull_cs_drbg *drbg, uint16_t step_counter)
{
	struct ull_cs_drbg_cache *cache = &drbg->cache[ULL_CS_DRBG_TID_T_PM_TONE_PRESENCE];
	uint8_t presence;

	if ((drbg->tone_presence_avail == 0U) || (cache->step_counter != step_counter)) {
		ull_cs_drbg_rand(drbg, step_counter, ULL_CS_DRBG_TID_T_PM_TONE_PRESENCE,
				 &drbg->tone_presence_bits, 1U);
		drbg->tone_presence_avail = 8U;
	}

	/* Bits are consumed starting from the most significant bit */
	presence = ((drbg->tone_presence_bits & BIT(7)) != 0U) ? 1U : 0U;
	drbg->tone_presence_bits <<= 1;
	drbg->tone_presence_avail--;

	return presence;
}
