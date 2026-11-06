/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stddef.h>
#include <stdint.h>

#include <zephyr/kernel.h>

#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>

#include "hal/ccm.h"
#include "hal/ticker.h"
#include "util/util.h"
#include "util/mem.h"
#include "util/memq.h"
#include "pdu_df.h"
#include "lll/pdu_vendor.h"
#include "pdu.h"
#include "lll.h"
#include "lll/lll_adv_types.h"
#include "lll_adv.h"
#include "lll/lll_adv_pdu.h"
#include "ull_adv_types.h"
#include "ull_adv_internal.h"
#include "ll.h"

#include "babblekit/testcase.h"

/* AD data length larger than PDU_AC_EXT_AD_DATA_LEN_MAX, so that the
 * auxiliary PDU is fragmented into chain PDUs, and with a maximum common
 * extended header size of 63 octets would wrap an 8-bit length.
 */
#define AUX_TIME_AD_DATA_LEN  200U
#define AUX_TIME_MFG_DATA_LEN (AUX_TIME_AD_DATA_LEN - 2U)
#define AUX_TIME_UPDATE_COUNT 2U

BUILD_ASSERT(AUX_TIME_AD_DATA_LEN > PDU_AC_EXT_AD_DATA_LEN_MAX);
BUILD_ASSERT(AUX_TIME_AD_DATA_LEN <= CONFIG_BT_CTLR_ADV_DATA_LEN_MAX);

static struct ll_adv_aux_set *aux_set_get(struct bt_le_ext_adv *adv)
{
	struct ll_adv_set *adv_set;
	uint8_t handle;
	uint8_t err;

	err = ll_adv_set_by_hci_handle_get(bt_le_ext_adv_get_index(adv), &handle);
	if (err != 0U) {
		return NULL;
	}

	adv_set = ull_adv_set_get(handle);
	if ((adv_set == NULL) || (adv_set->lll.aux == NULL)) {
		return NULL;
	}

	return HDR_LLL2ULL(adv_set->lll.aux);
}

static void aux_time_check(const struct ll_adv_aux_set *aux)
{
	const struct lll_adv_aux *lll_aux;
	const struct pdu_adv *pdu_chain;
	const struct lll_adv *lll;
	const struct pdu_adv *pdu;
	uint32_t slot_us;
	uint32_t time_us;
	uint8_t chain_count;

	lll_aux = &aux->lll;
	lll = lll_aux->adv;

	/* On-air time of the auxiliary PDU and its chain PDUs */
	pdu = lll_adv_aux_data_peek(lll_aux);
	time_us = PDU_AC_US(pdu->len, lll->phy_s, lll->phy_flags);

	chain_count = 0U;
	pdu_chain = lll_adv_pdu_linked_next_get(pdu);
	while (pdu_chain != NULL) {
		time_us += EVENT_B2B_MAFS_US;
		time_us += PDU_AC_US(pdu_chain->len, lll->phy_s, lll->phy_flags);
		chain_count++;
		pdu_chain = lll_adv_pdu_linked_next_get(pdu_chain);
	}

	if (chain_count == 0U) {
		TEST_FAIL("No chain PDU");
	}

	slot_us = HAL_TICKER_TICKS_TO_US(aux->ull.ticks_slot);
	printk("Aux PDU len %u, chain PDUs %u, on-air %u us, reserved %u us\n",
	       pdu->len, chain_count, time_us, slot_us);

	if (slot_us < time_us) {
		TEST_FAIL("Aux event time reservation %u us less than on-air %u us",
			  slot_us, time_us);
	}
}

static void test_adv_aux_time_main(void)
{
	static uint8_t mfg_data[AUX_TIME_MFG_DATA_LEN] = { 0xFF, 0xFF, };
	const struct bt_data ad_long[] = {
		BT_DATA(BT_DATA_MANUFACTURER_DATA, mfg_data, sizeof(mfg_data)),
	};
	const struct bt_data ad_short[] = {
		BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
			sizeof(CONFIG_BT_DEVICE_NAME) - 1U),
	};
	struct bt_le_adv_param adv_param = {
		.id = BT_ID_DEFAULT,
		.sid = 0U,
		.secondary_max_skip = 0U,
		.options = (BT_LE_ADV_OPT_EXT_ADV | BT_LE_ADV_OPT_NO_2M),
		.interval_min = BT_GAP_ADV_FAST_INT_MIN_2,
		.interval_max = BT_GAP_ADV_FAST_INT_MAX_2,
		.peer = NULL,
	};
	struct ll_adv_aux_set *aux;
	struct bt_le_ext_adv *adv;
	int err;

	err = bt_enable(NULL);
	if (err != 0) {
		TEST_FAIL("Bluetooth init failed (err %d)", err);
	}

	err = bt_le_ext_adv_create(&adv_param, NULL, &adv);
	if (err != 0) {
		TEST_FAIL("Failed to create advertising set (err %d)", err);
	}

	err = bt_le_ext_adv_set_data(adv, ad_short, ARRAY_SIZE(ad_short), NULL, 0);
	if (err != 0) {
		TEST_FAIL("Failed to set advertising data (err %d)", err);
	}

	err = bt_le_ext_adv_start(adv, BT_LE_EXT_ADV_START_DEFAULT);
	if (err != 0) {
		TEST_FAIL("Failed to start advertising set (err %d)", err);
	}

	aux = aux_set_get(adv);
	if (aux == NULL) {
		TEST_FAIL("No auxiliary set");
	}

	if (aux->is_started == 0U) {
		TEST_FAIL("Auxiliary set not started");
	}

	/* Update the AD data of the enabled advertising set, repeat with the
	 * same AD data.
	 */
	for (uint8_t i = 0U; i < AUX_TIME_UPDATE_COUNT; i++) {
		err = bt_le_ext_adv_set_data(adv, ad_long, ARRAY_SIZE(ad_long), NULL, 0);
		if (err != 0) {
			TEST_FAIL("Failed to update advertising data (err %d)", err);
		}

		aux_time_check(aux);

		k_sleep(K_MSEC(100));
	}

	TEST_PASS_AND_EXIT("Aux time reservation tests passed");
}

static const struct bst_test_instance test_def[] = {
	{
		.test_id = "adv_aux_time",
		.test_descr = "Auxiliary event time reservation on AD data update",
		.test_main_f = test_adv_aux_time_main
	},
	BSTEST_END_MARKER
};

struct bst_test_list *test_adv_aux_time_install(struct bst_test_list *tests)
{
	return bst_add_tests(tests, test_def);
}
