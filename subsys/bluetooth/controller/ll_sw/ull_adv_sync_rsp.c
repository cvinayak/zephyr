/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <soc.h>
#include <zephyr/bluetooth/hci_types.h>
#include <zephyr/sys/byteorder.h>

#include "hal/cpu.h"
#include "hal/ccm.h"
#include "hal/ticker.h"

#include "util/util.h"
#include "util/mem.h"
#include "util/memq.h"
#include "util/mayfly.h"
#include "util/dbuf.h"

#include "ticker/ticker.h"

#include "pdu_df.h"
#include "lll/pdu_vendor.h"
#include "pdu.h"

#include "lll.h"
#include "lll_clock.h"
#include "lll/lll_vendor.h"
#include "lll/lll_adv_types.h"
#include "lll_adv.h"
#include "lll/lll_adv_pdu.h"
#include "lll_adv_sync.h"

#include "ull_adv_types.h"

#include "ull_internal.h"
#include "ull_adv_internal.h"

#include "ll.h"

#include "hal/debug.h"

#if defined(CONFIG_BT_CTLR_ADV_PERIODIC_RSP)

/**
 * @brief Set subevent data for periodic advertising with responses, function
 *        handles one subevent, caller can call multiple times for each subevent.
 *
 * @param handle         Advertising set handle
 * @param subevent       Subevent index
 * @param response_slot_start First response slot for this subevent
 * @param response_slot_count Number of response slots for this subevent
 * @param subevent_data_len   Length of subevent data
 * @param subevent_data       Pointer to subevent data
 *
 * @return 0 on success, error code otherwise
 */
uint8_t ll_adv_sync_subevent_data_set(uint8_t handle,
				      uint8_t subevent,
				      uint8_t response_slot_start,
				      uint8_t response_slot_count,
				      uint8_t subevent_data_len,
				      uint8_t *subevent_data)
{
	struct ll_adv_sync_set *sync;
	struct ll_adv_set *adv;

	/* Get the advertising set */
	adv = ull_adv_is_created_get(handle);
	if (!adv) {
		return BT_HCI_ERR_UNKNOWN_ADV_IDENTIFIER;
	}

	/* Get sync context */
	sync = HDR_LLL2ULL(adv->lll.sync);
	if (!sync) {
		return BT_HCI_ERR_CMD_DISALLOWED;
	}

	/* Validate subevent index */
	if (subevent >= sync->num_subevents) {
		return BT_HCI_ERR_INVALID_PARAM;
	}

	/* Validate data length */
	if (subevent_data_len > CONFIG_BT_CTLR_ADV_DATA_LEN_MAX) {
		return BT_HCI_ERR_INVALID_PARAM;
	}

	/* Validate response slot parameters */
	if (response_slot_count > 0) {
		uint8_t slot_end = response_slot_start + response_slot_count;

		if (slot_end > sync->num_response_slots) {
			return BT_HCI_ERR_INVALID_PARAM;
		}
	}

	/* Store subevent data */
	sync->se_data[subevent].len = subevent_data_len;
	sync->se_data[subevent].response_slot_start = response_slot_start;
	sync->se_data[subevent].response_slot_count = response_slot_count;
	sync->se_data[subevent].is_data_set = 1U;

	if (subevent_data_len > 0 && subevent_data != NULL) {
		memcpy(sync->se_data[subevent].data, subevent_data, subevent_data_len);
	}

	return BT_HCI_ERR_SUCCESS;
}

#endif /* CONFIG_BT_CTLR_ADV_PERIODIC_RSP */
