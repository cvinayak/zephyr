/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

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
 * @brief Set periodic advertising parameters v2 (with PAwR support)
 *
 * @param handle      Advertising set handle
 * @param interval    Periodic advertising interval
 * @param flags       Periodic advertising properties flags
 * @param num_subevents Number of subevents
 * @param subevent_interval Subevent interval (N * 1.25ms)
 * @param response_slot_delay Response slot delay (N * 1.25ms)
 * @param response_slot_spacing Response slot spacing (N * 0.125ms)
 * @param num_response_slots Number of response slots per subevent
 *
 * @return 0 on success, error code otherwise
 */
uint8_t ll_adv_sync_param_set_v2(uint8_t handle, uint16_t interval, uint16_t flags,
				  uint8_t num_subevents, uint8_t subevent_interval,
				  uint8_t response_slot_delay, uint8_t response_slot_spacing,
				  uint8_t num_response_slots)
{
	struct ll_adv_sync_set *sync;
	struct ll_adv_set *adv;

	/* Get the advertising set */
	adv = ull_adv_is_created_get(handle);
	if (!adv) {
		return BT_HCI_ERR_UNKNOWN_ADV_IDENTIFIER;
	}

	/* Get or allocate sync context */
	sync = adv->lll.sync;
	if (!sync) {
		/* Allocate a new sync set if not already present */
		sync = sync_acquire();
		if (!sync) {
			return BT_HCI_ERR_MEM_CAPACITY_EXCEEDED;
		}

		/* Link sync to advertising set */
		adv->lll.sync = &sync->lll;
		sync->lll.adv = &adv->lll;
	}

	/* Validate parameters */
	if (num_subevents == 0 || num_subevents > BT_HCI_PAWR_SUBEVENT_MAX) {
		return BT_HCI_ERR_INVALID_PARAM;
	}

	if (num_response_slots == 0) {
		return BT_HCI_ERR_INVALID_PARAM;
	}

	/* Store PAwR parameters in sync context */
	sync->interval = interval;
	sync->num_subevents = num_subevents;
	sync->subevent_interval = subevent_interval;
	sync->response_slot_delay = response_slot_delay;
	sync->response_slot_spacing = response_slot_spacing;
	sync->num_response_slots = num_response_slots;

	/* Mark LLL as PAwR mode */
	sync->lll.is_pawr = 1;

	ARG_UNUSED(flags);

	return BT_HCI_ERR_SUCCESS;
}

/**
 * @brief Set subevent data for periodic advertising with responses
 *
 * @param handle         Advertising set handle
 * @param num_subevents  Number of subevents to set data for
 * @param subevent       Array of subevent indices
 * @param response_slot_start Array of response slot start values
 * @param response_slot_count Array of response slot count values
 * @param subevent_data_len   Array of subevent data lengths
 * @param subevent_data       Array of pointers to subevent data
 *
 * @return 0 on success, error code otherwise
 */
uint8_t ll_adv_sync_subevent_data_set(uint8_t handle, uint8_t num_subevents,
				       const uint8_t *subevent,
				       const uint8_t *response_slot_start,
				       const uint8_t *response_slot_count,
				       const uint8_t *subevent_data_len,
				       const uint8_t *const *subevent_data)
{
	struct ll_adv_sync_set *sync;
	struct ll_adv_set *adv;

	/* Get the advertising set */
	adv = ull_adv_is_created_get(handle);
	if (!adv) {
		return BT_HCI_ERR_UNKNOWN_ADV_IDENTIFIER;
	}

	/* Get sync context */
	sync = adv->lll.sync;
	if (!sync) {
		return BT_HCI_ERR_CMD_DISALLOWED;
	}

	/* Validate parameters */
	if (num_subevents == 0) {
		return BT_HCI_ERR_INVALID_PARAM;
	}

	/* TODO: Store subevent data when data structures are extended
	 * For now, we just validate and accept the command
	 */

	ARG_UNUSED(subevent);
	ARG_UNUSED(response_slot_start);
	ARG_UNUSED(response_slot_count);
	ARG_UNUSED(subevent_data_len);
	ARG_UNUSED(subevent_data);

	return BT_HCI_ERR_SUCCESS;
}

#endif /* CONFIG_BT_CTLR_ADV_PERIODIC_RSP */
