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
 * @brief Set subevent data for periodic advertising with responses, function
 *        handle one subevent, caller can call multiple times for each subvent.
 *
 * @param handle         Advertising set handle
 * @param subevent       Array of subevent indices
 * @param response_slot_start Array of response slot start values
 * @param response_slot_count Array of response slot count values
 * @param subevent_data_len   Array of subevent data lengths
 * @param subevent_data       Array of pointers to subevent data
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
