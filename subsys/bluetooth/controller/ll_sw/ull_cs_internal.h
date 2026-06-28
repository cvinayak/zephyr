/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

int ull_cs_init(void);
int ull_cs_reset(void);

uint8_t ll_cs_read_local_supported_capabilities(
	struct bt_hci_rp_le_read_local_supported_capabilities *rp);

uint8_t ll_cs_read_remote_supported_capabilities(uint16_t handle);

uint8_t ll_cs_write_cached_remote_supported_capabilities(
	const struct bt_hci_cp_le_write_cached_remote_supported_capabilities *cmd);

uint8_t ll_cs_security_enable(uint16_t handle);

uint8_t ll_cs_set_default_settings(
	const struct bt_hci_cp_le_cs_set_default_settings *cmd);

uint8_t ll_cs_read_remote_fae_table(uint16_t handle);

uint8_t ll_cs_write_cached_remote_fae_table(
	const struct bt_hci_cp_le_write_cached_remote_fae_table *cmd);

uint8_t ll_cs_create_config(const struct bt_hci_cp_le_cs_create_config *cmd,
			    uint8_t *config_id);

uint8_t ll_cs_remove_config(uint16_t handle, uint8_t config_id);

uint8_t ll_cs_set_channel_classification(const uint8_t *channel_map);

uint8_t ll_cs_set_procedure_parameters(
	const struct bt_hci_cp_le_set_procedure_parameters *cmd);

uint8_t ll_cs_procedure_enable(uint16_t handle, uint8_t config_id,
			       uint8_t enable);

uint8_t ll_cs_test(const struct bt_hci_op_le_cs_test *cmd);

uint8_t ll_cs_test_end(void);

/* Channel Sounding ULL scheduling helpers. */

struct lll_cs_step;

/* Build the list of Channel Sounding channel indices that are both enabled in
 * the agreed channel_map and allowed for Channel Sounding. Returns the number
 * of channel indices written to channels (bounded by max).
 */
uint8_t ull_cs_filtered_channels_get(const uint8_t channel_map[10],
				     uint8_t *channels, uint8_t max);

/* Deterministically shuffle the channel index list using the procedure/event
 * counter as the seed, implementing the per-procedure channel reordering of
 * the Channel Sounding channel selection algorithm #1.
 */
void ull_cs_channels_shuffle(uint8_t *channels, uint8_t count, uint32_t seed);

/* Plan the ordered list of Channel Sounding steps of a single subevent. The
 * plan starts with mode_0_steps mode-0 steps followed by the main mode steps,
 * interleaving sub mode steps every main_mode_repetition steps when a sub mode
 * is configured. The RF channel of each step is taken in order from the
 * provided (already filtered and shuffled) channel list. Returns the number of
 * steps written to steps (bounded by max_steps).
 */
uint8_t ull_cs_subevent_steps_plan(uint8_t main_mode, uint8_t sub_mode,
				   uint8_t main_mode_steps,
				   uint8_t main_mode_repetition,
				   uint8_t mode_0_steps, uint8_t rtt_type,
				   uint8_t *access_address,
				   const uint8_t *channels,
				   uint8_t channel_count,
				   struct lll_cs_step *steps,
				   uint8_t max_steps);

/* Start periodic Channel Sounding subevent generation for the given
 * connection. Called when CS procedure is enabled after LLCP negotiation.
 */
void ull_cs_procedure_start(uint16_t handle, uint8_t config_id,
			    uint8_t *access_address);

/* Stop Channel Sounding subevent generation for the given connection. */
void ull_cs_procedure_stop(uint16_t handle);

struct lll_cs;

/* Allocate, build and enqueue a Channel Sounding subevent result notification
 * towards the ULL from the LLL radio event context. The per-step measurement
 * results captured by the LLL (RTT timestamps and IQ/phase samples) are
 * serialized into the notification so that the HCI layer can encode the LE CS
 * Subevent Result meta event towards the Host. subevent_done_status and
 * procedure_done_status carry the completion state of the reported subevent and
 * of the CS procedure for the current event. Returns 1 if a notification was
 * enqueued, or 0 if no receive buffer was available.
 */
uint8_t ull_cs_subevent_report(const struct lll_cs *lll,
			       uint8_t subevent_done_status,
			       uint8_t procedure_done_status);
