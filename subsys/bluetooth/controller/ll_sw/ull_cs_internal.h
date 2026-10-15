/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

int ull_cs_init(void);
int ull_cs_reset(void);

uint8_t ll_cs_read_local_supported_capabilities(
	struct bt_hci_rp_le_read_local_supported_capabilities *rp);

/* Return the local Channel Sounding capabilities used to populate the
 * LL_CS_CAPABILITIES_RSP PDU when responding to a remote-initiated
 * capabilities exchange.
 */
const struct ll_cs_local_capabilities *ull_cs_local_capabilities_get(void);

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

/* Build the list of Channel Sounding channel indices that are both enabled in
 * the agreed channel_map and allowed for Channel Sounding. Returns the number
 * of channel indices written to channels (bounded by max).
 */
uint8_t ull_cs_filtered_channels_get(const uint8_t channel_map[10],
				     uint8_t *channels, uint8_t max);

bool ull_cs_chan_is_usable(uint8_t index);

/* Start periodic Channel Sounding subevent generation for the given
 * connection. Called when CS procedure is enabled after LLCP negotiation.
 */
void ull_cs_procedure_start(uint16_t handle, uint8_t config_id,
			    uint8_t *access_address);

/* Stop Channel Sounding subevent generation for the given connection. */
void ull_cs_procedure_stop(uint16_t handle);

struct lll_cs;
struct ll_conn;

/* Allocate, build and enqueue the Channel Sounding subevent result
 * notifications towards the ULL from the LLL radio event context. The results
 * of the first step_count planned steps captured by the LLL are serialized
 * into a LE CS Subevent Result notification followed by LE CS Subevent Result
 * Continue notifications. A non-zero abort_reason is the subevent abort
 * reason of an aborted subevent. Returns 1 if the notifications were
 * enqueued, or 0 if not enough receive buffers were available.
 */
uint8_t ull_cs_subevent_report(const struct lll_cs *lll, uint8_t step_count,
			       uint8_t abort_reason);

/* Called from the LLL execution context when the CS radio event of a CS
 * procedure instance is done or was aborted before it started.
 */
void ull_cs_lll_done(struct lll_cs *lll);

/* Called from the ULL_HIGH execution context on the prepare of every ACL
 * connection event to schedule the CS procedure instance anchored to it.
 */
void ull_cs_conn_prepare(struct ll_conn *conn, uint32_t ticks_at_expire, uint32_t remainder,
			 uint16_t lazy);

int8_t ull_cs_fae_for_channel(const int8_t *fae_table, uint8_t channel_index);

void ull_cs_host_ntf(uint8_t type, uint8_t status, uint16_t handle);
