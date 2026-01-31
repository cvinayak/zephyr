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
