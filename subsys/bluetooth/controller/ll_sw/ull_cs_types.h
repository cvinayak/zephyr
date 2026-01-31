/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

struct ll_conn_cs_data {
	uint8_t  config_id;
	uint8_t  role_enable;
	uint8_t  cs_sync_antenna_selection;
	int8_t   max_tx_power;
	uint8_t  procedure_enable;
	uint8_t  num_config;
	uint16_t max_procedure_len;
	uint16_t min_procedure_interval;
	uint16_t max_procedure_interval;
	uint16_t max_procedure_count;
	uint8_t  min_subevent_len[3];
	uint8_t  max_subevent_len[3];
	uint8_t  tone_antenna_config_selection;
	uint8_t  phy;
	uint8_t  tx_power_delta;
	uint8_t  preferred_peer_antenna;
	uint8_t  snr_control_initiator;
	uint8_t  snr_control_reflector;
	uint8_t  remote_capabilities_available:1;
	uint8_t  remote_fae_available:1;
	uint8_t  security_enabled:1;
	int8_t   remote_fae_table[72];
	struct {
		uint8_t  num_config_supported;
		uint16_t max_consecutive_procedures_supported;
		uint8_t  num_antennas_supported;
		uint8_t  max_antenna_paths_supported;
		uint8_t  roles_supported;
		uint8_t  modes_supported;
		uint8_t  rtt_capability;
		uint8_t  rtt_aa_only_n;
		uint8_t  rtt_sounding_n;
		uint8_t  rtt_random_payload_n;
		uint16_t nadm_sounding_capability;
		uint16_t nadm_random_capability;
		uint8_t  cs_sync_phys_supported;
		uint16_t subfeatures_supported;
		uint16_t t_ip1_times_supported;
		uint16_t t_ip2_times_supported;
		uint16_t t_fcs_times_supported;
		uint16_t t_pm_times_supported;
		uint8_t  t_sw_time_supported;
		uint8_t  tx_snr_capability;
	} remote_capabilities;
	struct {
		uint8_t  create_context;
		uint8_t  main_mode_type;
		uint8_t  sub_mode_type;
		uint8_t  min_main_mode_steps;
		uint8_t  max_main_mode_steps;
		uint8_t  main_mode_repetition;
		uint8_t  mode_0_steps;
		uint8_t  role;
		uint8_t  rtt_type;
		uint8_t  cs_sync_phy;
		uint8_t  channel_map[10];
		uint8_t  channel_map_repetition;
		uint8_t  channel_selection_type;
		uint8_t  ch3c_shape;
		uint8_t  ch3c_jump;
	} config[CONFIG_BT_CTLR_CHANNEL_SOUNDING_MAX_CONFIG];
};

struct ll_cs_local_capabilities {
	uint8_t  num_config_supported;
	uint16_t max_consecutive_procedures_supported;
	uint8_t  num_antennas_supported;
	uint8_t  max_antenna_paths_supported;
	uint8_t  roles_supported;
	uint8_t  modes_supported;
	uint8_t  rtt_capability;
	uint8_t  rtt_aa_only_n;
	uint8_t  rtt_sounding_n;
	uint8_t  rtt_random_payload_n;
	uint16_t nadm_sounding_capability;
	uint16_t nadm_random_capability;
	uint8_t  cs_sync_phys_supported;
	uint16_t subfeatures_supported;
	uint16_t t_ip1_times_supported;
	uint16_t t_ip2_times_supported;
	uint16_t t_fcs_times_supported;
	uint16_t t_pm_times_supported;
	uint8_t  t_sw_time_supported;
	uint8_t  tx_snr_capability;
};
