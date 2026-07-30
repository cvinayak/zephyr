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
	/* Scheduling parameters negotiated between the local and remote
	 * device (Initiator and Reflector) using the LL_CS_REQ, LL_CS_RSP
	 * and LL_CS_IND PDUs. The Initiator proposes a parameter set in
	 * LL_CS_REQ, the Reflector selects acceptable values in LL_CS_RSP,
	 * and the Initiator confirms the final values (including the chosen
	 * offset) in LL_CS_IND.
	 */
	struct {
		uint16_t conn_event_count;
		uint8_t  offset_min[3];
		uint8_t  offset_max[3];
		uint8_t  offset[3];
		uint16_t max_procedure_len;
		uint16_t event_interval;
		uint8_t  subevents_per_event;
		uint16_t subevent_interval;
		uint8_t  subevent_len[3];
		uint16_t procedure_interval;
		uint16_t procedure_count;
		uint8_t  config_id;
		uint8_t  aci;
		uint8_t  preferred_peer_ant;
		uint8_t  phy;
		int8_t   pwr_delta;
		uint8_t  tx_snr_i:4;
		uint8_t  tx_snr_r:4;
		uint8_t  valid:1;
	} schedule;
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
