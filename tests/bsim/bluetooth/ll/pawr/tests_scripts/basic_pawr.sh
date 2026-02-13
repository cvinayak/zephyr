#!/usr/bin/env bash
# Copyright 2024 Nordic Semiconductor ASA
# SPDX-License-Identifier: Apache-2.0

source ${ZEPHYR_BASE}/tests/bsim/sh_common.source

# Basic PAwR test: one device advertises with PAwR, another syncs to it
# Tests the ULL layer implementation for Periodic Advertising with Responses
simulation_id="basic_pawr"
verbosity_level=2
EXECUTE_TIMEOUT=120

cd ${BSIM_OUT_PATH}/bin

Execute ./bs_${BOARD_TS}_tests_bsim_bluetooth_ll_pawr_prj_conf \
  -v=${verbosity_level} -s=${simulation_id} -RealEncryption=1 -d=0 -testid=pawr_adv

Execute ./bs_${BOARD_TS}_tests_bsim_bluetooth_ll_pawr_prj_conf \
  -v=${verbosity_level} -s=${simulation_id} -RealEncryption=1 -d=1 -testid=pawr_sync

Execute ./bs_2G4_phy_v1 -v=${verbosity_level} -s=${simulation_id} \
  -D=2 -sim_length=60e6 $@

wait_for_background_jobs
