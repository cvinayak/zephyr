#!/usr/bin/env bash
# Copyright (c) 2024 Nordic Semiconductor ASA
# SPDX-License-Identifier: Apache-2.0

# Decision-based advertising filtering test:
#
# - A Bluetooth LE broadcaster advertises with extended advertising and
#   decision-based advertising filtering support, and an observer scans
#   with decision-based filtering enabled.

source ${ZEPHYR_BASE}/tests/bsim/sh_common.source

simulation_id="decision_adv"
verbosity_level=2

cd ${BSIM_OUT_PATH}/bin

Execute ./bs_${BOARD_TS}_tests_bsim_bluetooth_host_adv_decision_prj_broadcaster_conf \
  -v=${verbosity_level} -s=${simulation_id} -d=0 -RealEncryption=0 \
  -testid=decision_broadcaster -rs=23

Execute ./bs_${BOARD_TS}_tests_bsim_bluetooth_host_adv_decision_prj_observer_conf \
  -v=${verbosity_level} -s=${simulation_id} -d=1 -RealEncryption=0 \
  -testid=decision_observer -rs=6

Execute ./bs_2G4_phy_v1 -v=${verbosity_level} -s=${simulation_id} \
  -D=2 -sim_length=10e6 $@

wait_for_background_jobs
