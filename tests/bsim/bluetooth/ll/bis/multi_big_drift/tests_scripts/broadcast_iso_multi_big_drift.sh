#!/usr/bin/env bash
# Copyright The Zephyr Project Contributors
# SPDX-License-Identifier: Apache-2.0

source ${ZEPHYR_BASE}/tests/bsim/sh_common.source

# Three iso_broadcast devices with sleep clocks drifting +100 ppm, 0 ppm and
# -100 ppm, each creating a BIG with a unique advertising SID, and one
# iso_receive device synchronizing to all three BIGs. The receiver must not
# miss any SDU once all BIS are synchronized, i.e. it has to yield and resume
# subevents of the overlapping BIG events.
# The broadcasters are started at different offsets (in microseconds,
# overridable with START_OFFSET_0/1/2) so that their transmissions are not
# aligned on air.
simulation_id="${BOARD_TS}_broadcast_iso_multi_big_drift"
verbosity_level=2
EXECUTE_TIMEOUT=${EXECUTE_TIMEOUT:-600}

# Seconds to measure after all BIS are synchronized
duration_s=${DURATION_S:-60}
sim_length_us=$(( (duration_s + 30) * 1000000 ))

cd ${BSIM_OUT_PATH}/bin

bin_prefix=bs_${BOARD_TS}_tests_bsim_bluetooth_ll_bis_multi_big_drift

Execute ./${bin_prefix}_iso_broadcast_prj_conf \
  -v=${verbosity_level} -s=${simulation_id} -d=0 -RealEncryption=1 \
  -xo_drift=100e-6 -start_offset=${START_OFFSET_0:-0}

Execute ./${bin_prefix}_iso_broadcast_prj_conf \
  -v=${verbosity_level} -s=${simulation_id} -d=1 -RealEncryption=1 \
  -xo_drift=0 -start_offset=${START_OFFSET_1:-8400}

Execute ./${bin_prefix}_iso_broadcast_prj_conf \
  -v=${verbosity_level} -s=${simulation_id} -d=2 -RealEncryption=1 \
  -xo_drift=-100e-6 -start_offset=${START_OFFSET_2:-7600}

Execute ./${bin_prefix}_iso_receive_prj_conf \
  -v=${verbosity_level} -s=${simulation_id} -d=3 -RealEncryption=1 \
  -testid=iso_receive -argstest duration_s ${duration_s}

Execute ./bs_2G4_phy_v1 -v=${verbosity_level} -s=${simulation_id} \
  -D=4 -sim_length=${sim_length_us} $@

wait_for_background_jobs
