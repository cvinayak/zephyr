#!/usr/bin/env bash
# Copyright The Zephyr Project Contributors
# SPDX-License-Identifier: Apache-2.0

# Compile the iso_broadcast and iso_receive sample based applications

set -ue

: "${ZEPHYR_BASE:?ZEPHYR_BASE must be set to point to the zephyr root directory}"

source ${ZEPHYR_BASE}/tests/bsim/compile.source

test_path=tests/bsim/bluetooth/ll/bis/multi_big_drift

for role in iso_broadcast iso_receive; do
  sample_path=${ZEPHYR_BASE}/samples/bluetooth/${role}
  bsim_overlay=${ZEPHYR_BASE}/${test_path}/${role}/overlay-bsim.conf
  app=${test_path}/${role} \
    conf_file=${sample_path}/prj.conf \
    conf_overlay="${sample_path}/overlay-bt_ll_sw_split.conf;${bsim_overlay}" \
    exe_name=bs_${BOARD_TS}_${test_path}/${role}_prj_conf \
    compile
done

wait_for_background_jobs
