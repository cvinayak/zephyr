# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.28.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(bsim_test_bis_multi_big_drift_iso_receive)

set(sample_path ${ZEPHYR_BASE}/samples/bluetooth/iso_receive)

target_sources(app PRIVATE
  ${sample_path}/src/main.c
  src/test_main.c
)

target_include_directories(app PRIVATE ${sample_path}/src)

zephyr_include_directories(
  ${BSIM_COMPONENTS_PATH}/libUtilv1/src/
  ${BSIM_COMPONENTS_PATH}/libPhyComv1/src/
)
