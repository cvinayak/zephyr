# SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr COMPONENTS unittest REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(bluetooth_ull_cs_drbg)

target_include_directories(testbinary
  PRIVATE
    ${ZEPHYR_BASE}/subsys/bluetooth/controller
    ${ZEPHYR_BASE}/subsys/bluetooth/controller/ll_sw
)

target_sources(testbinary
  PRIVATE
    src/main.c
    src/aes.c
    ${ZEPHYR_BASE}/subsys/bluetooth/controller/ll_sw/ull_cs_drbg.c
)
