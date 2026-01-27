# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr COMPONENTS unittest REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(bluetooth_ull_prepare_pipeline)

add_subdirectory(${ZEPHYR_BASE}/tests/bluetooth/controller/common common)

target_link_libraries(testbinary PRIVATE common)

target_sources(testbinary
  PRIVATE
    src/main.c
    src/ull_prepare_impl.c
    src/test_ull_prepare_basic.c
    src/test_ull_prepare_ordering.c
    src/test_ull_prepare_iterator.c
    src/test_ull_prepare_edge_cases.c
    ${ZEPHYR_BASE}/subsys/bluetooth/controller/util/mem.c
)

# Include paths for controller sources
target_include_directories(testbinary PRIVATE
  ${ZEPHYR_BASE}/subsys/bluetooth/controller
  ${ZEPHYR_BASE}/subsys/bluetooth/controller/ll_sw
  ${ZEPHYR_BASE}/subsys/bluetooth/controller/ll_sw/nordic
  ${ZEPHYR_BASE}/subsys/bluetooth/controller/include
  ${ZEPHYR_BASE}/subsys/bluetooth/controller/util
  ${ZEPHYR_BASE}/subsys/bluetooth
  ${ZEPHYR_BASE}/include/zephyr/bluetooth
)
