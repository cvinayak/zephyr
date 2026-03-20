# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(observer_broadcaster_multiple)

target_sources(app PRIVATE
  src/main.c
  ../observer/src/observer.c
  ../broadcaster_multiple/src/broadcaster_multiple.c
)

if(CONFIG_BT_LL_SW_SPLIT)
  zephyr_code_relocate(LIBRARY subsys__bluetooth__controller__ll_sw__nordic LOCATION RAM NOKEEP)
endif()
