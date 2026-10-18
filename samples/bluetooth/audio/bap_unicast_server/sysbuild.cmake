# Copyright (c) 2023 Nordic Semiconductor ASA
# SPDX-License-Identifier: Apache-2.0

if(SB_CONFIG_NET_CORE_IMAGE_HCI_IPC)
  # For builds in the nrf5340, we build the netcore image with the controller

  set(NET_APP hci_ipc)
  set(NET_APP_SRC_DIR ${ZEPHYR_BASE}/samples/bluetooth/${NET_APP})

  ExternalZephyrProject_Add(
    APPLICATION ${NET_APP}
    SOURCE_DIR  ${NET_APP_SRC_DIR}
    BOARD       ${SB_CONFIG_NET_CORE_BOARD}
  )

  set(${NET_APP}_EXTRA_CONF_FILE
   ${NET_APP_SRC_DIR}/extra-iso-bt_ll_sw_split.conf
   CACHE INTERNAL ""
  )

  native_simulator_set_child_images(${DEFAULT_IMAGE} ${NET_APP})
endif()

if(BOARD STREQUAL "nrf54l15dk" AND BOARD_QUALIFIERS MATCHES "^/?nrf54l15/cpuapp$")
  # Bluetooth Controller on FLPR using hci_ipc, or combined in the application core image
  set(app_hci_ipc_conf ${APP_DIR}/boards/nrf54l15dk_nrf54l15_cpuapp_hci_ipc.conf)
  set(app_hci_ipc_overlay ${APP_DIR}/boards/nrf54l15dk_nrf54l15_cpuapp_hci_ipc.overlay)
  set(app_ll_sw_split_conf ${APP_DIR}/overlay-bt_ll_sw_split.conf)

  set(app_extra_conf ${${DEFAULT_IMAGE}_EXTRA_CONF_FILE})
  set(app_extra_overlay ${${DEFAULT_IMAGE}_EXTRA_DTC_OVERLAY_FILE})
  list(REMOVE_ITEM app_extra_conf ${app_hci_ipc_conf} ${app_ll_sw_split_conf})
  list(REMOVE_ITEM app_extra_overlay ${app_hci_ipc_overlay})

  if(SB_CONFIG_NET_CORE_IMAGE_HCI_IPC)
    list(APPEND app_extra_conf ${app_hci_ipc_conf})
    list(APPEND app_extra_overlay ${app_hci_ipc_overlay})
  else()
    list(APPEND app_extra_conf ${app_ll_sw_split_conf})
  endif()

  set(${DEFAULT_IMAGE}_EXTRA_CONF_FILE ${app_extra_conf}
      CACHE INTERNAL "Application extra Kconfig fragments" FORCE
  )
  set(${DEFAULT_IMAGE}_EXTRA_DTC_OVERLAY_FILE ${app_extra_overlay}
      CACHE INTERNAL "Application extra DTC overlay files" FORCE
  )
endif()

native_simulator_set_final_executable(${DEFAULT_IMAGE})
