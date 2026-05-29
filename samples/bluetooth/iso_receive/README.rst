.. zephyr:code-sample:: bluetooth_isochronous_receiver
   :name: Synchronized Receiver
   :relevant-api: bt_iso bluetooth

   Use Bluetooth LE Synchronized Receiver functionality.

Overview
********

A simple application demonstrating the Bluetooth Low Energy Synchronized
Receiver functionality. The sample can be configured to synchronize to
one or more iso_broadcast devices simultaneously using the
``CONFIG_ISO_BROADCAST_SOURCE_COUNT`` Kconfig option.

Requirements
************

* BlueZ running on the host, or
* A board with Bluetooth Low Energy 5.2 support
* A Bluetooth Controller and board that supports setting
  CONFIG_BT_CTLR_SYNC_ISO=y

Building and Running
********************

Use ``-DEXTRA_CONF_FILE=overlay-bt_ll_sw_split.conf`` to enable
required ISO feature support in Zephyr Bluetooth Controller on supported boards.

Use the sample found under :zephyr_file:`samples/bluetooth/iso_broadcast` on
another board that will start periodic advertising, create BIG to which this
sample will establish periodic advertising synchronization and synchronize to
the Broadcast Isochronous Stream.

Multiple Broadcast Sources
==========================

To synchronize to multiple iso_broadcast devices, set the
``CONFIG_ISO_BROADCAST_SOURCE_COUNT`` Kconfig option to the desired number of
sources. When synchronizing to multiple sources, the following Kconfig options
must also be adjusted:

* ``CONFIG_BT_ISO_MAX_CHAN`` must be at least
  ``2 * CONFIG_ISO_BROADCAST_SOURCE_COUNT``
* ``CONFIG_BT_PER_ADV_SYNC_MAX`` must be at least
  ``CONFIG_ISO_BROADCAST_SOURCE_COUNT``
* For the Zephyr Bluetooth Controller, ``CONFIG_BT_CTLR_SYNC_ISO_STREAM_MAX``
  and ``CONFIG_BT_CTLR_ISOAL_SINKS`` must also be increased accordingly.

For example, to synchronize to 2 broadcast sources::

   CONFIG_ISO_BROADCAST_SOURCE_COUNT=2
   CONFIG_BT_ISO_MAX_CHAN=4
   CONFIG_BT_PER_ADV_SYNC_MAX=2

See :zephyr:code-sample-category:`bluetooth` samples for details.
