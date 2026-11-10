.. zephyr:code-sample:: bluetooth_hci_vs_adv_report
   :name: HCI Vendor-Specific Advertising Report
   :relevant-api: bluetooth

   Use vendor-specific HCI events to receive legacy advertising reports
   with channel index information.

Overview
********

This simple application demonstrates how to enable and receive the Zephyr
vendor-specific LE Advertising Report event. This event is a clone of the
standard LE Advertising Report meta-event with an added channel index field
that indicates the advertising channel (37, 38, or 39) on which the PDU was
received.

The vendor-specific event is generated in addition to the standard event and
can be enabled at runtime using the ``BT_HCI_OP_VS_SET_ADV_REPORTS`` command.

Requirements
************

* A board with Bluetooth LE support running Zephyr's built-in Bluetooth LE
  controller (requires Zephyr-specific HCI vendor extensions; external HCI
  controllers are not supported).
* One or more nearby advertising Bluetooth LE devices.

Building and Running
********************

Build and flash the sample as follows, replacing ``<board>`` with your target
board:

.. zephyr-app-commands::
   :zephyr-app: samples/bluetooth/hci_vs_adv_report
   :board: <board>
   :goals: build flash
   :compact:

After flashing, the sample initializes Bluetooth, enables vendor-specific
advertising report events, and starts passive scanning. Advertising PDUs from
nearby devices are reported on the console with the advertising channel index.

Sample Output
*************

.. code-block:: console

   Starting VS Advertising Report sample
   Bluetooth initialized
   Scanning started
   VS adv report: peer 48:AB:CD:EF:01:23 (random) rssi -65 chan 37
   VS adv report: peer 48:AB:CD:EF:01:23 (random) rssi -67 chan 38
   VS adv report: peer DE:AD:BE:EF:CA:FE (random) rssi -72 chan 39
