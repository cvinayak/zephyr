.. _bluetooth_observer_decision:

Bluetooth: Observer with Decision-Based Advertising Filtering
##############################################################

Overview
********

This sample demonstrates extended scanning with decision-based advertising
filtering support as specified in Bluetooth Core Specification v6.2, section
4.6.43.

The application starts extended scanning with decision-based filtering enabled
and displays advertising reports from devices using decision-based advertising.

Requirements
************

* A board with Bluetooth Low Energy support
* Controller support for decision-based advertising filtering
  (CONFIG_BT_CTLR_DECISION_BASED_FILTERING)

Building and Running
********************

This sample can be found under
:zephyr_file:`samples/bluetooth/observer_decision` in the Zephyr tree.

Build the sample with the following commands:

.. zephyr-app-commands::
   :zephyr-app: samples/bluetooth/observer_decision
   :board: <board>
   :goals: build flash
   :compact:

To test the decision-based filtering, run the
:ref:`bluetooth_broadcaster_decision` sample on another device.

Sample Output
=============

When running, the sample will output:

.. code-block:: console

   Starting Decision-Based Advertising Observer
   Bluetooth initialized
   Starting scan with decision-based filtering
   Scanning successfully started
   Waiting for advertising reports...
   [DEVICE]: XX:XX:XX:XX:XX:XX, RSSI -45, Extended Advertising Name: Decision Broadcaster MFG: 0x05f1 Data: DECISION
