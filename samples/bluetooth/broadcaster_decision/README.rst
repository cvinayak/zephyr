.. _bluetooth_broadcaster_decision:

Bluetooth: Broadcaster with Decision-Based Advertising
#######################################################

Overview
********

This sample demonstrates extended advertising with decision-based advertising
filtering support as specified in Bluetooth Core Specification v6.2, section
4.6.43.

The application starts extended advertising and transmits advertising PDUs
that can be filtered using decision-based filtering on the scanner side.

Requirements
************

* A board with Bluetooth Low Energy support
* Controller support for decision-based advertising filtering
  (CONFIG_BT_CTLR_DECISION_BASED_FILTERING)

Building and Running
********************

This sample can be found under
:zephyr_file:`samples/bluetooth/broadcaster_decision` in the Zephyr tree.

Build the sample with the following commands:

.. zephyr-app-commands::
   :zephyr-app: samples/bluetooth/broadcaster_decision
   :board: <board>
   :goals: build flash
   :compact:

To observe the advertising with decision-based filtering, run the
:ref:`bluetooth_observer_decision` sample on another device.

Sample Output
=============

When running, the sample will output:

.. code-block:: console

   Starting Decision-Based Advertising Broadcaster
   Bluetooth initialized
   Extended advertising set created
   Advertising data set
   Extended advertising started
   Advertising with decision-based filtering support
   Device name: Decision Broadcaster
   Manufacturer data: DECISION
