.. zephyr:code-sample:: bluetooth_bap_unicast_client
   :name: Basic Audio Profile (BAP) Unicast Audio Client
   :relevant-api: bluetooth bt_audio bt_bap bt_conn

   Use BAP Unicast Client functionality.

Overview
********

Application demonstrating the BAP Unicast Client functionality. Scans for and
connects to a BAP Unicast Server and establishes an audio stream.

Check the :zephyr:code-sample-category:`bluetooth` samples for general information.

Requirements
************

* BlueZ running on the host, or
* A board with Bluetooth Low Energy 5.2 support

Building and Running
********************

When building targeting an nrf52 series board with the Zephyr Bluetooth Controller,
use ``-DEXTRA_CONF_FILE=overlay-bt_ll_sw_split.conf`` to enable the required ISO
feature support.

Building for an nrf52840dk
--------------------------

.. zephyr-app-commands::
   :zephyr-app: samples/bluetooth/audio/bap_unicast_client/
   :board: nrf52840dk/nrf52840
   :goals: build
   :gen-args: -DEXTRA_CONF_FILE=overlay-bt_ll_sw_split.conf

Building for an nrf5340dk
-------------------------

You can build both the application core image and an appropriate controller image for the network
core with:

.. zephyr-app-commands::
   :zephyr-app: samples/bluetooth/audio/bap_unicast_client/
   :board: nrf5340dk/nrf5340/cpuapp
   :goals: build
   :west-args: --sysbuild

If you prefer to only build the application core image, you can do so by doing instead:

.. zephyr-app-commands::
   :zephyr-app: samples/bluetooth/audio/bap_unicast_client/
   :board: nrf5340dk/nrf5340/cpuapp
   :goals: build

In that case you can pair this application core image with the
:zephyr:code-sample:`bluetooth_hci_ipc` sample
:zephyr_file:`samples/bluetooth/hci_ipc/extra-iso-bt_ll_sw_split.conf` extra configuration.

USB headset
-----------

On boards with a USB device controller the client appears as a USB headset to a PC:

* Speaker audio from the PC is encoded with LC3 and sent on two sink streams, front left and front
  right.
* The first source stream of the server is decoded and sent to the PC as the microphone.

.. list-table::
   :header-rows: 1

   * - Board
     - Audio
     - Sink preset
   * - ``nrf5340dk/nrf5340/cpuapp``, ``nrf5340_audio_dk/nrf5340/cpuapp``
     - USB headset (default)
     - 48_2_1
   * - ``nrf52840dk/nrf52840``
     - USB headset (default)
     - 24_2_1
   * - Other boards
     - Sine tone
     - 16_2_1

The audio input and output is selected with ``CONFIG_USE_USB_AUDIO`` or
``CONFIG_USE_SINE_AUDIO``, the sink stream preset with ``CONFIG_SINK_PRESET_48_2_1``,
``CONFIG_SINK_PRESET_24_2_1`` or ``CONFIG_SINK_PRESET_16_2_1``, and the source stream preset with
``CONFIG_SOURCE_PRESET_16_2_1`` or ``CONFIG_SOURCE_PRESET_24_2_1``. The sink streams use the
media context and the source stream the conversational context. The USB headset node is defined
in :file:`uac2_headset.dtsi` and included by the board overlays.

.. zephyr-app-commands::
   :zephyr-app: samples/bluetooth/audio/bap_unicast_client/
   :board: nrf5340dk/nrf5340/cpuapp
   :goals: build flash
   :west-args: --sysbuild
   :gen-args: -DCONFIG_USE_USB_AUDIO=y

Together with the :zephyr:code-sample:`bluetooth_bap_unicast_server` sample on an nrf54l15dk
with a TLV320 codec, the client and server form a headset for calls and music.

Selecting the USB headset on the host
-------------------------------------

The board enumerates as a USB Audio Class 2 headset named after
``CONFIG_SAMPLE_USBD_PRODUCT``, with a 48 kHz stereo speaker and a 48 kHz mono microphone.

* Windows: select the device in :menuselection:`Settings --> System --> Sound` for output and
  input.
* macOS: select the device in :menuselection:`System Settings --> Sound` on the
  :guilabel:`Output` and :guilabel:`Input` tabs.
* Linux: select the device with ``pavucontrol`` or :menuselection:`Settings --> Sound`, or with
  ``pactl set-default-sink`` and ``pactl set-default-source``.
* Microsoft Teams: select the device as :guilabel:`Speaker` and :guilabel:`Microphone` in
  :menuselection:`Settings --> Devices`.

The USB clock is declared synchronous to the USB Start of Frame. The drift between the USB and
Bluetooth clocks is corrected by dropping or repeating one sample frame when a ring buffer leaves
its fill window. USB underrun, overrun and drift correction counters are logged periodically.

Building for a simulated nrf52_bsim
-----------------------------------

Similarly to how you would for real HW, you can do:

.. zephyr-app-commands::
   :zephyr-app: samples/bluetooth/audio/bap_unicast_client/
   :board: nrf52_bsim
   :goals: build
   :gen-args: -DEXTRA_CONF_FILE=overlay-bt_ll_sw_split.conf

Note this will produce a Linux executable in :file:`./build/zephyr/zephyr.exe`.
For more information, check :ref:`this board documentation <nrf52_bsim>`.

Building for a simulated nrf5340bsim
------------------------------------

.. zephyr-app-commands::
   :zephyr-app: samples/bluetooth/audio/bap_unicast_client/
   :board: nrf5340bsim/nrf5340/cpuapp
   :goals: build
   :west-args: --sysbuild
