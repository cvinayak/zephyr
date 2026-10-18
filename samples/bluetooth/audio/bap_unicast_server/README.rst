.. zephyr:code-sample:: bluetooth_bap_unicast_server
   :name: Basic Audio Profile (BAP) Unicast Audio Server
   :relevant-api: bluetooth bt_audio bt_bap bt_pacs

   Use BAP Unicast Server functionality.

Overview
********

Application demonstrating the BAP Unicast Server functionality.
Starts advertising and awaits connection from a BAP Unicast Client.

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
   :zephyr-app: samples/bluetooth/audio/bap_unicast_server/
   :board: nrf52840dk/nrf52840
   :goals: build
   :gen-args: -DEXTRA_CONF_FILE=overlay-bt_ll_sw_split.conf

Building for an nrf5340dk
-------------------------

You can build both the application core image and an appropriate controller image for the network
core with:

.. zephyr-app-commands::
   :zephyr-app: samples/bluetooth/audio/bap_unicast_server/
   :board: nrf5340dk/nrf5340/cpuapp
   :goals: build
   :west-args: --sysbuild

If you prefer to only build the application core image, you can do so by doing instead:

.. zephyr-app-commands::
   :zephyr-app: samples/bluetooth/audio/bap_unicast_server/
   :board: nrf5340dk/nrf5340/cpuapp
   :goals: build

In that case you can pair this application core image with the
:zephyr:code-sample:`bluetooth_hci_ipc` sample
:zephyr_file:`samples/bluetooth/hci_ipc/extra-iso-bt_ll_sw_split.conf` extra configuration.

Audio input and output
----------------------

The audio input and output is selected with a Kconfig choice:

.. list-table::
   :header-rows: 1

   * - Option
     - Boards
     - Sink streams
     - Source stream
   * - ``CONFIG_USE_TLV320_AUDIO``
     - ``nrf54l15dk/nrf54l15/cpuapp`` (default)
     - TLV320 DAC, stereo
     - TLV320 microphone (``CONFIG_TLV320_MIC``)
   * - ``CONFIG_USE_USB_AUDIO``
     - ``nrf5340dk/nrf5340/cpuapp``, ``nrf5340_audio_dk/nrf5340/cpuapp`` (default)
     - USB microphone, stereo
     - USB speaker, left channel
   * - ``CONFIG_USE_SINE_AUDIO``
     - Other boards (default)
     - Counted, decoded with LC3 when enabled
     - Sine tone

The nRF54L15 has no USB device controller, so the USB headset is built for the nRF5340.

Building for an nrf5340dk as a USB headset
------------------------------------------

The left and right sink streams are decoded, upsampled to 48 kHz and sent to the PC as a stereo
microphone. Speaker audio from the PC is encoded on the source stream. The USB headset node is
defined in :file:`uac2_headset.dtsi` and included by the board overlays.

.. zephyr-app-commands::
   :zephyr-app: samples/bluetooth/audio/bap_unicast_server/
   :board: nrf5340dk/nrf5340/cpuapp
   :goals: build flash
   :west-args: --sysbuild
   :gen-args: -DCONFIG_USE_USB_AUDIO=y

Selecting the USB headset on the host
-------------------------------------

The board enumerates as a USB Audio Class 2 headset named after
``CONFIG_SAMPLE_USBD_PRODUCT``, with a 48 kHz stereo speaker and a 48 kHz stereo microphone.

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

Building for an nrf54l15dk with a TLV320 codec
----------------------------------------------

On the nrf54l15dk the application runs on the application core. The received LC3 audio is decoded,
upsampled to 48 kHz and played in stereo through a TLV320 DAC by a dedicated audio thread. Each
sink stream is played on the left or right channel based on its audio location. A single sink
stream is played on both channels.

By default the Bluetooth Controller runs on the FLPR core using the
:zephyr:code-sample:`bluetooth_hci_ipc` sample:

.. zephyr-app-commands::
   :zephyr-app: samples/bluetooth/audio/bap_unicast_server/
   :board: nrf54l15dk/nrf54l15/cpuapp
   :goals: build flash
   :west-args: --sysbuild

To build the Bluetooth Controller combined with the application on the application core, disable
the ``SB_CONFIG_NET_CORE_IMAGE_HCI_IPC`` sysbuild option:

.. zephyr-app-commands::
   :zephyr-app: samples/bluetooth/audio/bap_unicast_server/
   :board: nrf54l15dk/nrf54l15/cpuapp
   :goals: build flash
   :west-args: --sysbuild
   :gen-args: -DSB_CONFIG_NET_CORE_IMAGE_HCI_IPC=n

With ``CONFIG_TLV320_MIC``, enabled by default, the TLV320 microphone bias, PGA, AGC and ADC are
powered up. The ADC output is received on I2S SDIN, decimated to the source stream rate and
encoded on the source stream. This requires a codec with an ADC, such as the TLV320AIC3100 or
TLV320AIC3110; a TLV320DAC3100 provides silence. Used with the
:zephyr:code-sample:`bluetooth_bap_unicast_client` sample as a USB headset on a PC, the board is a
headset for calls, for example in Microsoft Teams.

The TLV320 codec is connected as follows:

* I2S20, nRF is I2S slave: MCK on P1.07, SCK on P1.06, LRCK on P1.05, SDOUT on P1.04,
  SDIN on P1.15
* I2C21: SDA on P1.11, SCL on P1.12, DAC address 0x18
* Codec reset on the ``led3`` GPIO
* Microphone on MIC1LP

The console is on ``uart30``. Connect from a BAP Unicast Client, for example a USB LE Audio
dongle used as an audio output device on a PC, and play music. Stream state changes and SDU
reception metrics, printed every 10 seconds, are shown on the console.

Building for a simulated nrf52_bsim
-----------------------------------

Similarly to how you would for real HW, you can do:

.. zephyr-app-commands::
   :zephyr-app: samples/bluetooth/audio/bap_unicast_server/
   :board: nrf52_bsim
   :goals: build
   :gen-args: -DEXTRA_CONF_FILE=overlay-bt_ll_sw_split.conf

Note this will produce a Linux executable in :file:`./build/zephyr/zephyr.exe`.
For more information, check :ref:`this board documentation <nrf52_bsim>`.

Building for a simulated nrf5340bsim
------------------------------------

.. zephyr-app-commands::
   :zephyr-app: samples/bluetooth/audio/bap_unicast_server/
   :board: nrf5340bsim/nrf5340/cpuapp
   :goals: build
   :west-args: --sysbuild
