Project documentation
*********************

For the complete repository structure, build, flashing, execution, and
reproduction procedure, see the top-level ``README.md``.

.. _channel_sounding_ras_initiator:

Bluetooth: Channel Sounding Initiator with Ranging Requestor
############################################################

.. contents::
   :local:
   :depth: 2

This sample demonstrates how to use the ranging service to request ranging data from a server.
It also provides a basic distance estimation algorithm to demonstrate the IQ data handling.
The accuracy is not representative for Channel Sounding and should be replaced if accuracy is important.

Requirements
************

The sample supports the following development kits:

.. table-from-sample-yaml::

The sample also requires a device running a Channel Sounding Reflector with Ranging Responder to connect to, such as the :ref:`channel_sounding_ras_reflector` sample.

Overview
********

The sample demonstrates a Bluetooth® Low Energy Central role that acts as a
GATT Ranging Requestor client and configures the Channel Sounding initiator
role. In the three-anchor RTLS configuration, procedures are executed one at a
time under control of the host application. This avoids SoftDevice Controller
radio-scheduling conflicts between three independent initiators.

A basic distance estimation algorithm is included in the sample. Each commanded
procedure uses CS Mode 3 and therefore returns PBR and RTT observations from the
same radio event. The three-link RTLS configuration deliberately uses AA-only
RTT and the 1M PHY: the shorter exchange provides more scheduling and RF margin
than 96-bit sounding RTT while the tag maintains three simultaneous ACL links.
The mathematical representations described in `Distance estimation based on phase and amplitude information`_ and `Distance estimation based on RTT packets`_ are used as the basis for this algorithm.

User interface
**************

The sample scans for ``CS_TAG`` advertising the GATT Ranging Service UUID.
The first LED on the development kit will be lit when a connection has been established.

The UART protocol used by :file:`src/scripts/trilateration_network.py` is::

   Host -> anchor: CS_PING
   Host -> anchor: CS_MEASURE
   Anchor -> host: CS_STATUS:ANCHOR:<id>|STATE:<state>[|CODE:<errno>]
   Anchor -> host: DIST_DATA:ANCHOR:<id>|SEQ:<n>|T_MS:<ms>|METHOD:<method>|RAW_VAL:<m>|SAMPLES:<n>|QUALITY:OK|PBR_VALID:<0|1>|PBR_VAL:<m>|PBR_SAMPLES:<n>|PBR_RMSE:<rad>|RTT_VALID:<0|1>|RTT_VAL:<m>|RTT_SAMPLES:<n>|RTT_STD:<m>

``CS_MEASURE`` schedules exactly one procedure. The anchor emits ``READY``
only after the procedure has stopped, so the host can safely pass the token to
the next anchor. If all local measurement retries fail, the anchor emits
``RECOVERING`` and recreates its BLE connection and CS scheduling state. The
host advances the token so one faulty link cannot starve the other anchors.
The host parser remains compatible with the shorter legacy Mode 2 record.

The host selects the estimate used by calibration and trilateration with
``--distance-source phase``, ``rtt`` or ``fused``. In ``fused`` mode PBR is
used while it agrees with RTT within ``--fusion-guard``; otherwise RTT guards
the PBR failure. Do not reuse Mode 2 calibration coefficients for Mode 3.
The neutral :file:`src/scripts/rtls_config_mode3_raw.json` is provided for the
first comparison and characterization runs.

Building and running
********************
.. |sample path| replace:: :file:`samples/bluetooth/channel_sounding_ras_initiator`

.. include:: /includes/build_and_run.txt

Testing
=======

After programming the sample to your development kit, you can test it by connecting to another device programmed with a Channel Sounding Reflector role with Ranging Responder, such as the :ref:`channel_sounding_ras_reflector` sample.

1. |connect_terminal_specific|
#. Reset both kits.
#. Wait until the scanner detects the Peripheral.
   In the terminal window, check for information similar to the following::

      I: Filters matched. Address: XX:XX:XX:XX:XX:XX (random) connectable: 1
      I: Connecting
      I: Connected to XX:XX:XX:XX:XX:XX (random) (err 0x00)
      I: Security changed: XX:XX:XX:XX:XX:XX (random) level 2
      I: MTU exchange success (498)
      I: The discovery procedure succeeded
      I: CS capability exchange completed.
      I: CS config creation complete. ID: 0
      I: CS security enabled.
      I: CS procedures enabled.
      I: Subevent result callback 0
      I: Ranging data ready 0
      I: Ranging data get completed for ranging counter 0
      I: Estimated distance to reflector:
      I: - Round-Trip Timing method: X.XXXXX meters (derived from X samples)
      I: - Phase-Based Ranging method: X.XXXXX meters (derived from X samples)

Host RTLS and calibration
=========================

The three-anchor host application, its external geometry/calibration file, and
the step-by-step calibration workflow are in :file:`src/scripts`. See
:file:`src/scripts/CALIBRATION.md` before collecting calibration data and
:file:`src/scripts/VALIDATION.md` for the static and dynamic test protocol.

Dependencies
************

This sample uses the following |NCS| libraries:

* :ref:`dk_buttons_and_leds_readme`
* :file:`include/bluetooth/gatt_dm.h`
* :file:`include/bluetooth/services/ras.h`

This sample uses the following Zephyr libraries:

* :file:`include/sys/printk.h`
* :file:`include/zephyr/types.h`
* :ref:`zephyr:kernel_api`:

  * :file:`include/kernel.h`

* :ref:`zephyr:bluetooth_api`:

* :file:`include/bluetooth/bluetooth.h`
* :file:`include/bluetooth/conn.h`
* :file:`include/bluetooth/cs.h`
