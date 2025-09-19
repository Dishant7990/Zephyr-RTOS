DFU OTA over BLE
=================

Overview
========

BLE Beacon with **MCUboot + mcumgr**

This sample demonstrates a **dynamic BLE beacon application** running on ESP32-S3
with **MCUboot bootloader** and **mcumgr over Bluetooth (SMP protocol)**.  

The device advertises:

- A complete **Bluetooth device name** (configurable via ``prj.conf``).
- Manufacturer-specific data (Espressif Company ID).
- A URI (GitHub profile link) in the scan response.  

It also supports **Device Firmware Update (DFU)** over Bluetooth using the
`nRF Connect Mobile App`_, allowing seamless upgrade of firmware without manual
flashing.

Features
========
- Connectable BLE advertising with custom device name.
- Integrated with **MCUboot** for firmware management.
- Supports **mcumgr commands** over Bluetooth and shell.
- Firmware update over BLE using **nRF Connect App**.

Project Structure
=================
.. code-block:: text

    .
    ├── boards
    │   └── esp32s3_devkitc_procpu.overlay
    ├── CMakeLists.txt
    ├── prj.conf
    ├── README.rst
    ├── src
    │   └── main.c
    └── sysbuild.conf

Building & Flashing
===================
1. Update **prj.conf** file, add data from ``zephyr/samples/subsys/mgmt/mcumgr/smp_svr/overlay-bt.conf``

2. Create ``sysbuild.conf`` file

This file enables **MCUboot** as the second-stage bootloader for DFU support:

.. code-block:: kconfig

   SB_CONFIG_BOOTLOADER_MCUBOOT=y

3. Build the application with MCUboot enabled:

.. code-block:: bash

   west build -b esp32s3_devkitc/esp32s3/procpu --sysbuild -- -DSB_CONF_FILE=sysbuild.conf

4. Flash the binary via USB/UART:

.. code-block:: bash

   west flash

After reset, the device will start advertising.

Expected Output Logs
====================

Example logs after initial flash (with device name ``Test Beacon 1.0``):

.. code-block:: text


   *** Booting Zephyr OS build v4.2.0-2290-g617b71bc174b ***
   [00:00:00.156,000] <inf> Dynamic_Beacon: Dynamic Beacon Demo Started
   [00:00:00.157,000] <inf> esp32_bt_adapter: BT controller compile version [4713a69]
   [00:00:00.157,000] <inf> esp32_bt_adapter: Feature Config, ADV:1, BLE_50:1, DTM:1, SCAN:1, CCA:0, SMP:1, CONNECT:1
   [00:00:00.193,000] <wrn> bt_hci_core: Num of Controller's ACL packets != ACL bt_conn_tx contexts (12 != 3)
   [00:00:00.194,000] <inf> bt_hci_core: HCI transport: BT ESP32
   [00:00:00.194,000] <inf> bt_hci_core: Identity: XX:XX:XX:XX:XX:XX (public)
   [00:00:00.194,000] <inf> bt_hci_core: HCI: version 5.0 (0x09) revision 0x0016, manufacturer 0x02e5
   [00:00:00.194,000] <inf> bt_hci_core: LMP: version 5.0 (0x09) subver 0x0016
   [00:00:00.194,000] <inf> Dynamic_Beacon: Bluetooth initialized
   [00:00:00.195,000] <inf> Dynamic_Beacon: Beacon started, advertising as XX:XX:XX:XX:XX:XX (public)
   [00:00:00.195,000] <inf> Dynamic_Beacon: Advertising packet size = 24
   [00:00:00.195,000] <inf> Dynamic_Beacon: Scan response packet size = 8
   [00:00:00.195,000] <inf> Dynamic_Beacon: BLE Device Name : Test Beacon 1.0
   [00:00:00.195,000] <inf> Dynamic_Beacon: Beacon active, advertising as XX:XX:XX:XX:XX:XX (public)


In the nRF Connect Mobile App, the beacon will appear as:
**Test Beacon 1.0**

Firmware Update (FOTA over BLE)
===============================

Step 1: Update prj.conf
-----------------------
Update the Bluetooth device name inside ``prj.conf``:

.. code-block:: diff

   - CONFIG_BT_DEVICE_NAME="Test Beacon 1.0"
   + CONFIG_BT_DEVICE_NAME="Test Beacon 1.1"

Step 2: Rebuild Application
---------------------------
1. Build the application with MCUboot enabled:

.. code-block:: bash

   west build -b esp32s3_devkitc/esp32s3/procpu --sysbuild -- -DSB_CONF_FILE=sysbuild.conf

2. The signed binary for DFU will be generated at:

.. code-block:: text

   build/zephyr/zephyr.signed.bin

- copy this file into Mobile phone.


Step 3: Upload Using Nordic nRF Connect App
-------------------------------------------
Follow the instructions from `Nordic_FOTA_BLE <https://academy.nordicsemi.com/courses/nrf-connect-sdk-intermediate/lessons/lesson-9-bootloaders-and-dfu-fota/topic/exercise-5-fota-over-bluetooth-low-energy/>`_  
(start from **Step 5: Upload Firmware Over BLE**).

1. Open **nRF Connect Mobile App**.
2. Scan and connect to your device (it will advertise as ``Test Beacon 1.0``).
3. Go to **DFU** section, select the file ``zephyr.signed.bin`` from the phone storage.
4. Start the update process.

Step 4: Observe Logs After Update
---------------------------------

After the update is applied and device reboots, logs will show the new name
``Test Beacon 1.1``:

.. code-block:: text

   *** Booting Zephyr OS build v4.2.0-2290-g617b71bc174b ***
   [00:00:02.246,000] <inf> Dynamic_Beacon: Dynamic Beacon Demo Started
   [00:00:02.246,000] <inf> esp32_bt_adapter: BT controller compile version [4713a69]
   [00:00:02.246,000] <inf> esp32_bt_adapter: Feature Config, ADV:1, BLE_50:1, DTM:1, SCAN:1, CCA:0, SMP:1, CONNECT:1
   [00:00:02.280,000] <wrn> bt_hci_core: Num of Controller's ACL packets != ACL bt_conn_tx contexts (12 != 3)
   [00:00:02.280,000] <inf> bt_hci_core: HCI transport: BT ESP32
   [00:00:02.280,000] <inf> bt_hci_core: Identity: XX:XX:XX:XX:XX:XX (public)
   [00:00:02.280,000] <inf> bt_hci_core: HCI: version 5.0 (0x09) revision 0x0016, manufacturer 0x02e5
   [00:00:02.280,000] <inf> bt_hci_core: LMP: version 5.0 (0x09) subver 0x0016
   [00:00:02.280,000] <inf> Dynamic_Beacon: Bluetooth initialized
   [00:00:02.281,000] <inf> Dynamic_Beacon: Beacon started, advertising as XX:XX:XX:XX:XX:XX (public)
   [00:00:02.281,000] <inf> Dynamic_Beacon: Advertising packet size = 24
   [00:00:02.281,000] <inf> Dynamic_Beacon: Scan response packet size = 8
   [00:00:02.281,000] <inf> Dynamic_Beacon: BLE Device Name : Test Beacon 1.1
   [00:00:02.282,000] <inf> Dynamic_Beacon: Beacon active, advertising as XX:XX:XX:XX:XX:XX (public)
   [00:00:03.282,000] <inf> Dynamic_Beacon: BLE Device Name : Test Beacon 1.1
   [00:00:03.282,000] <inf> Dynamic_Beacon: Beacon active, advertising as XX:XX:XX:XX:XX:XX (public)
   [00:00:04.282,000] <inf> Dynamic_Beacon: BLE Device Name : Test Beacon 1.1



References
==========
- `MCUboot Documentation <https://docs.zephyrproject.org/latest/services/device_mgmt/mcuboot.html>`_
- `mcumgr with Bluetooth SMP <https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/mcumgr/index.html>`_
- `Zephyr sysbuild (multi-image build system) <https://docs.zephyrproject.org/latest/build/sysbuild/index.html>`_
- `nRF Connect Mobile App <https://www.nordicsemi.com/Products/Development-tools/nRF-Connect-for-mobile>`_

.. _nRF Connect Mobile App: https://www.nordicsemi.com/Products/Development-tools/nRF-Connect-for-mobile