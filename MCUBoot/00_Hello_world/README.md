# MCUBOOT Basic Sample with Zephyr OS (v4.2)

This guide walks you through setting up a basic MCUboot bootloader sample using Zephyr OS version 4.2 on the ESP32-S3 DevKitC board. You will learn how to prepare the project, enable MCUboot support, build, flash, and verify the firmware image using MCUMGR [UART Based].

---

## Prerequisites

- Zephyr SDK and toolchain installed  
- Zephyr OS version 4.2 (tested with v4.2.0-2290-g617b71bc174b)  
- Basic familiarity with Zephyr's `west` build system  
- Hardware: ESP32-S3 DevKitC board or equivalent  
- `mcumgr` CLI installed for image management (optional, but recommended)

---

## 📌 Step 1: Prepare the Project Directory

1. Navigate to your Zephyr samples directory:
    
    ```bash
   cd zephyr/samples/
    ```

2. Create a new sample directory by copying the existing hello_world sample:

    ```bash
    mkdir mcuboot
    cp -r hello_world/* mcuboot/
    ```

3. Create a folder for board-specific device tree overlays:

    ```bash
    mkdir -p mcuboot/boards
    ```
    
4. Add a device tree overlay file named `esp32s3_devkitc_procpu.overlay` inside mcuboot/boards/:
    
    ```dts
    &uart0 {
        current-speed = <1000000>;
        status = "okay";
    };
    ```

    This configures UART0 to operate at 1,000,000 baud, which matches the MCUboot UART transport speed.

---
## 📌 Step 2: (Optional): Modify Application Source and Configuration

You can update the sample application to use Zephyr logging for clearer output:

`src/main.c`

```c
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(mcuboot, LOG_LEVEL_INF);

int main(void) {
    while (1) {
        LOG_INF("Hello world.. mcuboot");
        k_sleep(K_SECONDS(2));
    }
    return 0;
}
```

`prj.conf`
```ini
CONFIG_LOG=y
CONFIG_UART_CONSOLE=y
CONFIG_CONSOLE=y
```

> This step is optional since the `hello_world` sample already supports console output.

---

## 📌 Step 3: Configure Build and Project Files

### 3.1 Create overlay-serial.conf

Copy the file from:
```bash
zephyr/samples/subsys/mgmt/mcumgr/smp_svr/overlay-serial.conf
```

to 

```bash
mcuboot/overlay-serial.conf
```

`Contents`:
```ini
CONFIG_CONSOLE=y
CONFIG_BASE64=y
CONFIG_MCUMGR_TRANSPORT_UART=y
CONFIG_UART_CONSOLE=y
```

### 3.2 Create sysbuild.conf

Add this file to enable MCUboot as the second-stage bootloader:
```conf
SB_CONFIG_BOOTLOADER_MCUBOOT=y
```

### 3.3 Update prj.conf

Add or replace your prj.conf with the following to enable logging, MCUboot, flash support, statistics, and MCUMGR management groups:

```ini
# Console and Logging Support
CONFIG_LOG=y                      # Enable logging support
CONFIG_UART_CONSOLE=y             # Enable UART console
CONFIG_CONSOLE=y                  # General console support

# Bootloader Configuration
CONFIG_BOOTLOADER_MCUBOOT=y      # Build MCUboot-compatible bootloader

# Flash Memory Support
CONFIG_FLASH=y                   # Flash driver support
CONFIG_FLASH_MAP=y               # Flash memory map support
CONFIG_STREAM_FLASH=y            # Enable streaming flash operations

# Stack Sizes
CONFIG_SYSTEM_WORKQUEUE_STACK_SIZE=4096  # Increased stack for workqueue
CONFIG_MAIN_STACK_SIZE=2048                # Main thread stack size

# Statistics and Monitoring
CONFIG_STATS=y                   # Enable statistics collection
CONFIG_STATS_NAMES=y             # Human-readable stats names
CONFIG_THREAD_MONITOR=y          # Thread monitoring (required by taskstat)
CONFIG_MCUMGR_GRP_OS_TASKSTAT=y # MCUMGR OS task stats group

# Core Command Support
CONFIG_NET_BUF=y                 # Network buffer support
CONFIG_ZCBOR=y                  # CBOR serialization support
CONFIG_CRC=y                    # CRC support

# MCUMGR Management Groups
CONFIG_MCUMGR=y                 # Enable MCUMGR management library
CONFIG_IMG_MANAGER=y            # Enable Image Manager for firmware updates
CONFIG_MCUMGR_GRP_IMG=y         # Enable MCUMGR image group
CONFIG_MCUMGR_GRP_OS=y          # Enable MCUMGR OS group
CONFIG_MCUMGR_GRP_STAT=y        # Enable MCUMGR stats group
```

## 📌 Step 4: Build and Flash the Firmware

Build the sample specifying the board, enabling the system build, and applying the overlay configuration explicitly:

```bash
west build -b esp32s3_devkitc/esp32s3/procpu --sysbuild -- -DOVERLAY_CONFIG=overlay-serial.conf
```

Flash the firmware to your device:

```bash
west flash
```

## 📌 Step 5: Monitor Device Output

Open a serial terminal to monitor the logs from the device. Use the correct device path (/dev/ttyUSB0 is typical on Linux) and set baud rate to 1,000,000 to match UART config:

```bash
sudo minicom -D /dev/ttyUSB0 -b 1000000
```

Expected output:

```csharp
*** Booting Zephyr OS build v4.2.0-2290-g617b71bc174b ***
[00:00:00.156,000] <inf> mcuboot: Hello world.. mcuboot
[00:00:02.156,000] <inf> mcuboot: Hello world.. mcuboot
```

## 📌 Step 6: Verify Firmware Image Using MCUMGR

Use the mcumgr command-line tool to inspect the images on your device over UART:

```bash
mcumgr --conntype serial --connstring "dev=/dev/ttyUSB0,baud=1000000" image list
```

Sample output:

```yaml
Images:
 image=0 slot=0
    version: 0.0.0
    bootable: true
    flags: active confirmed
    hash: 2b8bbad79c56e689435494a9053c060ac06a2aa62a69d5fdd596de9e157fe0ce
Split status: N/A (0)
```

- Image Fields Explanation

    - `image=0 slot=0`: Primary firmware image slot on flash

    - `version: 0.0.0`: Firmware version (default if not explicitly set)

    - `bootable`: true: Image is valid and can boot

    - `flags`: active confirmed: Image is currently running and verified as good

    - `hash`: SHA-256 checksum for image integrity

    - `Split status: N/A (0)`: Image is not split into multiple parts


## 📌 Step 7: Update Firmware with New Image and Perform MCUboot Swap

### 7.1 Update main.c to Change the Log Message

Modify the log message in main.c:
```c
LOG_INF("New Hello world.. mcuboot\n");
```

### 7.2 Build the Updated Firmware

Rebuild the project using the same build command:

```bash
west build -b esp32s3_devkitc/esp32s3/procpu --sysbuild -- -DOVERLAY_CONFIG=overlay-serial.conf
```

### 7.3 Upload the New Firmware Image

Use mcumgr to upload the signed firmware image to the secondary slot:

```bash
mcumgr --conntype serial --connstring "dev=/dev/ttyUSB0,baud=1000000" image upload build/01_mcuboot_hello_world/zephyr/zephyr.signed.bin
```

### 7.4 Verify the Firmware Images

List all firmware images on the device to check current slots and flags:

```bash
mcumgr --conntype serial --connstring "dev=/dev/ttyUSB0,baud=1000000" image list
```

Expected output:

```yaml
Images:
 image=0 slot=0
    version: 0.0.0
    bootable: true
    flags: active confirmed
    hash: 2b8bbad79c56e689435494a9053c060ac06a2aa62a69d5fdd596de9e157fe0ce
 image=0 slot=1
    version: 0.0.0
    bootable: true
    flags: 
    hash: 101e74a5bb5d529092ccddcbd869f8ff972169a6e1b8c6cb87473ca071b19fca
Split status: N/A (0)
```

### 7.5 Mark the New Image as Pending (Test Before Confirm)

Update the flags of the newly uploaded image to pending by running the image test command with the hash from slot 1:

```bash
mcumgr --conntype serial --connstring "dev=/dev/ttyUSB0,baud=1000000" image test <hash>
```

In our case:
```bash
mcumgr --conntype serial --connstring "dev=/dev/ttyUSB0,baud=1000000" image test 101e74a5bb5d529092ccddcbd869f8ff972169a6e1b8c6cb87473ca071b19fca
```

```yaml
Expected output:

Images:
 image=0 slot=0
    version: 0.0.0
    bootable: true
    flags: active confirmed
    hash: 2b8bbad79c56e689435494a9053c060ac06a2aa62a69d5fdd596de9e157fe0ce
 image=0 slot=1
    version: 0.0.0
    bootable: true
    flags: pending
    hash: 101e74a5bb5d529092ccddcbd869f8ff972169a6e1b8c6cb87473ca071b19fca
Split status: N/A (0)
```

> Note: The image in slot 1 is now flagged as pending, indicating it will be tested on the next boot.

### 7.6 Reset the Device to Boot New Image

Perform a device reset either via hardware reset button or software reset command:

```bash
mcumgr --conntype serial --connstring "dev=/dev/ttyUSB0,baud=1000000" reset
```

### 7.7 Observe New Firmware Running

Open your serial terminal again (e.g., minicom) and observe logs indicating the new firmware is running:

```csharp
*** Booting Zephyr OS build v4.2.0-2290-g617b71bc174b ***
[00:00:00.802,000] <inf> mcuboot: New Hello world.. mcuboot
[00:00:02.802,000] <inf> mcuboot: New Hello world.. mcuboot
[00:00:04.802,000] <inf> mcuboot: New Hello world.. mcuboot
[00:00:06.802,000] <inf> mcuboot: New Hello world.. mcuboot
```