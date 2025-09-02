# MCUboot

### What is MCUboot?

**MCUboot** is a secure, open-source bootloader for 32-bit microcontrollers. It is designed to verify and load application firmware from flash memory during boot time, ensuring that the firmware running on a device is authentic and trustworthy.

Developed and maintained by the **MCUboot community** (backed by projects like Zephyr, Apache Mynewt, and others), it supports secure boot, firmware upgrade, rollback, and image versioning for resource-constrained systems.

| Feature                           | Description                                                                                        |
| --------------------------------- | -------------------------------------------------------------------------------------------------- |
| **Secure Boot**                 | Verifies firmware using cryptographic signatures before booting.                                   |
| **Firmware Upgrade**           | Supports upgrading firmware images over-the-air (OTA), USB, UART, BLE, etc.                        |
| **Rollback Protection**       | Prevents downgrading to old firmware versions that might have vulnerabilities.                     |
| **Image Versioning**           | Supports versioned firmware images for tracking and upgrade decisions.                             |
|  **Multiple Image Slots**       | Uses two image slots: **primary (slot 0)** and **secondary (slot 1)** for safe upgrade strategies. |
|  **Cryptographic Verification** | RSA, ECDSA, and more via **tinycrypt** or **mbedTLS** libraries.                                   |
|  **Flexible Configuration**     | Highly configurable for different platforms, storage layouts, and update mechanisms.               |

---

### How It Works – Architecture

MCUboot typically works in the following stages during the boot process:

#### **1. MCUboot Execution**

At power-on/reset, the microcontroller's ROM or loader jumps to MCUboot located in a reserved flash region (usually the first few KBs).

#### **2. Image Validation**

MCUboot checks the primary slot (slot 0) for a valid image. It verifies:

- Signature (using public key)
- Image version
- Header and metadata

If **slot 0 is valid**, it proceeds to boot it.

#### **3. Upgrade Check (if dual-slot)**

If an image in the **secondary slot (slot 1)** is marked for upgrade:

MCUboot swaps or copies the image into slot 0

Optional: Saves a backup or keeps slot 0 intact depending on the **upgrade strategy**

After swap, it **boots into the new firmware**

#### **4. Post-Boot Confirmation**

After the application boots:

It must "**confirm**" the new image using mcumgr or an API call

If not confirmed, MCUboot will roll back on the next boot (if rollback is enabled)

---

### Image Slots

MCUboot uses a dual-slot strategy for upgrades:

| **Slot**   | **Description**                          |
| ---------- | ---------------------------------------- |
| Slot 0     | **Primary** image – always booted from.  |
| Slot 1     | **Secondary** image – used for upgrades. |


Upgrades are done via:
 - **Swap (default)**: Slot 1 image is swapped into slot 0
 - **Overwrite**: Slot 0 is erased and overwritten with Slot 1

---

### Security Model

Security is at the core of MCUboot. It enforces:

- Cryptographic signature verification
- Optional hardware-based Root of Trust (RoT)
- Secure image rollback prevention
- Immutable bootloader region

Supported Cryptographic Algorithms:

| **Algorithm**         | **Description**                    |
| --------------------- | ---------------------------------- |
| **RSA-2048**          | Strong, but larger footprint       |
| **ECDSA (secp256r1)** | Smaller keys, faster, lower memory |
| **EC256 + X25519**    | Experimental, but efficient        |
| **SHA-256**           | Standard hashing algorithm         |

---

### Integration with Zephyr & MCUmgr

In Zephyr, MCUboot is fully integrated and can be enabled via:

```ini
CONFIG_BOOTLOADER_MCUBOOT=y
```

You can manage and upgrade images using the **mcumgr** command-line tool:

```bash
mcumgr --conntype serial --connstring "dev=/dev/ttyUSB0,baud=1000000" image list

mcumgr --conntype serial --connstring "dev=/dev/ttyUSB0,baud=1000000" image upload build/sample_name/zephyr/zephyr.signed.bin

mcumgr --conntype serial --connstring "dev=/dev/ttyUSB0,baud=1000000" image test <hash>

mcumgr --conntype serial --connstring "dev=/dev/ttyUSB0,baud=1000000" reset

mcumgr --conntype serial --connstring "dev=/dev/ttyUSB0,baud=1000000" image confirm <hash>
```

Zephyr uses the sysbuild system to build both MCUboot and the application together, simplifying integration.

---

### Example Use Cases

- Secure OTA firmware upgrades for IoT devices (ESP32, STM32, Nordic, etc.)
- Remote device recovery and rollback
- Version control and audit of deployed firmware
- BLE/USB/UART-based bootloading

---

### Pros and Cons
#### ✅ Pros:

- Open source and community-driven
- Highly configurable and portable
- Secure and efficient
- Compatible with Zephyr, Mynewt, RIOT, and other RTOSes

#### ⚠️ Cons:
- Requires additional flash (dual slot)
- Needs learning curve for image management tools
- More complexity compared to a simple bootloader

---
### Useful Tools

| Tool                 | Purpose                            |
| -------------------- | ---------------------------------- |
| `west`               | Builds Zephyr apps and MCUboot     |
| `imgtool`            | Signs firmware images              |
| `mcumgr`             | Upload, list, test, reset firmware |
| `nrfutil`, `esp-idf` | Vendor-specific utilities          |
