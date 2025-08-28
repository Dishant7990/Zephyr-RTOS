# Integrating TMP119 Sensor Driver in Zephyr RTOS


**Author**: Dishantpari Goswami

**Target Sensor**: Texas Instruments TMP119 (I²C-based)

**Platform**: [Zephyr RTOS](https://docs.zephyrproject.org/latest/index.html)

**Objective**: Add native support for TMP119 temperature sensor as a custom driver module in Zephyr.

## 🧱 Prerequisites

Ensure the following before starting:

* ✅ Zephyr RTOS source code cloned
* ✅ Familiarity with:

  * Device Tree (`.dts`/`.overlay`)
  * Kconfig system
  * Zephyr driver model
* ✅ TMP119 hardware connected via I²C

---

## 📁 Step 1: Create Driver Directory

```sh
cd zephyr/drivers/sensor/ti
mkdir tmp119
```

---

## 📄 Step 2: Create Required Files

Navigate to the new folder and create:

```
zephyr/drivers/sensor/ti/tmp119/
├── CMakeLists.txt
├── Kconfig
├── tmp119.h
└── tmp119.c
```

---

### 📄 CMakeLists.txt

```cmake
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(tmp119.c)
```

---

### 📄 Kconfig

```kconfig
# TMP119 temperature sensor configuration options

config TMP119
	bool "TMP119 Temperature Sensor"
	default y
	depends on DT_HAS_TI_TMP119_ENABLED
	select I2C
	help
	  Enable driver for the TMP119 temperature sensor.
```

---

### 📄 tmp119.h

```c
#ifndef ZEPHYR_DRIVERS_SENSOR_TMP119_TMP119_H_
#define ZEPHYR_DRIVERS_SENSOR_TMP119_TMP119_H_

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/util.h>

/* TMP119 Register Map */
#define TMP119_REG_TEMP         0x00
#define TMP119_REG_CFGR         0x01
#define TMP119_REG_HIGH_LIM     0x02
#define TMP119_REG_LOW_LIM      0x03
#define TMP119_REG_EEPROM_UL    0x04
#define TMP119_REG_EEPROM1      0x05
#define TMP119_REG_EEPROM2      0x06
#define TMP119_REG_TEMP_OFFSET  0x07
#define TMP119_REG_EEPROM3      0x08
#define TMP119_REG_DEVICE_ID    0x0F

/* Constants */
#define TMP119_DEVICE_ID        0x2117
#define TMP119_RESOLUTION       78125   /* In tens of uCelsius */
#define TMP119_RESOLUTION_DIV   10000000

/* Configuration bits */
#define TMP119_CFGR_AVG         (BIT(5) | BIT(6))
#define TMP119_CFGR_CONV        (BIT(7) | BIT(8) | BIT(9))
#define TMP119_CFGR_MODE        (BIT(10) | BIT(11))
#define TMP119_CFGR_DATA_READY  BIT(13)

/* EEPROM unlock flags */
#define TMP119_EEPROM_UL_UNLOCK BIT(15)
#define TMP119_EEPROM_UL_BUSY   BIT(14)

/* Averaging Modes */
#define TMP119_AVG_1_SAMPLE     0
#define TMP119_AVG_8_SAMPLES    BIT(5)
#define TMP119_AVG_32_SAMPLES   BIT(6)
#define TMP119_AVG_64_SAMPLES   (BIT(5) | BIT(6))

/* Conversion Modes */
#define TMP119_MODE_CONTINUOUS  0
#define TMP119_MODE_SHUTDOWN    BIT(10)
#define TMP119_MODE_ONE_SHOT    (BIT(10) | BIT(11))

/* Runtime driver data */
struct tmp119_data {
    uint16_t sample;
    uint16_t id;
};

/* Device configuration from devicetree */
struct tmp119_dev_config {
    struct i2c_dt_spec bus;
    uint16_t odr;
};

#endif /* ZEPHYR_DRIVERS_SENSOR_TMP119_TMP119_H_ */
```

---

### 📄 tmp119.c

```c
#define DT_DRV_COMPAT ti_tmp119

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/tmp119.h>
#include <zephyr/dt-bindings/sensor/tmp119.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include "tmp119.h"

LOG_MODULE_REGISTER(tmp119, CONFIG_SENSOR_LOG_LEVEL);

static int tmp119_reg_read(const struct device *dev, uint8_t reg,
                uint16_t *val)
{
	const struct tmp119_dev_config *cfg = dev->config;
    if(i2c_burst_read_dt(&cfg->bus, reg, (uint8_t *)val, 2) < 0) {
        LOG_ERR("%s, Failed to read register 0x%02X", dev->name, reg);
        return -EIO;
    }

    *val = sys_be16_to_cpu(*val);

	return 0;
}

static int tmp119_reg_write(const struct device *dev, uint8_t reg, uint16_t val)
{
	const struct tmp119_dev_config *cfg = dev->config;
	uint8_t tx_buf[3] = {reg, val >> 8, val & 0xFF};

	return i2c_write_dt(&cfg->bus, tx_buf, sizeof(tx_buf));
}

int tmp119_write_config(const struct device *dev, uint16_t mask, uint16_t conf)
{
    uint16_t config = 0;
    int result;

    /* Step 1: Read current configuration from TMP119 register */
    result = tmp119_reg_read(dev, TMP119_REG_CFGR, &config);
    if (result < 0) {
        LOG_ERR("Register read operation failed !");
        return result;  // Return error if read failed 
    }

    /* Step 2: Clear the bits that are to be updated using the mask */
    config &= ~mask;

    /* Step 3: Apply the new configuration values to the cleared bits */
    config |= conf;

    /* Step 4: Write updated configuration back to TMP119 register */
    return tmp119_reg_write(dev, TMP119_REG_CFGR, config);
}

int tmp119_device_id_check(const struct device *dev, uint16_t *id)
{
	int ret;

	/* Step 1: Read the device ID from TMP119 */
	ret = tmp119_reg_read(dev, TMP119_REG_DEVICE_ID, id);
	if (ret != 0) {
		LOG_ERR("%s: Failed to read Device ID register!", dev->name);
		return -EIO;
	}

	/* Step 2: Compare the read ID with the expected value */
	if (*id != TMP119_DEVICE_ID) {
		LOG_ERR("%s: Device ID mismatch! Got: 0x%04X, Expected: 0x%04X",
		        dev->name, *id, TMP119_DEVICE_ID);
		return -EINVAL;
	}

	/* Step 3: Log success */
	LOG_INF("%s: TMP119 Device ID verified successfully: 0x%04X",
	        dev->name, *id);

	return 0;
}

/* Function to initialize the sensor */
static int tmp119_init(const struct device *dev)
{
	struct tmp119_data *drv_data = dev->data;
	const struct tmp119_dev_config *cfg = dev->config;

	int ret; /* Store function return value*/
	uint16_t id;

	if (!device_is_ready(cfg->bus.bus)) {
		LOG_ERR("I2C dev %s not ready.", cfg->bus.bus->name);
		return -EINVAL;
	}

	// /* Check the Device ID */
	ret = tmp119_device_id_check(dev, &id);
	if (ret < 0) {
		return ret;
	}

	LOG_DBG("Got device ID: 0x%X", id);
	drv_data->id = id;

	ret = tmp119_write_config(dev, TMP119_CFGR_CONV, cfg->odr);
}

/* Function to fetch new sensor sample */
static int tmp119_sample_fatch(const struct device *dev, enum sensor_channel chan)
{
    struct tmp119_data *drv_data = dev->data;
    uint16_t value;
    uint16_t cfg_reg = 0;   /* Store configure register value */
    int ret;    /* Store return value */

    __ASSERT_NO_MSG(chan ==  SENSOR_CHAN_ALL || 
            chan == SENSOR_CHAN_AMBIENT_TEMP);

    /* clear sensor values */
    drv_data->sample = 0U;

    /* Make sure that a data is available */
    ret = tmp119_reg_read(dev, TMP119_REG_CFGR, &cfg_reg);
    if (ret < 0) {
        LOG_ERR("%s, Failed to read from CFGR register", dev->name);
        return ret;
    }

    if((cfg_reg & TMP119_CFGR_DATA_READY) == 0) {
        LOG_DBG("%s, No data ready", dev->name);
        return -EBUSY;
    }

    /* Get the most recent temperature measurement */
    ret = tmp119_reg_read(dev, TMP119_REG_TEMP, &value);
    if(ret < 0) {
        LOG_ERR("%s, Failed to read from TEMP register!", dev->name);
        return ret;
    }

    /* Store measurement to the driver */
    drv_data->sample = (int16_t)value;

    return 0;
}

/* Function to get sensor value */
static int tmp119_channel_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val)
{
    struct tmp119_data *drv_data = dev->data;
    int32_t tmp;

    if(chan != SENSOR_CHAN_AMBIENT_TEMP) {
        return -ENOTSUP;
    }

    /**
     * See datasheet "Temperature Results and limits" section for more
     * details on processing sample data.
    */
    tmp = ((int16_t)drv_data->sample * (int32_t)TMP119_RESOLUTION) / 10;
    val->val1 = tmp / 1000000;   /* Degrees Celsius (integer part) */
    val->val2 = tmp % 1000000;   /* Micro-degrees Celsius (fractional part) */

    return 0;
}

static int16_t tmp119_conv_value(const struct sensor_value *val)
{
	uint32_t freq_micro = sensor_value_to_micro(val);

	switch (freq_micro) {
	case 64000000: /* 1 / 15.5 ms has been rounded down */
		return TMP119_DT_ODR_15_5_MS;
	case 8000000:
		return TMP119_DT_ODR_125_MS;
	case 4000000:
		return TMP119_DT_ODR_250_MS;
	case 2000000:
		return TMP119_DT_ODR_500_MS;
	case 1000000:
		return TMP119_DT_ODR_1000_MS;
	case 250000:
		return TMP119_DT_ODR_4000_MS;
	case 125000:
		return TMP119_DT_ODR_8000_MS;
	case 62500:
		return TMP119_DT_ODR_16000_MS;
	default:
		LOG_ERR("%" PRIu32 " uHz not supported", freq_micro);
		return -EINVAL;
	}
}

/* Optional: Set sensor attributes */
static int tmp119_attr_set(const struct device *dev, enum sensor_channel chan,
                           enum sensor_attribute attr, const struct sensor_value *val)
{
	struct tmp119_data *drv_data = dev->data;
	int16_t value;
	uint16_t avg;

	if (chan != SENSOR_CHAN_AMBIENT_TEMP) {
		return -ENOTSUP;
	}

	switch ((int)attr) {
	case SENSOR_ATTR_SAMPLING_FREQUENCY:
		value = tmp119_conv_value(val);
		if (value < 0) {
			return value;
		}

		return tmp119_write_config(dev, TMP119_CFGR_CONV, value);

	case SENSOR_ATTR_OVERSAMPLING:
		/* sensor supports averaging 1, 8, 32 and 64 samples */
		switch (val->val1) {
		case 1:
			avg = TMP119_AVG_1_SAMPLE;
			break;

		case 8:
			avg = TMP119_AVG_8_SAMPLES;
			break;

		case 32:
			avg = TMP119_AVG_32_SAMPLES;
			break;

		case 64:
			avg = TMP119_AVG_64_SAMPLES;
			break;

		default:
			return -EINVAL;
		}
		return tmp119_write_config(dev, TMP119_CFGR_AVG, avg);
	case SENSOR_ATTR_TMP119_SHUTDOWN_MODE:
		return tmp119_write_config(dev, TMP119_CFGR_MODE, TMP119_MODE_SHUTDOWN);

	case SENSOR_ATTR_TMP119_CONTINUOUS_CONVERSION_MODE:
		return tmp119_write_config(dev, TMP119_CFGR_MODE, TMP119_MODE_CONTINUOUS);

	case SENSOR_ATTR_TMP119_ONE_SHOT_MODE:
		return tmp119_write_config(dev, TMP119_CFGR_MODE, TMP119_MODE_ONE_SHOT);

	default:
		return -ENOTSUP;
	}
}

/* Optional: Get sensor attributes */
static int tmp119_attr_get(const struct device *dev, enum sensor_channel chan,
                           enum sensor_attribute attr, struct sensor_value *val)
{
	uint16_t data;
	int ret;

	if (chan != SENSOR_CHAN_AMBIENT_TEMP) {
		return -ENOTSUP;
	}

	switch (attr) {
	case SENSOR_ATTR_CONFIGURATION:
		ret = tmp119_reg_read(dev, TMP119_REG_CFGR, &data);
		if (ret < 0) {
			return ret;
		}
		break;
	default:
		return -ENOTSUP;
	}

	val->val1 = data;
	val->val2 = 0;

	return 0;
}

/* Define driver API */
// static const struct sensor_driver_api tmp119_driver_api = {
//     .sample_fetch = tmp119_sample_fetch,
//     .channel_get  = tmp119_channel_get,
//    .attr_set     = tmp119_attr_set,
//    .attr_get     = tmp119_attr_get,
//};
static DEVICE_API(sensor, tmp119_driver_api) = {
    .attr_set = tmp119_attr_set,
    .attr_get = tmp119_attr_get,
    .sample_fetch = tmp119_sample_fatch,
    .channel_get = tmp119_channel_get
};

/* Device instantiation */
#define DEFINE_TMP119(inst)                                                  \
    static struct tmp119_data tmp119_data_##inst;                            \
    static const struct tmp119_dev_config tmp119_config_##inst = {          \
        .bus = I2C_DT_SPEC_INST_GET(inst),                                   \
        .odr = DT_INST_PROP(inst, odr),                                      \
    };                                                                       \
    SENSOR_DEVICE_DT_INST_DEFINE(inst, tmp119_init, NULL,                    \
                                  &tmp119_data_##inst,                       \
                                  &tmp119_config_##inst,                     \
                                  POST_KERNEL,                               \
                                  CONFIG_SENSOR_INIT_PRIORITY,              \
                                  &tmp119_driver_api);

DT_INST_FOREACH_STATUS_OKAY(DEFINE_TMP119)
```

---

## 📌 Driver Code Structure

### 🔖 Define Compatibility

```c
#define DT_DRV_COMPAT ti_tmp119
```

🔹 This links the driver to the compatible string `ti,tmp119` defined in your `.dts` or `.yaml` file.

🔹 Required for `I2C_DT_SPEC_INST_GET()` to work.

---

### ✅ Initialization

Your `tmp119_init()` must ensure:

* I²C bus is ready
* Optional: Device ID is correct (`0x2117`)
* Sensor is configured (e.g., continuous mode)

---

### 🧩 Implement API Functions

| Function Name           | Purpose                                            |
| ----------------------- | -------------------------------------------------- |
| `tmp119_init()`         | Initializes the driver and checks device readiness |
| `tmp119_sample_fetch()` | Triggers a new data read                           |
| `tmp119_channel_get()`  | Converts raw data into Zephyr `sensor_value`       |
| `tmp119_attr_set()`     | (Optional) Sets runtime attributes                 |
| `tmp119_attr_get()`     | (Optional) Gets runtime attributes                 |

---

### 🧱 Register Device Instance

Used to auto-generate driver instance(s) from Devicetree:

```c
DT_INST_FOREACH_STATUS_OKAY(DEFINE_TMP119)
```

## 📜 Step 3: Update Parent Directory Configs

Go to: `zephyr/drivers/sensor/ti`

### Update CMakeLists.txt

add the line:
```
add_subdirectory_ifdef(CONFIG_TMP119 tmp119)
```

### Update Kconfig

add the line:
```
source "drivers/sensor/ti/tmp119/Kconfig"
```
---

## 📜 Step 4: Create Device Binding YAML File

create a file at: `zephyr/dts/bindings/sensor/ti,tmp119.yaml`

### ti,tmp119.yaml
```
# SPDX-License-Identifier: Apache-2.0

description: Texas Instruments TMP119 Digital Temperature Sensor

compatible: "ti,tmp119"

include: [sensor-device.yaml, i2c-device.yaml]

properties:
  reg:
    required: true

  odr:
    type: int
    description: |
      Output data rate (ODR) configuration for TMP119.
      This is a raw register value (macro defined in driver):
        - TMP119_DT_ODR_15_5_MS  = 0x000
        - TMP119_DT_ODR_125_MS   = 0x080
        - TMP119_DT_ODR_250_MS   = 0x100
        - TMP119_DT_ODR_500_MS   = 0x180
        - TMP119_DT_ODR_1000_MS  = 0x200
        - TMP119_DT_ODR_4000_MS  = 0x280
        - TMP119_DT_ODR_8000_MS  = 0x300
        - TMP119_DT_ODR_16000_MS = 0x380
    required: false
    default: 0x200  # default to 1000ms
```
---

## 📁 Step 5: Create DT Binding Header

create a header file: `zephyr/include/zephyr/dt-bindings/sensor/tmp119.h`

```h
/**
 * Copyright (c) 2024 Vitrolife A/S
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_TI_TMP119_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_TI_TMP119_H_

/**
 * @defgroup TMP119 Texas Instruments (TI) TMP119 DT Options
 * @ingroup sensor_interface
 * @{
 */

/**
 * @defgroup TMP119_ODR Temperature output data rate
 * @{
 */
#define TMP119_DT_ODR_15_5_MS  0
#define TMP119_DT_ODR_125_MS   0x80
#define TMP119_DT_ODR_250_MS   0x100
#define TMP119_DT_ODR_500_MS   0x180
#define TMP119_DT_ODR_1000_MS  0x200
#define TMP119_DT_ODR_4000_MS  0x280
#define TMP119_DT_ODR_8000_MS  0x300
#define TMP119_DT_ODR_16000_MS 0x380
/** @} */

/** @} */

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_TI_TMP119_H_ */
```


## 📁 Step 6 : Create a file for TMP119 driver header (tmp119.h)

Path: `zephyr/include/drivers/sensor/tmp119.h`

```c
/*
 * Copyright (c) 2021 Innoseis B.V
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_SENSOR_MYTMP119_H_
#define ZEPHYR_INCLUDE_DRIVERS_SENSOR_MYTMP119_H_

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <sys/types.h>

enum sensor_attribute_tmp_119 {
	/** Turn on power saving/one shot mode */
	SENSOR_ATTR_TMP119_ONE_SHOT_MODE = SENSOR_ATTR_PRIV_START,
	/** Shutdown the sensor */
	SENSOR_ATTR_TMP119_SHUTDOWN_MODE,
	/** Turn on continuous conversion */
	SENSOR_ATTR_TMP119_CONTINUOUS_CONVERSION_MODE,
	/* Soft Reset */
	SENSOR_ATTR_TMP119_SOFT_RESET,
};

#endif /* ZEPHYR_INCLUDE_DRIVERS_SENSOR_TMP119_H_ */
```

---

## 🧾 Step 7: Initiate Device Node in Your Application Code(main.c)
```c
#define TMP119_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(ti_tmp119)

const struct device *dev = DEVICE_DT_GET(TMP119_NODE);
```


---
## 🧠 Why Use Zephyr’s Sensor API?

**Portability** — Use the same APIs (`sensor_sample_fetch()`, `sensor_channel_get()`) across all sensors

**Modularity** — Works with Zephyr shell, CLI, and middleware

**Scalability** — Easily supports multiple sensor instances

---

## 📚 References

* [Zephyr Sensor API Docs](https://docs.zephyrproject.org/latest/reference/peripherals/sensor.html)
* [TMP119 Datasheet - TI](https://www.ti.com/product/TMP119)
* [Zephyr I2C API](https://docs.zephyrproject.org/latest/reference/peripherals/i2c.html)
* [SENSOR\_DEVICE\_DT\_INST\_DEFINE](https://docs.zephyrproject.org/latest/reference/kernel/api.html#c.SENSOR_DEVICE_DT_INST_DEFINE)

