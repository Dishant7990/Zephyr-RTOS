# HDC2010 Sensor Driver in Zephyr RTOS


**Author**: Dishantpari Goswami

**Target Sensor**: Texas Instruments HDC2010 (I²C-based)

**Platform**: [Zephyr RTOS](https://docs.zephyrproject.org/latest/index.html)

**Objective**: Add native support for HDC2010 temperature & Humidity sensor as a custom driver module in Zephyr.


## 🧱 Prerequisites

Ensure the following before starting:

* ✅ Zephyr RTOS source code cloned
* ✅ Familiarity with:

  * Device Tree (`.dts`/`.overlay`)
  * Kconfig system
  * Zephyr driver model
* ✅ HDC2010 hardware connected via I²C

---

## 📁 Step 1: Create Driver Directory

```sh
cd zephyr/drivers/sensor/ti
mkdir hdc2010
```

## 📄 Step 2: Create Required Files

Navigate to the new folder and create:
```
zephyr/drivers/sensor/ti/hdc2010
├── CMakeLists.txt
├── hdc2010.c
├── hdc2010.h
├── hdc2010_trigger.c
└── Kconfig
```

---
### 📄 CMakeLists.txt

```cmake
zephyr_library()

zephyr_library_sources(hdc2010.c)
zephyr_library_sources_ifdef(CONFIG_HDC2010_TRIGGER hdc2010_trigger.c)
```

---

### 📄 Kconfig

```kconfig
menuconfig HDC2010
    bool "HDC2010 Temperature and Humidity Sensor"
    default y
    depends on DT_HAS_TI_HDC2010_ENABLED
    select I2C
    help
        Enable driver for TI HDC2010 Temperature and Humidity

if HDC2010

config HDC2010_TRIGGER
    bool "Allow interrupt to service over and under temp and humidity alerts"
    help 
        This will set up interrupt to service under and over temp and humidity
        alerts see HDC2010 spec sheet for more information on how these work.

endif   # HDC2010
```

---

### 📄 hdc2010.h
```
/**
 * @file
 * @brief TI HDC2010 Temperature and Humidity Sensor Driver
 *
 * Register definitions, configuration macros, and
 * data structures for the HDC2010 sensor.
 */
#ifndef ZEPHYR_DRIVERS_SENSOR_HDC2010_HDC2010_H_
#define ZEPHYR_DRIVERS_SENSOR_HDC2010_HDC2010_H_

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/hdc2010.h>
#include <zephyr/sys/util.h>
#include <sys/types.h>

/* ================= Register Map ================= */

#define HDC2010_REG_TEMP_HIGH            0x01  /* Temperature high byte */
#define HDC2010_REG_TEMP_LOW             0x00  /* Temperature low byte */
#define HDC2010_REG_HUMI_LOW             0x02  /* Humidity low byte */
#define HDC2010_REG_HUMI_HIGH            0x03  /* Humidity high byte */
#define HDC2010_REG_INT_DRDY             0x04  /* Interrupt/DRDY status */
#define HDC2010_REG_TEMP_MAX             0x05  /* Max temperature */
#define HDC2010_REG_HUMI_MAX             0x06  /* Max humidity */
#define HDC2010_REG_INT_ENABLE           0x07  /* Interrupt enable */
#define HDC2010_REG_TEMP_OFFSET_ADJ      0x08  /* Temperature offset adjust */
#define HDC2010_REG_HUMI_OFFSET_ADJ      0x09  /* Humidity offset adjust */
#define HDC2010_REG_TEMP_THR_HI          0x0A  /* High temp threshold */
#define HDC2010_REG_TEMP_THR_LO          0x0B  /* Low temp threshold */
#define HDC2010_REG_HUMI_THR_HI          0x0C  /* High humidity threshold */
#define HDC2010_REG_HUMI_THR_LO          0x0D  /* Low humidity threshold */
#define HDC2010_REG_CONFIG               0x0E  /* Main configuration register */
#define HDC2010_REG_MEAS_CONFIG          0x0F  /* Measurement configuration */
#define HDC2010_REG_MANUFACTURER_ID_LOW  0xFC
#define HDC2010_REG_MANUFACTURER_ID_HIGH 0xFD
#define HDC2010_REG_DEVICE_ID_LOW        0xFE
#define HDC2010_REG_DEVICE_ID_HIGH       0xFF

/* ================= Fixed ID Values ================= */

#define HDC2010_MANUFACTURER_ID_VAL 0x5449
#define HDC2010_DEVICE_ID_VAL       0x07D0

/* ================= Conversion Constants ================= */
/* From datasheet: Temp = (raw / 2^16) * 165 - 40, Humidity = (raw / 2^16) * 100 */

#define HDC2010_TEMP_SCALE_C        165U
#define HDC2010_TEMP_OFFSET_C       40U
#define HDC2010_HUMI_SCALE_PERCENT  100U
#define HDC2010_16BIT_MAX           65536U

/* ================= Status Register Bits (0x04) ================= */

#define HDC2010_DRDY_STATUS BIT(7)  /* Data ready status bit */
#define HDC2010_TH_STATUS   BIT(6)  /* Temperature high status bit */
#define HDC2010_TL_STATUS   BIT(5)  /* Temperature low status bit */
#define HDC2010_HH_STATUS   BIT(4)  /* Humidity high status bit */
#define HDC2010_HL_STATUS   BIT(3)  /* Humidity low status bit */


/** @brief HDC2010 driver data (runtime) */
struct hdc2010_data {
    uint16_t temp_sample;
    uint16_t humi_sample;
#ifdef CONFIG_HDC2010_TRIGGER
    const struct device *hdc2010_dev;
    const struct sensor_trigger *hdc2010_alert_trigger;
    sensor_trigger_handler_t hdc2010_alert_handler;
    struct gpio_callback alert_gpio_cb;
#endif // CONFIG_HDC2010_TRIGGER
};

/** @brief HDC2010 device configuration (build-time) */
struct hdc2010_dev_config {
    struct i2c_dt_spec bus;
#ifdef CONFIG_HDC2010_TRIGGER
    const struct gpio_dt_spec alert_gpios;
#endif // CONFIG_HDC2010_TRIGGER
};

void hdc2010_trigger_handler_alert(const struct device *gpio,
                                    struct gpio_callback *cb,
                                    gpio_port_pins_t pins);

int hdc2010_trigger_set(const struct device *dev,
                        const struct sensor_trigger *trig,
                        sensor_trigger_handler_t handler);
                                    
int setup_interrupts(const struct device *dev);

static int hdc2010_reg_read(const struct device *dev, uint8_t reg, uint16_t *val, uint32_t num_bytes);

static int hdc2010_reg_write(const struct device *dev, uint8_t reg,const void *value, uint32_t num_bytes);

static int hdc2010_write_config(const struct device *dev,uint8_t reg, uint16_t mask, uint16_t conf, uint32_t num_bytes);

int hdc2010_device_id_check(const struct device *dev, uint16_t *id);

int hdc2010_init(const struct device *dev);

static int hdc2010_set_auto_mode(const struct device *dev, hdc2010_amm_t mode);

static int hdc2010_attr_set(const struct device *dev, enum sensor_channel chan,
                    enum sensor_attribute attr, const struct sensor_value *val);

static int hdc2010_attr_get(const struct device *dev, enum sensor_channel chan,
                    enum sensor_attribute attr, struct sensor_value *val);

static int hdc2010_sample_fetch(const struct device *dev, enum sensor_channel chan);

#endif // ZEPHYR_DRIVERS_SENSOR_HDC2010_HDC2010_H_
```

---
###  📄 hdc2010.c

```
/**
 * @file hdc2010.c
 * @brief TI HDC2010 Humidity and Temperature Sensor Driver file
 */
#define DT_DRV_COMPAT   ti_myhdc2010

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "hdc2010.h"

LOG_MODULE_REGISTER(HDC2010, CONFIG_SENSOR_LOG_LEVEL);

static int hdc2010_reg_read(const struct device *dev, uint8_t reg, uint16_t *val, uint32_t num_bytes)
{
    const struct hdc2010_dev_config *cfg = dev->config;
    int ret;

    /* Allow only 1 or 2 byte reads */
    if(num_bytes != 1 && num_bytes != 2) {
        LOG_ERR("%s: Invalid read size (%u), only 1 or 2 bytes supported", dev->name, num_bytes);
        return -EINVAL;
    }

    /* Read raw bytes of into temporary buffer */
    uint8_t buf[2] = {0};
    ret = i2c_burst_read_dt(&cfg->bus, reg, buf, num_bytes);
    if(ret < 0) {
        LOG_ERR("%s: Failed to read %u byte from 0x%02X (err %d)", dev->name, num_bytes, reg, ret);
        return ret;
    }

    if(num_bytes == 1) {
        *(uint8_t *)val = buf[0];
    } else {
        /* Convert from little-endian to CPU order */
        *(uint16_t *)val = sys_get_le16(buf);
    }

    return 0;
}

static int hdc2010_reg_write(const struct device *dev, uint8_t reg,const void *value, uint32_t num_bytes)
{
    const struct hdc2010_dev_config *cfg = dev->config;
    uint8_t buf[3]; /* 1 byte reg + up to 2 bytes data */
    int ret;

    /* Allow only 1 or 2 byte write */
    if(num_bytes != 1 && num_bytes != 2) {
        LOG_ERR("%s: Invalid write size (%u), only 1 or 2 bytes write.", dev->name, num_bytes);
        return -EINVAL;
    }

    /* First byte is register address */
    buf[0] = reg;

    /* Copy user data after the register byte */
    memcpy(&buf[1], value, num_bytes);

    /* Perform I2C Write (Reg + Data together) */
    ret = i2c_write_dt(&cfg->bus, buf, num_bytes + 1);
    if (ret < 0) {
        LOG_ERR("%s: Failed to Write value to register 0x%02X (err = %d)", dev->name, reg, ret);
        return ret;
    }

    if (num_bytes == 1) {
	    LOG_INF("Register = 0X%04X, Value = 0X%02X", buf[0], buf[1]);
    } else {
	    LOG_INF("Register = 0X%04X, Value = 0X%02X%02X", buf[0], buf[1], buf[2]);
    }

    return 0;
}

static int hdc2010_write_config(const struct device *dev,uint8_t reg, uint16_t mask, uint16_t conf, uint32_t num_bytes)
{
    uint16_t config = 0;
    int ret;

    /* Step 1: Read current value of register */
    ret = hdc2010_reg_read(dev, reg, &config, num_bytes);
    if (ret < 0) {
        LOG_ERR("Register read operation failed (err = %d) !!", ret);
        return ret;
    }
    LOG_INF("CONFIG == 0x%04X       MASK == 0x%04X", config, mask);

    /* Step 2: Clear the bits that are to be updated using the mask */
    config &= ~mask;

    /* Step 3: Apply the new confifuration value to the cleared bits */
    config |= conf;

    /* Step 4: Write updated configuration back to register */
    ret = hdc2010_reg_write(dev, reg, &config, num_bytes);
    if(ret < 0) {
        LOG_ERR("Register write operation failed (err = %d) !!", ret);
        return ret;
    }

    LOG_INF("Updated Register value written : 0x%04X", config);

    return 0;
}

int hdc2010_device_id_check(const struct device *dev, uint16_t *id)
{
    int ret;

    ret = hdc2010_reg_read(dev, HDC2010_REG_DEVICE_ID_LOW, id, 2);
    if(ret < 0) {
        LOG_ERR("%s: Failed to read Device ID Register !! (Err = %d)", dev->name, ret);
        return ret;
    }

    if (*id != HDC2010_DEVICE_ID_VAL) {
        LOG_ERR("%s: Device ID mismatch!! Got: 0x%04X, Expected: 0x%04X", dev->name, *id, HDC2010_DEVICE_ID_VAL);
        return -EINVAL;
    }

    LOG_INF("%s: HDC2010 Device ID verified successfully: 0x%04X", dev->name, *id);

    return 0;
}

int hdc2010_init(const struct device *dev)
{
    printk("HDC2010 Initialization process.");
    struct hdc2010_data *drv_data = dev->data;
    const struct hdc2010_dev_config *cfg = dev->config;
    uint16_t id;
    int ret;

    if(!device_is_ready(cfg->bus.bus)) {
        LOG_ERR("I2C dev %s not ready", cfg->bus.bus->name);
        return -EINVAL;
    }
    
    /* Check Device id */
    ret = hdc2010_device_id_check(dev, &id);
    if (ret < 0) {
        return ret;
    }

    LOG_INF("Got Device ID : 0X%04X", id);

#ifdef CONFIG_HDC2010_TRIGGER
#warning "CONFIG_HDC2010_TRIGGER ENABLED"

    drv_data->hdc2010_dev = dev;

    ret = setup_interrupts(dev);
    if (ret < 0) {
        LOG_ERR("Failed to setup interrupt");
        return ret;
    }
#endif // CONFIG_HDC2010_TRIGGER

    return 0;
}

static int hdc2010_set_auto_mode(const struct device *dev, hdc2010_amm_t mode)
{
    int ret;
    uint8_t reg_val;

    // ret = hdc2010_reg_read(dev, HDC2010_REG_CONFIG, &reg_val, 1);
    ret = hdc2010_write_config(dev, HDC2010_REG_CONFIG, HDC2010_CONFIG_AUTO_MODE_MASK, (uint16_t)mode, 1);
    if (ret < 0) {
        LOG_ERR("%s: Failed to config auto mode. (Err = %d)", dev->name, ret);
        return ret;
    }

    return 0;
}

static int hdc2010_attr_set(const struct device *dev, enum sensor_channel chan,
                    enum sensor_attribute attr, const struct sensor_value *val)
{
    struct hdc2010_data *drv_data = dev->data;
    int16_t value = 0;
    int ret;
    
    if(chan != SENSOR_CHAN_AMBIENT_TEMP && chan != SENSOR_CHAN_ALL && chan != SENSOR_CHAN_HUMIDITY) {
        return -ENOTSUP;
    }

    switch ((int)attr) {
    case SENSOR_ATTR_SAMPLING_FREQUENCY:
	    LOG_INF("\nSENSOR_ATTR_SAMPLING_FREQUENCY");
	    ret = hdc2010_set_auto_mode(dev, val->val1);
	    break;

    case SENSOR_ATTR_HDC2010_INT_EN:
        LOG_INF("\nSENSOR_ATTR_HDC2010_INT_EN");
	    ret = hdc2010_write_config(dev, HDC2010_REG_CONFIG,
		    (HDC2010_CONFIG_DRDY_INT_EN | HDC2010_CONFIG_INT_POL | HDC2010_CONFIG_INT_MODE),
		    (uint16_t)val->val1, 1);
        
        // Temporary 
        ret = hdc2010_write_config(dev, HDC2010_REG_CONFIG, 0XFF, 0X57, 1);
        break;

    case SENSOR_ATTR_RESOLUTION:
	    LOG_INF("\nSENSOR_ATTR_RESOLUTION");
	    ret = hdc2010_write_config(dev, HDC2010_REG_MEAS_CONFIG,
				 (HDC2010_TEMP_RES_MASK | HDC2010_HUMI_RES_MASK), (uint16_t)val->val1, 1);
        k_sleep(K_MSEC(20));    /* Delay for conversion finish */
	    break;

    case SENSOR_ATTR_HDC2010_MEAS_MODE:
	    LOG_INF("\nSENSOR_ATTR_HDC2010_MEAS_MODE");
	    ret = hdc2010_write_config(dev, HDC2010_REG_MEAS_CONFIG, HDC2010_MEAS_MODE_MASK, (uint16_t)val->val1, 1);
        break;

    case SENSOR_ATTR_HDC2010_SOFT_RESET:
	    LOG_INF("\nSENSOR_ATTR_HDC2010_SOFT_RESET");
	    ret = hdc2010_write_config(dev, HDC2010_REG_CONFIG, HDC2010_CONFIG_SOFT_RESET, (uint16_t)val->val1, 1);
	    break;

    case SENSOR_ATTR_HDC2010_START_MEAS:
        LOG_INF("\nSENSOR_ATTR_HDC2010_START_MEAS");
	    ret = hdc2010_write_config(dev, HDC2010_REG_MEAS_CONFIG, HDC2010_MEAS_TRIG, (uint16_t)val->val1, 1);
        break;


#ifdef CONFIG_HDC2010_TRIGGER
    case SENSOR_ATTR_ALERT:
	    LOG_INF("\nSENSOR_ATTR_ALERT");
	    ret = hdc2010_write_config(dev, HDC2010_REG_INT_ENABLE, 0XF8, (uint16_t)val->val1, 1);
        if (ret < 0) {
            return ret;
        }
	    break;

    case SENSOR_ATTR_HDC2010_ALERT_POLARITY:
        LOG_INF("\nSENSOR_ATTR_HDC2010_ALERT_POLARITY");
        ret = hdc2010_write_config(dev, HDC2010_REG_CONFIG, HDC2010_CONFIG_INT_POL, (uint16_t)val->val1, 1);
        break;

    case SENSOR_ATTR_HDC2010_ALERT_MODE:
        LOG_INF("\nSENSOR_ATTR_HDC2010_ALERT_MODE");
        ret = hdc2010_write_config(dev, HDC2010_REG_CONFIG, HDC2010_CONFIG_INT_MODE, (uint16_t)val->val1, 1);
        break;
    
#endif // CONFIG_HDC2010_TRIGGER

    default:
        LOG_ERR("Invalid Sensor Attribute !!!");
        return -EINVAL;
    }

    return ret;
}

static int hdc2010_attr_get(const struct device *dev, enum sensor_channel chan,
                    enum sensor_attribute attr, struct sensor_value *val)
{
    struct hdc2010_data *drv_data = (struct hdc2010_data *)dev->data;
    uint8_t value;
    int ret;

    if(chan != SENSOR_CHAN_AMBIENT_TEMP && chan != SENSOR_CHAN_HUMIDITY && chan != SENSOR_CHAN_ALL) {
        LOG_ERR("Not valid channel");
        return -ENOTSUP;
    }

    switch((int)attr) {
        case SENSOR_ATTR_CONFIGURATION:
            ret = hdc2010_reg_read(dev, HDC2010_REG_CONFIG, (uint16_t *)&value, 1);

            val->val1 = (int32_t)value;
            val->val2 = 0;
            break;

        default:
            LOG_ERR("Not valid attribute");
            return -ENOTSUP;
    }

    return 0;
}

static int hdc2010_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
    struct hdc2010_data *drv_data = dev->data;
    uint16_t temp, humi;
    int ret;

    if (chan != SENSOR_CHAN_AMBIENT_TEMP &&
        chan != SENSOR_CHAN_HUMIDITY &&
        chan != SENSOR_CHAN_ALL) {
        LOG_ERR("Not valid sensor channel.");
        return -ENOTSUP;
    }

    /* Read raw temperature (2 bytes) */
    ret = hdc2010_reg_read(dev, HDC2010_REG_TEMP_LOW, &temp, sizeof(temp));
    if (ret < 0) {
        LOG_ERR("Failed to read temperature raw data !!");
        return ret;
    }

    /* Read raw humidity (2 bytes) */
    ret = hdc2010_reg_read(dev, HDC2010_REG_HUMI_LOW, &humi, sizeof(humi));
    if (ret < 0) {
        LOG_ERR("Failed to read humidity raw data !!");
        return ret;
    }

    drv_data->temp_sample = sys_le16_to_cpu(temp);
    drv_data->humi_sample = sys_le16_to_cpu(humi);

    return 0;
}

static int hdc2010_channel_get(const struct device *dev,
                                enum sensor_channel chan,
                                struct sensor_value *val)
{
    struct hdc2010_data *drv_data = dev->data;
    int64_t temp_micro, humi_micro;

    if (chan != SENSOR_CHAN_AMBIENT_TEMP && chan != SENSOR_CHAN_HUMIDITY) {
        LOG_ERR("Invalid Sensor channel.");
        return -EINVAL;
    }

    switch (chan) {
    case SENSOR_CHAN_AMBIENT_TEMP:
        /* Temp(µ°C) = (raw * 165000000 / 65536) - 40000000 */
        temp_micro = ((int64_t)drv_data->temp_sample * 165000000LL) / HDC2010_16BIT_MAX;
        temp_micro -= 40000000LL;

        val->val1 = temp_micro / 1000000;   /* integer °C */
        val->val2 = temp_micro % 1000000;   /* fractional part in µ°C */
        break;

    case SENSOR_CHAN_HUMIDITY:
        /* RH(µ%) = (raw * 1000000) / 65536 */
        humi_micro = ((int64_t)drv_data->humi_sample * 1000000LL) / HDC2010_16BIT_MAX;

        val->val1 = humi_micro / 1000000;   /* integer % */
        val->val2 = humi_micro % 1000000;   /* fractional part in µ% */
        break;

    default:
        return -ENOTSUP;
    }

    return 0;
}

static DEVICE_API(sensor, hdc2010_driver_api) = {
    .attr_set = hdc2010_attr_set,
    .attr_get = hdc2010_attr_get,
    .sample_fetch = hdc2010_sample_fetch,
    .channel_get = hdc2010_channel_get,
#ifdef CONFIG_HDC2010_TRIGGER
    .trigger_set = hdc2010_trigger_set,
#endif // CONFIG_HDC2010_TRIGGER
};

#define DEFINE_HDC2010(inst)                                                \
    static struct hdc2010_data hdc2010_data_##inst;                         \
    static const struct hdc2010_dev_config hdc2010_config_##inst = {        \
        .bus = I2C_DT_SPEC_INST_GET(inst),                                  \
        IF_ENABLED(CONFIG_HDC2010_TRIGGER,                                  \
                (.alert_gpios = GPIO_DT_SPEC_INST_GET(inst, alert_gpios)))  \
    };                                                                      \
                                                                            \
    SENSOR_DEVICE_DT_INST_DEFINE(inst, hdc2010_init, NULL,                  \
        &hdc2010_data_##inst, &hdc2010_config_##inst, POST_KERNEL,          \
        CONFIG_SENSOR_INIT_PRIORITY, &hdc2010_driver_api);                  \
                                                                            
DT_INST_FOREACH_STATUS_OKAY(DEFINE_HDC2010)
```

###  📄 hdc2010_trigger.c
```
/**
 * @file hdc2010_trigger.c
 * @brief HDC2010 Trigger & Interrupt Handling
 */

#ifdef CONFIG_HDC2010_TRIGGER

#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include "hdc2010.h"

#warning "HDC2010 Trigger Added"

LOG_MODULE_DECLARE(HDC2010);

/**
 * @brief GPIO interrupt callback handler for ALERT pin
 */
void hdc2010_trigger_handler_alert(const struct device *gpio,
                                   struct gpio_callback *cb,
                                   gpio_port_pins_t pins)
{
    struct hdc2010_data *drv_data =
        CONTAINER_OF(cb, struct hdc2010_data, alert_gpio_cb);

    LOG_INF("====== hdc2010_trigger_handler_alert ========");
    if (drv_data->hdc2010_alert_handler) {
        drv_data->hdc2010_alert_handler(drv_data->hdc2010_dev, drv_data->hdc2010_alert_trigger);
    }
}

/**
 * @brief Set trigger and callback handler
 */
int hdc2010_trigger_set(const struct device *dev,
                        const struct sensor_trigger *trig,
                        sensor_trigger_handler_t handler)
{
    struct hdc2010_data *drv_data = dev->data;

    if (trig->type == SENSOR_TRIG_DATA_READY) {
        drv_data->hdc2010_alert_handler = handler;
        drv_data->hdc2010_alert_trigger = trig;
        return 0;
    }

    return -ENOTSUP;
}

/**
 * @brief Configure and enable ALERT pin interrupts
 */
int setup_interrupts(const struct device *dev)
{
    struct hdc2010_data *drv_data = dev->data;
    const struct hdc2010_dev_config *config = dev->config;
    const struct gpio_dt_spec *alert_gpio = &config->alert_gpios;
    int ret;

    if (!device_is_ready(alert_gpio->port)) {
        LOG_ERR("HDC2010: GPIO controller %s not ready", alert_gpio->port->name);
        return -ENODEV;
    }

    ret = gpio_pin_configure_dt(alert_gpio, GPIO_INPUT);
    if (ret < 0) {
        LOG_ERR("Failed to configure GPIO pin as input.");
        return ret;
    }

    gpio_init_callback(&drv_data->alert_gpio_cb,
                       hdc2010_trigger_handler_alert,
                       BIT(alert_gpio->pin));

    ret = gpio_add_callback(alert_gpio->port, &drv_data->alert_gpio_cb);
    if (ret < 0) {
        LOG_ERR("Failed to add GPIO callback function");
        return ret;
    }

    ret = gpio_pin_interrupt_configure_dt(alert_gpio, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure GPIO interrupt");
        return ret;
    }

    LOG_INF("HDC2010 interrupt initialized successfully");
    return 0;
}

#endif /* CONFIG_HDC2010_TRIGGER */
```

## 📜 Step 3: Update Parent Directory Configs

Go to: `zephyr/drivers/sensor/ti`

### Update CMakeLists.txt

add the line:
```
add_subdirectory_ifdef(CONFIG_HDC2010 hdc2010)
```

### Update Kconfig

add the line:
```
source "drivers/sensor/ti/hdc2010/Kconfig"
```

## 📜 Step 4: Create Device Binding YAML File

file already there at: `zephyr/dts/bindings/sensor/ti,hdc2010.yaml`

### 📄 ti,hdc2010.yaml 
```
# Copyright (c) 2021, Aurelien Jarno
# SPDX-License-Identifier: Apache-2.0

description: Texas Instruments HDC2010 Temperature and Humidity Sensor

compatible: "ti,hdc2010"

include: ti,hdc20xx.yaml

properties:
  alert-gpios:
    type: phandle-array
    description: |
      Identifies the ALERT signal, which is active-low open drain when
      produced by the sensor.
```

---

## 📁 Step 5: Create DT Binding Header

create a header file: `zephyr\include\zephyr\drivers\sensor\hdc2010.h`

```
/**
 * @file hdc2010.h
 * @brief Extended public API for TI's HDC2010 Temperature & Humidity Sensor
 * 
 * This exposes attributes for the hdc2010 which can be used for
 * setting the on-chip Temperature and Humidity Mode and alert
 * parameters.
 */
#ifndef ZEPHYR_INCLUDE_DRIVERS_SENSOR_HDC2010_H_
#define ZEPHYR_INCLUDE_DRIVERS_SENSOR_HDC2010_H_

#include <zephyr/sys/util.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ================= Interrupt Configure Register Bits (0x07) ================= */

/* Data Ready interrupt enable (0 = disable, 1 = enable) */
#define HDC2010_DRDY_ENABLE   BIT(7)

/* Temperature High threshold interrupt enable (0 = disable, 1 = enable) */
#define HDC2010_TH_ENABLE     BIT(6)

/* Temperature Low threshold interrupt enable (0 = disable, 1 = enable) */
#define HDC2010_TL_ENABLE     BIT(5)

/* Humidity High threshold interrupt enable (0 = disable, 1 = enable) */
#define HDC2010_HH_ENABLE     BIT(4)

/* Humidity Low threshold interrupt enable (0 = disable, 1 = enable) */
#define HDC2010_HL_ENABLE     BIT(3)

/* ================= Config Register Bits (0x0E) ================= */

/* Soft Reset */
#define HDC2010_CONFIG_SOFT_RESET BIT(7)

/* Auto Measurement Mode (AMM) Bit[6:4] */
#define HDC2010_CONFIG_AUTO_MODE_MASK (BIT(6) | BIT(5) | BIT(4))

/* HDC2010 Auto Measurement Mode (AMM) rates. */
typedef enum {
    HDC2010_AMM_DISABLED   = 0,                             /* Manual trigger via I²C   */
    HDC2010_AMM_1_120HZ    = BIT(4),                        /* 1 sample every 2 minutes */
    HDC2010_AMM_1_60HZ     = BIT(5),                        /* 1 sample every 1 minute   */
    HDC2010_AMM_0_1HZ      = (BIT(4) | BIT(5)),             /* 1 sample every 10 seconds */
    HDC2010_AMM_0_2HZ      = BIT(6),                        /* 1 sample every 5 seconds  */
    HDC2010_AMM_1HZ        = (BIT(6) | BIT(4)),             /* 1 sample every second */
    HDC2010_AMM_2HZ        = (BIT(6) | BIT(5)),             /* 2 samples every second */
    HDC2010_AMM_5HZ        = (BIT(6) | BIT(5) | BIT(4)),    /* 5 samples every second */
} hdc2010_amm_t;

#define HDC2010_CONFIG_HEATER_ENABLE BIT(3) /* 0: Heater off, 1: Heater on */
#define HDC2010_CONFIG_DRDY_INT_EN   BIT(2) /* 0: High Z, 1: Enable */
#define HDC2010_CONFIG_INT_POL       BIT(1) /* 0: Active Low, 1: Active High */
#define HDC2010_CONFIG_INT_MODE      BIT(0) /* 0: Level, 1: Comparator */

/* ================= Measurement Configuration Register (0x0F) ================= */

/* -------- Temperature Resolution bits [7:6] -------- */
#define HDC2010_TEMP_RES_MASK  (BIT(7) | BIT(6))
#define HDC2010_TEMP_RES_14BIT 0      /* 14-bit resolution (default) */
#define HDC2010_TEMP_RES_11BIT BIT(6) /* 11-bit resolution */
#define HDC2010_TEMP_RES_9BIT  BIT(7) /* 9-bit resolution */

/* -------- Humidity Resolution bits [5:4] -------- */
#define HDC2010_HUMI_RES_MASK  (BIT(5) | BIT(4))
#define HDC2010_HUMI_RES_14BIT 0      /* 14-bit resolution (default) */
#define HDC2010_HUMI_RES_11BIT BIT(4) /* 11-bit resolution */
#define HDC2010_HUMI_RES_9BIT  BIT(5) /* 9-bit resolution */

/* -------- Measurement Configuration bits [2:1] -------- */
#define HDC2010_MEAS_MODE_MASK      (BIT(2) | BIT(1))
#define HDC2010_MEAS_MODE_HUMI_TEMP 0      /* Measure Humidity + Temperature (default) */
#define HDC2010_MEAS_MODE_TEMP_ONLY BIT(1) /* Measure Temperature only */

/* -------- Measurement Trigger bit [0] -------- */
#define HDC2010_MEAS_TRIG BIT(0) /* 1 = Start measurement (self-clearing) */


#define HDC2010_REG_MASK    0xFF

enum sensor_attribute_hdc2010 {
    /* Soft reset sensor configuration */
    SENSOR_ATTR_HDC2010_SOFT_RESET = SENSOR_ATTR_PRIV_START,
    /* Enable/disable sensor measurement mode */
    SENSOR_ATTR_HDC2010_MEAS_MODE,
    /** Set the alert pin polarity */
	SENSOR_ATTR_HDC2010_ALERT_POLARITY,
    /* Set the alert pin mode */
	SENSOR_ATTR_HDC2010_ALERT_MODE,
    /* Start measurement */
    SENSOR_ATTR_HDC2010_START_MEAS,
    /* Sensor Interrupt enable */
    SENSOR_ATTR_HDC2010_INT_EN,
};

#ifdef __cplusplus
}
#endif

#endif // ZEPHYR_INCLUDE_DRIVERS_SENSOR_HDC2010_H_
```

---

## 🧾 Step 6: Initiate Device Node in Your Application Code(main.c)
```c
#define HDC2010_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(ti_hdc2010)

const struct device *dev = DEVICE_DT_GET(HDC2010_NODE);
```

---
## 🧠 Why Use Zephyr’s Sensor API?

**Portability** — Use the same APIs (`sensor_sample_fetch()`, `sensor_channel_get()`) across all sensors

**Modularity** — Works with Zephyr shell, CLI, and middleware

**Scalability** — Easily supports multiple sensor instances

---

## 📚 References

* [Zephyr Sensor API Docs](https://docs.zephyrproject.org/latest/reference/peripherals/sensor.html)
* [HDC2010 Datasheet - TI](https://www.ti.com/lit/ds/symlink/hdc2010.pdf?ts=1756358381346&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FHDC2010%253Futm_source%253Dgoogle%2526utm_medium%253Dcpc%2526utm_campaign%253Dasc-null-null-GPN_EN-cpc-pf-google-ww_en_cons%2526utm_content%253DHDC2010%2526ds_k%253DHDC2010+Datasheet%2526DCM%253Dyes%2526gclsrc%253Daw.ds%2526gad_source%253D1%2526gad_campaignid%253D14388345080%2526gbraid%253D0AAAAAC068F3qzqpQchdVW_8_cYD2c2jAk%2526gclid%253DCjwKCAjw2brFBhBOEiwAVJX5GDhNKrAb8JrMZadJh87mOUTrbaMggxjoV7UInDtb2Rdxor09qPHGKRoClLkQAvD_BwE)
* [Zephyr I2C API](https://docs.zephyrproject.org/latest/reference/peripherals/i2c.html)
* [SENSOR\_DEVICE\_DT\_INST\_DEFINE](https://docs.zephyrproject.org/latest/reference/kernel/api.html#c.SENSOR_DEVICE_DT_INST_DEFINE)