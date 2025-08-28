# Integrating TMP119 Sensor Driver [Interrupt mode]in Zephyr RTOS


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
mkdir mytmp119
```

---

## 📄 Step 2: Create Required Files

Navigate to the new folder and create:

```
zephyr/drivers/sensor/ti/mytmp119/
├── CMakeLists.txt
├── Kconfig
├── tmp119.h
└── tmp119.c
└── tmp119_trigger.c
```

---

### 📄 CMakeLists.txt

```cmake
zephyr_library()

zephyr_library_sources(tmp119.c)
zephyr_library_sources_ifdef(CONFIG_TMP119_ALERT_INTERRUPTS tmp119_trigger.c)
```

---

### 📄 Kconfig

```kconfig
# TMP116 temperature sensor configuration options

config MYTMP119
	bool "TMP119 Temperature Sensors"
	default y
	depends on DT_HAS_TI_MYTMP119_ENABLED
	select I2C
	help
	  Enable driver for TMP119 temperature sensors.

if MYTMP119

config TMP119_ALERT_INTERRUPTS
    bool "Allow interrupts to service over and under temp alerts"
    help
        This will set up interrupts to service under and over temp alerts
        see TMP119 spec sheet for more information on how these work.

endif # MYTMP119
```

---

### 📄 tmp119.h

```c
#ifndef ZEPHYR_DRIVERS_SENSOR_MYTMP119_TMP119_H_
#define ZEPHYR_DRIVERS_SENSOR_MYTMP119_TMP119_H_

#include <zephyr/sys/util_macro.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>

#define TMP119_REG_TEMP         0x0
#define TMP119_REG_CFGR         0x1
#define TMP119_REG_HIGH_LIM     0x2
#define TMP119_REG_LOW_LIM      0x3
#define TMP119_REG_EEPROM_UL    0x4
#define TMP119_REG_EEPROM1      0x5
#define TMP119_REG_EEPROM2      0x6
#define TMP119_REG_TEMP_OFFSET  0x7
#define TMP119_REG_EEPROM3      0x8
#define TMP119_REG_DEVICE_ID    0xF

#define TMP119_RESOLUTION       78125   /* In tens of uCelsius */
#define TMP119_RESOLUTION_DIV   10000000

#define TMP119_DEVICE_ID        0x2117

#define TMP119_SOFT_RESET       BIT(1)
#define TMP119_CFGR_AVG         (BIT(5) | BIT(6))
#define TMP119_CFGR_CONV        (BIT(7) | BIT(8) | BIT(9))
#define TMP119_CFGR_MODE        (BIT(10) | BIT(11))
#define TMP119_CFGR_DATA_READY  BIT(13)

#ifdef CONFIG_TMP119_ALERT_INTERRUPTS

/* Alert mode */
#define TMP119_CFGR_ALERT_MODE  (BIT(2))  // DR/Alert = 1, others 0 (POL = 0(Active low), T/nA = 0)
#define TMP119_CFGR_ALERT_MODE_ACTIVE_HIGH  (BIT(2) | BIT(3))  // POL = 1 

#endif /* CONFIG_TMP119_ALERT_INTERRUPTS */

/* Thermal mode */
#define TMP119_CFGR_THERMAL_MODE BIT(4)

#define TMP119_EEPROM_UL_UNLOCK BIT(15)
#define TMP119_EEPROM_UL_BUSY   BIT(14)

#define TMP119_AVG_1_SAMPLE     0
#define TMP119_AVG_8_SAMPLES    BIT(5)
#define TMP119_AVG_32_SAMPLES   BIT(6)
#define TMP119_AVG_64_SAMPLES   (BIT(5) | BIT(6))

#define TMP119_MODE_CONTINUOUS  0
#define TMP119_MODE_SHUTDOWN    BIT(10)
#define TMP119_MODE_ONE_SHOT    (BIT(10) | BIT(11))

struct tmp119_data {
    uint16_t sample;
    uint16_t id;

#ifdef CONFIG_TMP119_ALERT_INTERRUPTS
    const struct device *tmp119_dev;

    const struct sensor_trigger *temp_alert_trigger;
    sensor_trigger_handler_t temp_alert_handler;
    
    struct gpio_callback temp_alert_gpio_cb;
#endif /* CONFIG_TMP119_ALERT_INTERRUPTS */
};

struct tmp119_dev_config {
    struct i2c_dt_spec bus;
    uint16_t odr;
#ifdef CONFIG_TMP119_ALERT_INTERRUPTS
    const struct gpio_dt_spec alert_gpios;
#endif /* CONFIG_TMP119_ALERT_INTERRUPTS */

};

static int tmp119_attr_set(const struct device *dev,
			   enum sensor_channel chan,
			   enum sensor_attribute attr,
			   const struct sensor_value *val);

static int tmp119_attr_get(const struct device *dev, enum sensor_channel chan,
			   enum sensor_attribute attr, struct sensor_value *val);

int tmp119_sample_fetch(const struct device *dev, enum sensor_channel chan);

int tmp119_channel_get(const struct device *dev,
                enum sensor_channel chan,
                struct sensor_value *val);

int tmp119_trigger_set(const struct device *dev,
				    const struct sensor_trigger *trig,
				    sensor_trigger_handler_t handler);


void tmp119_trigger_handler_alert(const struct device *gpio, 
                        struct gpio_callback *cb,
                        gpio_port_pins_t pins);
#endif //ZEPHYR_DRIVERS_SENSOR_TMP119_TMP119_H_
```

---

### 📄 tmp119.c

```c
#define DT_DRV_COMPAT   ti_mytmp119

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

#define EEPROM_SIZE_REG     sizeof(uint16_t)
#define EEPROM_MIN_BUSY_MS  7

LOG_MODULE_REGISTER(TMP119, CONFIG_SENSOR_LOG_LEVEL);

/**
 * @brief Reads a 16-bit register from the TMP119 sensor.
 *
 * This function performs a burst read over I2C to retrieve two bytes of data
 * from the specified TMP119 register and converts the value from big-endian
 * to CPU endianness.
 *
 * @param dev Pointer to the TMP119 device structure.
 * @param reg Register address to read from.
 * @param val Pointer to store the read 16-bit value.
 *
 * @return 0 on success, -EIO on I2C communication failure.
 */
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

/**
 * @brief Writes a 16-bit value to a TMP119 register.
 *
 * This function writes two bytes of data to the specified TMP119 register
 * using an I2C write operation.
 *
 * @param dev Pointer to the TMP119 device structure.
 * @param reg Register address to write to.
 * @param val 16-bit value to write to the register.
 *
 * @return 0 on success, or a negative error code on failure.
 */
static int tmp119_reg_write(const struct device *dev, uint8_t reg, uint16_t val)
{
	const struct tmp119_dev_config *cfg = dev->config;
	uint8_t tx_buf[3] = {reg, val >> 8, val & 0xFF};

	return i2c_write_dt(&cfg->bus, tx_buf, sizeof(tx_buf));
}

/**
 * @brief   Validate the TMP119 sensor's device ID.
 *
 * This function reads the device ID register from the TMP119 sensor and compares
 * it with the expected device ID constant (`TMP119_DEVICE_ID`) to ensure that the
 * connected device is indeed a TMP119. It helps verify sensor presence and identity
 * during initialization.
 * 
 * @par Steps performed:
 *  1. Read the 16-bit device ID from the TMP119's device ID register
 *     (`TMP119_REG_DEVICE_ID`) using `tmp119_reg_read()`.
 *  2. If the read fails, log an error and return `-EIO`.
 *  3. If the read value does not match `TMP119_DEVICE_ID`, log an error and return `-EINVAL`.
 *  4. If the device ID matches, return `0` indicating success.
 *
 * @param[in]  dev  Pointer to the TMP119 device structure.
 * @param[out] id   Pointer to a variable that will store the read device ID.
 * 
 * @retval 0         Device ID matched successfully.
 * @retval -EIO      If reading the device ID register fails.
 * @retval -EINVAL   If the read device ID does not match the expected TMP119 ID.
 */
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

/**
 * @brief Modify and write specific configuration bits to TMP119 sensor.
 *
 * This function updates selected bits in the TMP119 sensor's configuration register.
 * It first reads the current configuration, clears bits specified by the mask,
 * sets new bits provided in the conf argument, and then writes the result back.
 *
 * @par Steps performed:
 * 1. Read the current 16-bit configuration value from the TMP119 configuration register (TMP119_REG_CFGR).
 * 2. Use the bitmask (`mask`) to clear specific bits in the configuration that need to be changed.
 * 3. Apply the new configuration bits (`conf`) to the cleared bits in the register.
 * 4. Write the modified configuration value back to the sensor's configuration register.
 *
 * This ensures only targeted bits are updated without affecting the rest of the register.
 *
 * @param[in] dev   Pointer to the TMP119 device structure.
 * @param[in] mask  Bitmask indicating which bits to update in the configuration register.
 * @param[in] conf  New configuration bits to apply (only those covered by the mask).
 *
 * @retval 0        Success. Configuration register was updated correctly.
 * @retval <0       Error code returned from register read/write operation.
 */
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
    result = tmp119_reg_write(dev, TMP119_REG_CFGR, config);

	LOG_INF("Configure Register value written: 0x%04X", config);

	return result;
}

int tmp119_read_config(const struct device *dev)
{
    uint16_t config = 0;
    int result;

    /* Step 1: Read current configuration from TMP119 register */
    result = tmp119_reg_read(dev, TMP119_REG_CFGR, &config);
    if (result < 0) {
        LOG_ERR("Register read operation failed !");
        return result;  // Return error if read failed 
    }
	LOG_INF("tmp119_read_configure Register value written: 0x%04X", config);
	return result;
}

#ifdef CONFIG_TMP119_ALERT_INTERRUPTS
static int setup_interrupts(const struct device *dev)
{
    struct tmp119_data *drv_data = dev->data;
    const struct tmp119_dev_config *config = dev->config;
    const struct gpio_dt_spec *alert_gpios = &config->alert_gpios;
    int ret;    /* Store return value */

    if(!device_is_ready(alert_gpios->port)) {
        LOG_ERR("TMP119: gpio controller %s not ready", alert_gpios->port->name);
        return -ENODEV;
    }

    ret = gpio_pin_configure_dt(alert_gpios, (GPIO_INPUT | GPIO_PULL_UP));
    if (ret < 0) {
        LOG_ERR("Failed to configure gpio pin as input.");
        return ret;
    }

    gpio_init_callback(&drv_data->temp_alert_gpio_cb,
                tmp119_trigger_handler_alert,
                BIT(alert_gpios->pin));

    ret = gpio_add_callback(alert_gpios->port,
                &drv_data->temp_alert_gpio_cb);
    if(ret < 0) {
        LOG_ERR("Failed to add gpio callback function");
        return ret;
    }

    ret = gpio_pin_interrupt_configure_dt(alert_gpios,
                        GPIO_INT_EDGE_TO_INACTIVE);
    if(ret < 0) {
        LOG_ERR("Failed to configure gpio interrupt as rising edge");
        return ret;
    }
	LOG_INF("TMP119 interrupt initialized");
    return 0;
}
#endif /* CONFIG_TMP119_ALERT_INTERRUPTS */

/**
 * @brief Initializes the TMP119 sensor device.
 *
 * This function performs sensor initialization by checking the readiness of the I2C bus,
 * verifying the device ID, and writing initial configuration settings such as conversion mode
 * and output data rate. It also stores the device ID in the driver data structure.
 *
 * @param dev Pointer to the device structure for the TMP119 sensor.
 *
 * @return 0 on success, or a negative error code on failure.
 */
int tmp119_init(const struct device *dev)
{
	struct tmp119_data *drv_data = dev->data;
	const struct tmp119_dev_config *cfg = dev->config;

	int ret; /* Store function return value*/
	uint16_t id;

	if (!device_is_ready(cfg->bus.bus)) {
		LOG_ERR("I2C dev %s not ready.", cfg->bus.bus->name);
		return -EINVAL;
	}

#ifdef CONFIG_TMP119_ALERT_INTERRUPTS
#warning "CONFIG_TMP119_ALERT_INTERRUPTS ENABLED"
	/* save this driver instance for passing to other functions */
	drv_data->tmp119_dev = dev;

	ret = setup_interrupts(dev);

	if (ret < 0) {
		return ret;
	}
#endif /* CONFIG_TMP119_ALERT_INTERRUPTS */

	/* Check the Device ID */
	ret = tmp119_device_id_check(dev, &id);
	if (ret < 0) {
		return ret;
	}

	LOG_DBG("Got device ID: 0x%X", id);
	drv_data->id = id;

	ret = tmp119_write_config(dev, TMP119_CFGR_CONV, cfg->odr);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

/**
 * @brief Set the conversion cycle (output data rate) for TMP119
 * 
 * @param dev   Pointer to device structure
 * @param odr_value One of TMP119_DT_ODR_* macros (e.g., TMP119_DT_ODR_1000_MS)
 * 
 * @return 0 on success, or negative error code on failure
*/
int tmp119_set_conversion_cycle(const struct device *dev, uint16_t odr_value)
{
    /* Validate that the provided ODR value is one of the supported options */
    switch (odr_value) {
        case TMP119_DT_ODR_15_5_MS:
        case TMP119_DT_ODR_125_MS:
        case TMP119_DT_ODR_250_MS:
        case TMP119_DT_ODR_500_MS:
        case TMP119_DT_ODR_1000_MS:
        case TMP119_DT_ODR_4000_MS:
        case TMP119_DT_ODR_8000_MS:
        case TMP119_DT_ODR_16000_MS:
            break;

        default:
            return -EINVAL; /* Invalid ODR value */
    }

    /* Update configure register */
    return tmp119_write_config(dev, TMP119_CFGR_CONV, odr_value);
}

/**
 * @brief Set the operating mode of the TMP119 sensor
 * 
 * @param dev   Pointer to TMP119 device structure
 * @param mode  One of TMP119_MODE_* macros: 
 *              - TMP119_MODE_CONTINUOUS
 *              - TMP119_MODE_SHUTDOWN
 *              - TMP119_MODE_ONE_SHOT
 * 
 * @return  0 on success, or negative error code on failure
 */
int tmp119_set_mode(const struct device *dev, uint16_t mode)
{
	switch (mode) {
	case TMP119_MODE_CONTINUOUS:
	case TMP119_MODE_SHUTDOWN:
	case TMP119_MODE_ONE_SHOT:
		break;

	default:
		return -EINVAL; /* Invalid mode */
	}

	/* Update configure register */
	return tmp119_write_config(dev, TMP119_CFGR_MODE, mode);
}

/**
 * @brief Set the sampling rate (averaging) for the TMP119 sensor
 *
 * The TMP119 supports averaging of 1, 8, 32, or 64 temperature samples before
 * updating the temperature register. This function sets the desired averaging mode
 * by updating the AVG[1:0] bits (bits 6:5) in the configuration register.
 *
 * @param dev         Pointer to TMP119 device structure
 * @param avr_sample  One of TMP119_AVG_* macros:
 *                    - TMP119_AVG_1_SAMPLE
 *                    - TMP119_AVG_8_SAMPLE
 *                    - TMP119_AVG_32_SAMPLE
 *                    - TMP119_AVG_64_SAMPLE
 *
 * @return 0 on success, or negative error code on failure
 */
int tmp119_set_sampling_rate(const struct device *dev, uint16_t avr_sample)
{
    switch(avr_sample) {
        case TMP119_AVG_1_SAMPLE:
        case TMP119_AVG_8_SAMPLES:
        case TMP119_AVG_32_SAMPLES:
        case TMP119_AVG_64_SAMPLES:
            break;

        default:
            return -EINVAL; /* Invalid average sample rate */
    }

    /* Update configure register */
    return tmp119_write_config(dev, TMP119_CFGR_AVG, avr_sample);
}

/**
 * @brief Fetch the latest temperature sample from the TMP119 sensor.
 *
 * This function checks if new temperature data is available in the TMP119
 * sensor by reading the Configuration Register (CFGR). If the data is ready,
 * it reads the Temperature Register (TEMP) and stores the value in the driver's
 * internal data structure.
 *
 * @param dev Pointer to the device structure representing TMP119.
 * @param chan Sensor channel to fetch; must be SENSOR_CHAN_ALL or SENSOR_CHAN_AMBIENT_TEMP.
 *
 * @retval 0        Success, data fetched and stored.
 * @retval -EBUSY   Data is not yet ready in the sensor.
 * @retval -EINVAL  If channel provided is invalid.
 * @retval <0       Error code returned by I2C register read operation.
 *
 * @note This function should be called before calling `tmp119_channel_get()` 
 *       to retrieve the temperature value.
 *
 * @details
 * Steps performed by the function:
 * 1. Assert that the requested channel is either SENSOR_CHAN_ALL or SENSOR_CHAN_AMBIENT_TEMP.
 * 2. Clear the previous sample value in the driver's data structure.
 * 3. Read the Configuration Register (CFGR) to check if new data is ready.
 *    - If the Data Ready (DRDY) bit is not set, return -EBUSY.
 * 4. If data is ready, read the Temperature Register (TEMP).
 *    - If read fails, return the error code.
 * 5. Store the 16-bit temperature value (cast to signed) into `drv_data->sample`.
 */
int tmp119_sample_fetch(const struct device *dev, enum sensor_channel chan)
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
	LOG_INF("Sample Fetch cfg_reg = 0x%4X", cfg_reg);

	// uint8_t data_ready = ( cfg_reg & TMP119_CFGR_DATA_READY ) ? 1 : 0;
	// LOG_INF("TMP119_CFGR_DATA_READY = %d", data_ready);
	// tmp119_read_config(dev);

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

/**
 * @brief Get the latest temperature reading from the TMP119 sensor.
 *
 * This function retrieves the latest raw sample value from the sensor's driver 
 * context (previously fetched via `tmp119_sample_fetch`) and converts it to 
 * a standardized `sensor_value` format. It only supports the `SENSOR_CHAN_AMBIENT_TEMP` 
 * channel as TMP119 is a single-channel temperature sensor.
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param chan Sensor channel requested (must be SENSOR_CHAN_AMBIENT_TEMP).
 * @param val Pointer to struct sensor_value where converted temperature will be stored.
 *
 * @return 0 on success, 
 *         -ENOTSUP if channel is not supported.
 *
 * @details
 * Steps:
 * 1. Get the pointer to driver’s internal data using `dev->data`.
 * 2. Check if the requested channel is `SENSOR_CHAN_AMBIENT_TEMP`.
 *    - If not, return `-ENOTSUP` as only ambient temperature is supported.
 * 3. Convert the raw temperature sample to micro-Celsius:
 *    - TMP119 stores temperature as a signed 16-bit value.
 *    - Multiply the sample by TMP119_RESOLUTION (which should represent µ°C/LSB).
 *    - Divide by 10 for fixed-point scaling adjustment.
 * 4. Store the result in `sensor_value`:
 *    - `val->val1` holds the integer part (in °C).
 *    - `val->val2` holds the fractional part in micro-Celsius (µ°C).
 */
int tmp119_channel_get(const struct device *dev,
                enum sensor_channel chan,
                struct sensor_value *val)
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

/**
 * @brief Convert sensor_value representing output data rate (ODR) to TMP119-specific register value.
 *
 * This function takes a `sensor_value` representing frequency (in Hz) and converts it
 * into a corresponding TMP119 register value for configuring the output data rate (ODR).
 * The input is internally converted to micro-Hz and matched against known supported
 * ODR values of the TMP119 sensor. If no match is found, the function logs an error and returns -EINVAL.
 *
 * @param val Pointer to sensor_value representing ODR frequency.
 *
 * @return TMP119 ODR register value on success, or -EINVAL if unsupported.
 */
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

/**
 * @brief Set sensor attributes for the TMP119 device.
 *
 * This function configures the TMP119 sensor based on the provided attribute.
 * Supported attributes include sampling frequency, offset (only for TMP117),
 * oversampling, and different operating modes (shutdown, continuous, one-shot).
 *
 * @param dev   Pointer to the sensor device structure.
 * @param chan  Sensor channel (must be SENSOR_CHAN_AMBIENT_TEMP).
 * @param attr  Attribute to be set (e.g., sampling frequency, offset).
 * @param val   Value to set for the attribute.
 *
 * @return 0 on success, negative error code on failure.
 */
static int tmp119_attr_set(const struct device *dev,
			   enum sensor_channel chan,
			   enum sensor_attribute attr,
			   const struct sensor_value *val)
{
	struct tmp119_data *drv_data = dev->data;
	int16_t value;
	uint16_t avg;
	uint16_t mode;

	if (chan != SENSOR_CHAN_AMBIENT_TEMP) {
		return -ENOTSUP;
	}

	switch ((int)attr) {
#ifdef CONFIG_TMP119_ALERT_INTERRUPTS
	case SENSOR_ATTR_ALERT:
		LOG_INF("SENSOR_ATTR_ALERT");
		return tmp119_write_config(dev, TMP119_CFGR_ALERT_MODE_ACTIVE_HIGH, TMP119_CFGR_ALERT_MODE);
#endif /* CONFIG_TMP119_ALERT_INTERRUPTS */

	case SENSOR_ATTR_SAMPLING_FREQUENCY:
		LOG_INF("SENSOR_ATTR_SAMPLING_FREQUENCY");
		value = tmp119_conv_value(val);
		if (value < 0) {
			return value;
		}

		return tmp119_write_config(dev, TMP119_CFGR_CONV, value);

	case SENSOR_ATTR_OVERSAMPLING:
		/* sensor supports averaging 1, 8, 32 and 64 samples */
		LOG_INF("SENSOR_ATTR_OVERSAMPLING");
		LOG_INF("Val----->VAL1 = %d", val->val1);
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
		LOG_INF("SENSOR_ATTR_TMP119_SHUTDOWN_MODE");
		return tmp119_write_config(dev, TMP119_CFGR_MODE, TMP119_MODE_SHUTDOWN);

	case SENSOR_ATTR_TMP119_CONTINUOUS_CONVERSION_MODE:
		LOG_INF("SENSOR_ATTR_TMP119_CONTINUOUS_CONVERSION_MODE");
		return tmp119_write_config(dev, TMP119_CFGR_MODE, TMP119_MODE_CONTINUOUS);

	case SENSOR_ATTR_TMP119_ONE_SHOT_MODE:
		LOG_INF("SENSOR_ATTR_TMP119_ONE_SHOT_MODE");
		return tmp119_write_config(dev, TMP119_CFGR_MODE, TMP119_MODE_ONE_SHOT);

	case SENSOR_ATTR_TMP119_SOFT_RESET:
		LOG_INF("SENSOR_ATTR_TMP119_SOFT_RESET");
		return tmp119_write_config(dev, TMP119_SOFT_RESET, TMP119_SOFT_RESET);

	default:
		return -ENOTSUP;
	}
}

/**
 * @brief Get attribute value from TMP119 sensor.
 *
 * This function reads the value of a specified attribute for the ambient temperature channel.
 *
 * @param dev  Pointer to the device structure.
 * @param chan Sensor channel (only SENSOR_CHAN_AMBIENT_TEMP is supported).
 * @param attr Attribute to read (e.g., SENSOR_ATTR_CONFIGURATION).
 * @param val  Pointer to sensor_value structure to store the result.
 *
 * @return 0 on success, negative error code on failure or if unsupported.
 */
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

static DEVICE_API(sensor, tmp119_driver_api) = {
    .attr_set = tmp119_attr_set,
    .attr_get = tmp119_attr_get,
    .sample_fetch = tmp119_sample_fetch,
    .channel_get = tmp119_channel_get,
#ifdef CONFIG_TMP119_ALERT_INTERRUPTS
	.trigger_set = tmp119_trigger_set,
#endif /* CONFIG_TMP119_ALERT_INTERRUPTS */
};

/**
 * @brief Register TMP119 sensor driver instances defined in the device tree.
 *
 * This block defines and registers each TMP119 sensor instance found in the device tree.
 * It associates the sensor with its configuration and runtime data, sets the initialization 
 * priority, and binds the driver API (e.g., attribute get/set, sample fetch, and channel get).
 *
 * The macro `DEFINE_TMP119` expands for each child node with status "okay" under the TMP119 
 * compatible device node using `DT_INST_FOREACH_CHILD_STATUS_OKAY`.
 */
#define DEFINE_TMP119(inst)                                                     \
    static struct tmp119_data tmp119_data_##inst;                               \
    static const struct tmp119_dev_config tmp119_config_##inst = {              \
        .bus = I2C_DT_SPEC_INST_GET(inst),                                      \
        .odr = DT_INST_PROP(inst, odr),                                         \
        IF_ENABLED(CONFIG_TMP119_ALERT_INTERRUPTS,                              \
                (.alert_gpios = GPIO_DT_SPEC_INST_GET(inst, alert_gpios)))      \
    };                                                                          \
                                                                                \
    SENSOR_DEVICE_DT_INST_DEFINE(inst,                                          \
                                  tmp119_init,                                  \
                                  NULL,                                         \
                                  &tmp119_data_##inst,                          \
                                  &tmp119_config_##inst,                        \
                                  POST_KERNEL,                                  \
                                  CONFIG_SENSOR_INIT_PRIORITY,                  \
                                  &tmp119_driver_api);


DT_INST_FOREACH_STATUS_OKAY(DEFINE_TMP119)
```

### 📄 tmp119_trigger.c
```c
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>

#include "tmp119.h"

#warning "TMP119 Trigger Added"

LOG_MODULE_REGISTER(TMP119, CONFIG_SENSOR_LOG_LEVEL);

void tmp119_trigger_handler_alert(const struct device *gpio, 
                        struct gpio_callback *cb,
                        gpio_port_pins_t pins)
{
    struct tmp119_data *drv_data = CONTAINER_OF(cb, 
                                struct tmp119_data,
                                temp_alert_gpio_cb);

    /* Successful read, call set callbacks */
    if(drv_data->temp_alert_handler) {
        drv_data->temp_alert_handler(drv_data->tmp119_dev, 
                                drv_data->temp_alert_trigger);
    }
}

int tmp119_trigger_set(const struct device *dev,
				    const struct sensor_trigger *trig,
				    sensor_trigger_handler_t handler)
{
	struct tmp119_data *drv_data = dev->data;

    if(trig->type == SENSOR_TRIG_DATA_READY) {
        drv_data->temp_alert_handler = handler;
        drv_data->temp_alert_trigger = trig;
        return 0;
    }
	return -ENOTSUP;
}
```

---

## 📌 Driver Code Structure

### 🔖 Define Compatibility

```c
#define DT_DRV_COMPAT ti_tmp119
```

🔹 This links the driver to the compatible string `ti,mytmp119` defined in your `.dts` or `.yaml` file.

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
| `tmp119_trigger_set()`  | Set data ready interrupt from sensor               |

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

```c
add_subdirectory_ifdef(CONFIG_MYTMP119 mytmp119)
```

### Update Kconfig

add the line:
```c
source "drivers/sensor/ti/mytmp119/Kconfig"
```


## 📜 Step 4: Create Device Binding YAML File

create a file at: `zephyr/dts/bindings/sensor/ti,my	tmp119.yaml`

### ti,mytmp119.yaml
```yaml
# SPDX-License-Identifier: Apache-2.0

description: Texas Instruments TMP119 Digital Temperature Sensor

compatible: "ti,mytmp119"

include: [sensor-device.yaml, i2c-device.yaml]

properties:
  reg:
    required: true

  odr:
    type: int
    required: true
    default: 0x200  # default to 1000ms
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

  alert-gpios:
    type: phandle-array
    description: |
      Identifies the ALERT signal, which is active-low open drain when
      produced by the sensor.
```

## 📁 Step 5 : Create DT Binding Header

create a header file: `zephyr/include/zephyr/dt-bindings/sensor/mytmp119.h`

```c
/**
 * Copyright (c) 2024 Vitrolife A/S
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_TI_MYTMP119_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_TI_MYTMP119_H_

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

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_TI_MYTMP119_H_ */
```

## 📁 Step 6 : Create a file for TMP119 driver header (tmp119.h)

Path: `zephyr/include/drivers/sensor/mytmp119.h`

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
#define TMP119_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(ti_mytmp119)

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

