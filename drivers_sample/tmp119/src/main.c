#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/tmp119.h>
#include <zephyr/dt-bindings/sensor/tmp119.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(tmp119_sample, LOG_LEVEL_INF);

#define TMP119_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(ti_tmp119)
// #define TMP119_NODE DT_NODELABEL(tmp119)

// const struct device *dev = DEVICE_DT_GET_ANY(ti_tmp119);
const struct device *dev = DEVICE_DT_GET(TMP119_NODE);
// const struct device *dev = DEVICE_DT_GET_ANY(ti_tmp119);

// const struct device *const dev = DEVICE_DT_GET_ANY(ti_tmp119);

int main() {
    // int ret;    /* Store return value */
    LOG_INF("TMP119 Sensor's Temperature Reading Sample");

    if(!device_is_ready(dev)) {
        LOG_ERR("TMP119 device not found or not ready.");
        return -ENODEV;
    }

    // ret = tmp119_init(dev); 
    // if (ret < 0) {
    //     LOG_ERR("TMP119 is not initalized !");
    //     return -EINVAL;
    // }

    LOG_INF("TMP119 sensor sample started");

    // === Set SENSOR_ATTR_SAMPLING_FREQUENCY ===
	struct sensor_value freq_val;
	// Example: 1 Hz → 1 sample per second → 1 Hz
	freq_val.val1 = 1;
	freq_val.val2 = 0;

	if (sensor_attr_set(dev, SENSOR_CHAN_AMBIENT_TEMP,
	                    SENSOR_ATTR_SAMPLING_FREQUENCY, &freq_val) < 0) {
		LOG_ERR("Failed to set sampling frequency");
	}

	// === Set SENSOR_ATTR_OVERSAMPLING ===
	struct sensor_value oversample_val;
	oversample_val.val1 = 8;  // Valid: 1, 8, 32, 64
	oversample_val.val2 = 0;

	if (sensor_attr_set(dev, SENSOR_CHAN_AMBIENT_TEMP,
	                    SENSOR_ATTR_OVERSAMPLING, &oversample_val) < 0) {
		LOG_ERR("Failed to set oversampling");
	}

    if (sensor_attr_set(dev, SENSOR_CHAN_AMBIENT_TEMP,
	                    SENSOR_ATTR_TMP119_CONTINUOUS_CONVERSION_MODE, NULL) < 0) {
		LOG_ERR("Failed to set continuous  conversion mode");
	}

	// === Read and log temperature every second ===
	struct sensor_value temp;

	while (1) {
		if (sensor_sample_fetch(dev) < 0) {
			LOG_ERR("Failed to fetch sample from TMP119");
			k_sleep(K_SECONDS(1));
			continue;
		}

		if (sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp) < 0) {
			LOG_ERR("Failed to get temperature reading");
			k_sleep(K_SECONDS(1));
			continue;
		}

		LOG_INF("Temperature: %d.%06d °C", temp.val1, temp.val2);
		k_sleep(K_SECONDS(1));
	}
}