#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/mytmp119.h>
#include <zephyr/dt-bindings/sensor/mytmp119.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(tmp119_sample, LOG_LEVEL_DBG);

#define TMP119_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(ti_mytmp119)

const struct device *dev = DEVICE_DT_GET(TMP119_NODE);
struct sensor_value temp;

// Semaphore for ALERT interrupt
K_SEM_DEFINE(alert_sem, 0, 1);

// Work item for temperature data handling
static struct k_work temp_work;

/**
 * @brief Initialize TMP119
 */
int init_main_tmp119(const struct device *dev)
{
	if (!device_is_ready(dev)) {
		LOG_ERR("TMP119 device not found or not ready.");
		return -ENODEV;
	}
	return 0;
}

/**
 * @brief Configure TMP119
 */
void configure_tmp119(const struct device *dev)
{
	struct sensor_value freq_val = {1, 0};       // 1 Hz sampling frequency
	struct sensor_value oversample_val = {8, 0}; // 8x oversampling

	if (sensor_attr_set(dev, SENSOR_CHAN_AMBIENT_TEMP, SENSOR_ATTR_TMP119_SOFT_RESET, NULL) < 0) {
		LOG_ERR("Failed to SOFT RESET");
	}
	k_sleep(K_MSEC(10));

	if (sensor_attr_set(dev, SENSOR_CHAN_AMBIENT_TEMP, SENSOR_ATTR_SAMPLING_FREQUENCY, &freq_val) < 0) {
		LOG_ERR("Failed to set sampling frequency");
	}

	if (sensor_attr_set(dev, SENSOR_CHAN_AMBIENT_TEMP, SENSOR_ATTR_OVERSAMPLING, &oversample_val) < 0) {
		LOG_ERR("Failed to set oversampling");
	}

	if (sensor_attr_set(dev, SENSOR_CHAN_AMBIENT_TEMP, SENSOR_ATTR_TMP119_CONTINUOUS_CONVERSION_MODE, NULL) < 0) {
		LOG_ERR("Failed to set continuous conversion mode");
	}

	if (sensor_attr_set(dev, SENSOR_CHAN_AMBIENT_TEMP, SENSOR_ATTR_ALERT, NULL) < 0) {
		LOG_ERR("Failed to enable ALERT mode");
	}
}

/**
 * @brief Work handler for processing temperature
 */
static void temp_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	if (sensor_sample_fetch(dev) < 0) {
		LOG_ERR("Failed to fetch sample from TMP119 in work");
		return;
	}

	if (sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp) < 0) {
		LOG_ERR("Failed to get temperature readings in work");
		return;
	}

	LOG_INF("Workqueue: Temperature = %d.%02d °C", temp.val1, temp.val2);
}

/**
 * @brief Data acquisition thread
 */
void data_thread(void)
{
	while (1) {
		// Wait until interrupt gives semaphore
		k_sem_take(&alert_sem, K_FOREVER);

		// Submit work to system workqueue
		k_work_submit(&temp_work);
	}
}

K_THREAD_DEFINE(data_thread_id, 1024, data_thread, NULL, NULL, NULL,
		5, 0, 0);

/**
 * @brief Interrupt callback from TMP119 ALERT
 */
static void tmp119_trigger_handler(const struct device *dev,
				   const struct sensor_trigger *trig)
{
	LOG_DBG("TMP119 ALERT Interrupt Received");
	k_sem_give(&alert_sem);
}

/**
 * @brief Main
 */
int main(void)
{
	LOG_INF("TMP119 Sensor Sample with Thread + Work");

	if (init_main_tmp119(dev) < 0) {
		return -1;
	}

	configure_tmp119(dev);

	// Initialize work item
	k_work_init(&temp_work, temp_work_handler);

	// Set up trigger (data ready interrupt)
	struct sensor_trigger trig = {
		.type = SENSOR_TRIG_DATA_READY,
		.chan = SENSOR_CHAN_AMBIENT_TEMP
	};

	if (sensor_trigger_set(dev, &trig, tmp119_trigger_handler) < 0) {
		LOG_ERR("Failed to set TMP119 trigger");
		return -1;
	}

	LOG_INF("Initialization complete, waiting for interrupts...");

	return 0; // main thread exits, data_thread keeps running
}
