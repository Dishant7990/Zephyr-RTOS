#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <stdlib.h>       // for k_malloc, k_free
#include <string.h>
#include "main.h"

LOG_MODULE_REGISTER(hdc2010_sample, LOG_LEVEL_DBG);

#define THREAD_STACK_SIZE           2048U
#define DATA_PRINT_THREAD_PRIORITY  4
#define DATA_FETCH_THREAD_PRIORITY  5

// Get device node for HDC2010 sensor (compatible "ti,hdc2010")
#define HDC2010_NODE    DT_COMPAT_GET_ANY_STATUS_OKAY(ti_myhdc2010)
const struct device *dev = DEVICE_DT_GET(HDC2010_NODE);

// FIFO for sensor readings (queue holds pointers to dynamically allocated readings)
K_FIFO_DEFINE(my_fifo);

/* Workqueue related declarations */
K_THREAD_STACK_DEFINE(my_stack_area, THREAD_STACK_SIZE);
static struct k_work_q my_workq;
static struct k_work my_work;

/**
 * @brief Initialize HDC2010 device, check if ready
 * 
 * @param dev Pointer to device struct
 * 
 * @return 0 if success, error code otherwise
 */
int init_main_hdc2010(const struct device *dev)
{
    if (!device_is_ready(dev)) {
        LOG_ERR("HDC2010 device not ready");
        return -ENODEV;
    }
    return 0;
}

/**
 * @brief Fetch sensor data and put reading into FIFO
 * 
 * @param dev Device pointer
 * 
 * @param reading Pointer to allocated reading struct to store results
 */
void get_sensor_value(const struct device *dev, struct hdc2010_reading *reading)
{
    int ret;

    ret = sensor_sample_fetch(dev);
    if (ret < 0) {
        LOG_ERR("Failed to fetch sample from HDC2010");
        return;
    }

    ret = sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &reading->temperature);
    if (ret < 0) {
        LOG_ERR("Failed to get temperature from HDC2010");
        return;
    }

    ret = sensor_channel_get(dev, SENSOR_CHAN_HUMIDITY, &reading->humidity);
    if (ret < 0) {
        LOG_ERR("Failed to get humidity from HDC2010");
        return;
    }
}

/**
 * @brief Work handler function to be scheduled on workqueue.
 *        Reads sensor data and puts it into FIFO for processing.
 * 
 * @param work Pointer to k_work struct (unused here)
 */
void hdc2010_data_acquired(struct k_work *work)
{
    // Allocate memory for reading
    struct hdc2010_reading *reading = k_malloc(sizeof(*reading));
    if (reading == NULL) {
        LOG_ERR("Failed to allocate memory for sensor reading");
        return;
    }

    // Get sensor data into allocated struct
    get_sensor_value(dev, reading);

    // Put pointer into FIFO for processing in print thread
    k_fifo_put(&my_fifo, reading);
}

/**
 * @brief Thread function to continuously read from FIFO and print sensor data.
 * @param p1, p2, p3 Unused parameters for thread signature
 */
void hdc2010_data_print(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    while (1) {
        // Get pointer to next sensor reading from FIFO (wait forever)
        struct hdc2010_reading *reading = k_fifo_get(&my_fifo, K_FOREVER);

        if (reading != NULL) {
            LOG_INF("Temperature: %d.%02d C\tHumidity: %d.%02d %%RH",
                    reading->temperature.val1,
                    reading->temperature.val2,
                    reading->humidity.val1,
                    reading->humidity.val2);

            // Free allocated memory after processing
            k_free(reading);
        }

        k_sleep(K_MSEC(10));
    }
}

/**
 * @brief Configure HDC2010 sensor attributes such as reset, frequency, resolution, etc.
 * @param dev Device pointer
 * @return 0 if success, error code otherwise
 */
int configure_hdc2010(const struct device *dev)
{
    int ret;
    struct sensor_value val;

    // Soft reset
    val.val1 = HDC2010_CONFIG_SOFT_RESET;
    val.val2 = 0;
    ret = sensor_attr_set(dev, SENSOR_CHAN_AMBIENT_TEMP, SENSOR_ATTR_HDC2010_SOFT_RESET, &val);
    if (ret != 0) {
        LOG_ERR("Failed to set soft reset attribute");
        return ret;
    }

    /* Set interrupt comparator mode */
    memset(&val, 0, sizeof(val));
    val.val1 = HDC2010_CONFIG_INT_MODE;
    val.val2 = 0;
    ret = sensor_attr_set(dev, SENSOR_CHAN_AMBIENT_TEMP, SENSOR_ATTR_HDC2010_ALERT_MODE, &val);
    if (ret != 0) {
        LOG_ERR("Failed to enable data ready interrupt");
        return ret;
    }
    
    // Sampling frequency (e.g. 1 Hz)
    memset(&val, 0, sizeof(val));
    val.val1 = HDC2010_AMM_1HZ;
    val.val2 = 0;
    ret = sensor_attr_set(dev, SENSOR_CHAN_AMBIENT_TEMP, SENSOR_ATTR_SAMPLING_FREQUENCY, &val);
    if (ret != 0) {
        LOG_ERR("Failed to set sampling frequency");
        return ret;
    }

    // Enable Sensor DRDY Interrupt
    memset(&val, 0, sizeof(val));
    val.val1 = HDC2010_CONFIG_DRDY_INT_EN;
    val.val2 = 0;
    ret = sensor_attr_set(dev, SENSOR_CHAN_AMBIENT_TEMP, SENSOR_ATTR_HDC2010_INT_EN, &val);
    if (ret != 0) {
        LOG_ERR("Failed to set sampling frequency");
        return ret;
    }

    // Data ready interrupt enable
    memset(&val, 0, sizeof(val));
    val.val1 = HDC2010_DRDY_ENABLE;
    val.val2 = 0;
    ret = sensor_attr_set(dev, SENSOR_CHAN_AMBIENT_TEMP, SENSOR_ATTR_ALERT, &val);
    if (ret != 0) {
        LOG_ERR("Failed to enable data ready interrupt");
        return ret;
    }

    // Resolution (temp + humidity 14-bit)
    memset(&val, 0, sizeof(val));
    val.val1 = (HDC2010_TEMP_RES_14BIT | HDC2010_HUMI_RES_14BIT);
    val.val2 = 0;
    ret = sensor_attr_set(dev, SENSOR_CHAN_AMBIENT_TEMP, SENSOR_ATTR_RESOLUTION, &val);
    if (ret != 0) {
        LOG_ERR("Failed to set resolution");
        return ret;
    }

    // Measurement mode (humidity + temperature)
    memset(&val, 0, sizeof(val));
    val.val1 = HDC2010_MEAS_MODE_HUMI_TEMP;
    val.val2 = 0;
    ret = sensor_attr_set(dev, SENSOR_CHAN_AMBIENT_TEMP, SENSOR_ATTR_HDC2010_MEAS_MODE, &val);
    if (ret != 0) {
        LOG_ERR("Failed to set measurement mode");
        return ret;
    }

    
    // Measurement mode (humidity + temperature)
    memset(&val, 0, sizeof(val));
    val.val1 = HDC2010_MEAS_TRIG;
    val.val2 = 0;
    ret = sensor_attr_set(dev, SENSOR_CHAN_AMBIENT_TEMP, SENSOR_ATTR_HDC2010_START_MEAS, &val);
    if (ret != 0) {
        LOG_ERR("Failed to start measurement");
        return ret;
    }

    return 0;
}

/**
 * @brief Interrupt handler triggered by sensor data ready event.
 *        Submits work to the custom workqueue.
 * @param dev Sensor device pointer (unused)
 * @param trig Trigger information (unused)
 */
void hdc2010_trigger_handler(const struct device *dev, const struct sensor_trigger *trig)
{
    LOG_DBG("HDC2010 data ready interrupt received");

    // Submit work to custom work queue (NOT default system workqueue)
    k_work_submit_to_queue(&my_workq, &my_work);
}

/**
 * @brief Main application entry point
 */
int main(void)
{
    int ret;

    LOG_INF("Starting HDC2010 sensor data acquisition using interrupt");

    // const struct device *dev = DEVICE_DT_GET(HDC2010_NODE);

    ret = init_main_hdc2010(dev);
    if (ret != 0) {
        return ret;
    }

    // Initialize work struct with handler
    k_work_init(&my_work, hdc2010_data_acquired);

    // Initialize and start custom work queue with dedicated stack and priority
    k_work_queue_init(&my_workq);
    k_work_queue_start(&my_workq, my_stack_area,
                       K_THREAD_STACK_SIZEOF(my_stack_area),
                       DATA_FETCH_THREAD_PRIORITY, NULL);

    // Configure the sensor with required settings
    ret = configure_hdc2010(dev);
    if (ret != 0) {
        LOG_ERR("Sensor configuration failed");
        return ret;
    }

    // Set up data ready trigger (interrupt)
    struct sensor_trigger trig = {
        .type = SENSOR_TRIG_DATA_READY,
        .chan = SENSOR_CHAN_AMBIENT_TEMP,
    };

    ret = sensor_trigger_set(dev, &trig, hdc2010_trigger_handler);
    if (ret < 0) {
        LOG_ERR("Failed to set sensor trigger");
        return ret;
    }

    LOG_INF("Initialization complete, starting data print thread");

    // Start a dedicated thread for printing data from FIFO
    // This thread is defined below with K_THREAD_DEFINE macro

    // Keep main thread alive indefinitely
    while (1) {
        k_sleep(K_FOREVER));
    }

    return 0;
}

// Thread definition for data print thread
K_THREAD_DEFINE(sensor_data_print, THREAD_STACK_SIZE, hdc2010_data_print,
                NULL, NULL, NULL, DATA_PRINT_THREAD_PRIORITY, 0, 0);
