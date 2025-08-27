#ifndef __MAIN_H__
#define __MAIN_H__

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/hdc2010.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Struct for storing sensor readings */
struct hdc2010_reading {
    void *fifo_reserved;  /* Required by k_fifo */
    struct sensor_value temperature;
    struct sensor_value humidity;
};

/* Function prototypes */
int init_main_hdc2010(const struct device *dev);
void get_sensor_value(const struct device *dev, struct hdc2010_reading *reading);
// int configure_hdc2010(const struct device *dev);
void hdc2010_data_acquired(struct k_work *work);
void hdc2010_data_print(void *p1, void *p2, void *p3);
// void hdc2010_trigger_handler(const struct device *dev, const struct sensor_trigger *trig);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */
