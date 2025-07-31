#ifndef HDC_H
#define HDC_H

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>

#include "sensors.h"
#include "ble.h"

#define HDC_NODE DT_ALIAS(hdc)
#if DT_NODE_HAS_STATUS(HDC_NODE, okay)
const static struct device *hdc_dev = DEVICE_DT_GET(HDC_NODE);
#else
#error "Node is disabled"
#endif

typedef struct {
	float humidity;
	float temperature;
	float timestamp;
	float array[3];
	uint16_t timer_interval;
	uint8_t config[20];
}HDC;

extern HDC hdc_data;
static struct sensor_value hdc_temp, hdc_humid;

static struct k_timer timer_hdc;
static struct k_work work_hdc;
static struct k_work config_work_hdc;

extern bool init_hdc();
extern void sleep_hdc(bool sleep);
extern void submit_config_hdc();

void send_data_hdc();
void hdc_data_ready();
void set_config_hdc();

extern void hdc_logging(bool l);
#endif