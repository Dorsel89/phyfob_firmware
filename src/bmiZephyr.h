#ifndef _BMIZEPHYR_H
#define _BMIZEPHYR_H

#include "bmi3.h"
#include "bmi323_defs.h"
#include "bmi323.h"
//#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include "sensors.h"
#include <stdio.h>
#include <math.h>
#include <zephyr/drivers/spi.h>

#define BMI3_SENSOR_OK 0x00
#define BMI384_I2C_ADDR	0x68
#define BMI_INT DT_NODELABEL(button0)

#define BMI_X 0b00000001
#define BMI_Y 0b00000010
#define BMI_Z 0b00000100
#define BMI_ABS 0b00001000

const static struct device *bmi_dev = DEVICE_DT_GET(DT_ALIAS(spi));

extern int8_t init_bmi();
extern void submit_config_bmi();
extern uint8_t sleep_bmi(bool SLEEP);

static struct k_work work_bmi;
static struct k_work config_work_bmi;

static int8_t bmiResult;
static uint8_t dByte;

static struct device *spi_dev;


static struct bmi3_dev bmi3_dev;
//static struct bmi3_data bmi_data;
static struct bmi3_sensor_data sensor_data[2] = {{ 0 }};
static struct bmi3_sens_config config[2] = {{ 0 }};


static uint8_t*  bmi_en;
static uint8_t*  bmi_rate;
static uint8_t*  bmi_range_acc;
static uint8_t*  bmi_range_gyr;
static uint8_t*  bmi_average;
static uint8_t*  bmi_axis;
static uint8_t*  bmi_event_size;

#endif