#ifndef _LSM6DSR_H
#define _LSM6DSR_H

#include "lsm6dsr.h"
#include <zephyr/kernel.h>
#include <stdint.h>
#include <zephyr/drivers/spi.h>
#include "lsm6dsr_reg.h"
#include "sensors.h"
#include <zephyr/pm/device.h>

#define FORMAT_FLOAT 0
#define FORMAT_INT16 1

#define ACC_BIT 0
#define GYR_BIT 1
static uint8_t whoamI,rst;
const static struct device *lsm_dev = DEVICE_DT_GET(DT_ALIAS(spi));

extern int8_t init_lsm();
extern void adjust_lsm_configuration();
extern uint8_t enable_lsm(uint8_t en);

static struct k_work work_lsm;
static struct k_work config_work_lsm;

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static void tx_com( uint8_t *tx_buffer, uint16_t len );
static void platform_delay(uint32_t ms);
static void platform_init(void);

#define LSM_INT DT_NODELABEL(button0)

static uint8_t*  lsm_en;
static uint8_t*  lsm_rate;
static uint8_t*  lsm_range_acc;
static uint8_t*  lsm_range_gyr;
static uint8_t*  lsm_average;
static uint8_t*  lsm_event_size;
static uint8_t*  lsm_format;

#endif