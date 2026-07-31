#ifndef _SUPERCAP_H
#define _SUPERCAP_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/adc.h>
#include "sensors.h"

/* Supercap voltage divider (2x 1 MOhm, on P0.30/AIN6): the supercap can be
 * charged up above the nRF52832's absolute maximum pin voltage (~VDD+0.3V),
 * so it's divided by 2 before reaching the ADC. Actual voltage = ADC-measured
 * voltage * SUPERCAP_DIVIDER_RATIO. */
#define SUPERCAP_DIVIDER_RATIO 2.0f

/* config[] layout written via the supercap_cnfg BLE characteristic:
 *   config[0] : enable (0 = stop, 1 = start periodic voltage readout)
 *   config[1] : interval in units of 100 ms (e.g. 10 -> 1000 ms), same
 *               convention as STCC4's config[1] */

static struct k_timer timer_supercap;
static struct k_work work_supercap;
static struct k_work config_work_supercap;

extern int8_t init_supercap(void);
extern void submit_config_supercap(void);
void send_data_supercap(void);
void supercap_data_ready(void);
void set_config_supercap(void);

#endif
