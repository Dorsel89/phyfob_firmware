#ifndef _BMV080ZEPHYR_H
#define _BMV080ZEPHYR_H
        
#include "bmv080.h"
#include "bmv080_defs.h"
#include <zephyr/drivers/i2c.h>
#include <stdbool.h>
#include <zephyr/sys/byteorder.h>


#define BMV080_ADDR 0x57

// Operation mode
#define CONTINUOUS_MODE     0 ///< Continuous mode, sensor takes measurements continuously
#define DUTY_CYCLE_MODE     1 ///< Duty cycle mode, sensor takes measurements at specified intervals

// Measurement algorithm
#define FAST_RESPONSE       1 ///< response,suitable for scenarios requiring quick response
#define BALANCED            2 ///< Balanced, suitable for scenarios where a balance needs to be struck between precision and rapid response
#define HIGH_PRECISION      3 ///< High precision, suitable for scenarios requiring high accuracy

#define ERR_OK            0      ///< no error
#define ERR_DATA_BUS      1      ///< data bus error
#define ERR_DATA_READ     2      ///< data read error
#define ERR_IC_VERSION    3      ///< IC version mismatch



static bool getBmv080ID(char *id);

static bmv080_output_t _bmv080Data; // BMV080 sensor data.
static bool _bmv080DataOK = false; // Flag to indicate if BMV080 data is ready.


extern uint8_t init_bmv080();

#endif