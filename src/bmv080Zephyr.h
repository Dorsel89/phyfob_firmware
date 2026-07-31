#ifndef _BMV080ZEPHYR_H
#define _BMV080ZEPHYR_H
        
#include "bmv080.h"
#include "bmv080_defs.h"
#include <zephyr/drivers/i2c.h>
#include <stdbool.h>
#include <zephyr/sys/byteorder.h>
#include "sensors.h"
#include "ble.h"


#define BMV080_ADDR 0x57

// Operation mode
#define CONTINUOUS_MODE     0 ///< Continuous mode, sensor takes measurements continuously
#define DUTY_CYCLE_MODE     1 ///< Duty cycle mode, sensor takes measurements at specified intervals

// Measurement algorithm (bmv080_measurement_algorithm_t)
#define FAST_RESPONSE       1 ///< response,suitable for scenarios requiring quick response
#define BALANCED            2 ///< Balanced, suitable for scenarios where a balance needs to be struck between precision and rapid response
#define HIGH_PRECISION      3 ///< High precision, suitable for scenarios requiring high accuracy

/* config[] layout written via the bmv080_cnfg BLE characteristic:
 *   config[0]   : mode (see BMV080_MODE_* below)
 *   config[1]   : measurement algorithm (1=fast, 2=balanced, 3=high
 *                 precision; 0 or out-of-range -> keep library default =
 *                 high precision). Only used in continuous mode - the
 *                 library fixes this to "fast response" in duty-cycling
 *                 mode regardless of what's set here.
 *   config[2..3]: duty-cycling period in seconds, uint16 little-endian.
 *                 Only used in duty-cycling mode. 0 -> keep the library's
 *                 own default (30s). Must be at least the integration time
 *                 (library default 10s) + 2s. */
#define BMV080_MODE_STOP        0 ///< stop/idle
#define BMV080_MODE_CONTINUOUS  1 ///< continuous measurement (~once/sec)
#define BMV080_MODE_DUTY_CYCLE  2 ///< one measurement every config[2..3] seconds

#define ERR_OK            0      ///< no error
#define ERR_DATA_BUS      1      ///< data bus error
#define ERR_DATA_READ     2      ///< data read error
#define ERR_IC_VERSION    3      ///< IC version mismatch



static bool getBmv080ID(char *id);

static bmv080_output_t _bmv080Data; // BMV080 sensor data.
static bool _bmv080DataOK = false; // Flag to indicate if BMV080 data is ready.


extern uint8_t init_bmv080();
extern void bmv080_poll(void);
/* Called from ble.c config_submits() when the bmv080_cnfg characteristic is
 * written. Only flags the new config; it is applied on the main thread inside
 * bmv080_poll() because the BMV080 library needs ~10 kB of stack (only main's
 * stack is sized for that) and to keep all library access on a single thread. */
extern void submit_config_bmv080(void);
/* Called from ble.c's disconnected() when the phone/phyphox disconnects, so
 * the measurement stops (like the other sensors) instead of continuing to
 * run - and burning power - unattended. Only flags the stop; see
 * submit_config_bmv080() above for why. */
extern void bmv080_request_stop(void);

/* How often main()'s loop calls bmv080_poll() (i.e. bmv080_serve_interrupt()).
 * Must be <= 1000ms per the datasheet; new PM data itself only actually
 * arrives roughly once every ~1.03s regardless (continuous mode's own
 * natural rate), this just has to be frequent enough not to miss it. */
#define BMV080_POLL_INTERVAL_MS 500

#endif