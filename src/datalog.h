#ifndef _DATALOG_H
#define _DATALOG_H

#include <zephyr/kernel.h>
#include <stdint.h>

#define DATALOG_CMD_STOP  0x00
#define DATALOG_CMD_START 0x01
#define DATALOG_CMD_DUMP  0x02
#define DATALOG_CMD_ERASE 0x03

#define DATALOG_SENSOR_CO2      0x01
#define DATALOG_SENSOR_TEMP     0x02
#define DATALOG_SENSOR_HUMIDITY 0x04
#define DATALOG_SENSOR_PRESSURE 0x08

void init_datalog(void);
/* sensors_mask is only used by DATALOG_CMD_START. The third parameter is
 * reused for two different purposes depending on cmd:
 * - DATALOG_CMD_START: logging interval in seconds.
 * - DATALOG_CMD_DUMP: maximum age of records to send, in minutes
 *   (0 = no limit, send the whole log).
 * Ignored for DATALOG_CMD_STOP/DATALOG_CMD_ERASE. */
void datalog_configure(uint8_t cmd, uint8_t sensors_mask, uint16_t interval_s);

#endif
