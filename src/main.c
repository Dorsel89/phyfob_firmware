#include <zephyr/kernel.h>

#include <zephyr/sys/printk.h>
#include <stddef.h>
#include <zephyr/sys/util.h>
#include <zephyr/types.h>
#include <sys/_types.h>
#include "ble.h"
#include "bmpZephyr.h"
#include "hdc.h"
#include "lsm6dsr.h"
#include "stcc4Zephyr.h"
#include "datalog.h"

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#define sensirion_hal_sleep_us sensirion_i2c_hal_sleep_usec

int main(void)
{
        printk("HELLOWORLD\r\n");

        //init logging
        logging.enable = true;
        logging.interval_s = 60;

        init_ble();
        init_hdc();
        init_bmp();
        init_lsm();
        init_stcc4();
        init_datalog();
        //default: log CO2, temperature, humidity and pressure every 60s
        datalog_configure(DATALOG_CMD_START,
                          DATALOG_SENSOR_CO2 | DATALOG_SENSOR_TEMP | DATALOG_SENSOR_HUMIDITY | DATALOG_SENSOR_PRESSURE,
                          10);
        //init_BAS();

        return 0;
}
