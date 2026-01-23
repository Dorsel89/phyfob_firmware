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

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>


#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>



#define sensirion_hal_sleep_us sensirion_i2c_hal_sleep_usec

#include <zephyr/drivers/spi.h>
#define FLASH_LABEL "MX25R6435F"


struct spi_cs_control spi_flash_cs = {
	.gpio = SPI_CS_GPIOS_DT_SPEC_GET(DT_NODELABEL(flash_spi_master)),
	.delay = 0,
};

static const struct spi_config spi_cfg = {
    .frequency = 8000000,
    .operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
    .slave = 0,
    .cs = &spi_flash_cs,
};



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
        //init_BAS();
       
        
        const struct device *flash_dev = DEVICE_DT_GET(DT_NODELABEL(mx25r6435f));
       
        if (!device_is_ready(flash_dev)) {
                printk("SPI NOR Device nicht bereit\n");
                return;
        }

        /*
        0xB9 for mx25r6435f
        0x79 for AT25FF161A
        */
        uint8_t dpd_cmd = 0x79;
        int ret = spi_write(flash_dev, &spi_cfg, &(struct spi_buf_set){
                .buffers = &(struct spi_buf){
                .buf = &dpd_cmd,
                .len = 1,
                },
                .count = 1,
        });
        if (ret) {
                printk("Deep Power Down Kommando fehlgeschlagen: %d\n", ret);
        } else {
                printk("Flash-Speicher in DeepSleep versetzt.\n");
        }
        
        
        pm_device_action_run(flash_dev,PM_DEVICE_ACTION_SUSPEND);


        return 0;
}
