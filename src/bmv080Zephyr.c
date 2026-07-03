#include "bmv080Zephyr.h"
#include <zephyr/drivers/gpio.h>
#include "sensors.h"
#include <stdio.h>
#include <math.h>
#include <zephyr/drivers/i2c.h>

static bmv080_handle_t _bmv080_handle = NULL;  // Handle for the BMV080 sensor.
static struct device *bmv_dev = DEVICE_DT_GET(DT_ALIAS(i2c));
#define BMV080_NODE DT_ALIAS(bmv080)
static const struct i2c_dt_spec bmv080_dev = I2C_DT_SPEC_GET(BMV080_NODE);

static int8_t bmv080_i2c_read(bmv080_sercom_handle_t sercom_handle, uint16_t header, uint16_t *payload, uint16_t payload_length)
{
    const struct i2c_dt_spec *dev = (const struct i2c_dt_spec *)sercom_handle;
    uint16_t register_address = (uint16_t)(header << 1);
    size_t read_len = payload_length * 2;
    uint8_t header_buf[2];
    uint8_t payload_buf[read_len];

    /* Big Endian */
    sys_put_be16(register_address, header_buf);

    int32_t rc = i2c_write_read_dt(dev, header_buf, sizeof(header_buf), payload_buf, read_len);
    if (rc != 0) {
        printk("bmv080_i2c_read: i2c_write_read_dt failed: %d\n", rc);
        return E_BMV080_ERROR_HW_READ;
    }

    /* Convert the read buffer (Big Endian) back into 16-bit words */
    for (uint16_t i = 0; i < payload_length; i++) {
        payload[i] = sys_get_be16(&payload_buf[i * 2]);
    }

    return E_BMV080_OK;
}

static int8_t bmv080_i2c_write(bmv080_sercom_handle_t sercom_handle, uint16_t header, const uint16_t *payload, uint16_t payload_length)
{
    const struct i2c_dt_spec *dev = (const struct i2c_dt_spec *)sercom_handle;
    uint16_t register_address = (uint16_t)(header << 1);
    size_t total_len = 2 + (payload_length * 2);
    uint8_t buf[total_len];
    
    /* Big Endian */
    sys_put_be16(register_address, &buf[0]);

    /* Put the payload data into the same buffer (Big Endian) */
    for (uint16_t i = 0; i < payload_length; i++) {
        sys_put_be16(payload[i], &buf[2 + (i * 2)]);
    }

    int32_t rc = i2c_write_dt(dev, buf, total_len);
    if (rc != 0) {
        printk("bmv080_i2c_write: i2c_write_dt failed with error: %d\n", rc);
        return E_BMV080_ERROR_HW_WRITE;
    }

    return E_BMV080_OK;
}

static int8_t bmv080_delay_ms(uint32_t period_ms)
{
    k_msleep(period_ms);
    return E_BMV080_OK;
}

static uint32_t bmv080_get_tick_ms(void)
{
    return k_uptime_get_32();
}

static bool getBmv080ID(char *id)
{
  bmv080_status_code_t bmv080_status = bmv080_get_sensor_id(_bmv080_handle, id);
  printk("getBmv080ID\r\n");
  return (bmv080_status == E_BMV080_OK);
}

bool setObstructionDetection(bool obstructed)
{
  bmv080_status_code_t bmv080_status = bmv080_set_parameter(_bmv080_handle, "do_obstruction_detection", (void *)&obstructed);

  return (bmv080_status == E_BMV080_OK);
}

extern uint8_t init_bmv080(){
  bmv080_status_code_t status = E_BMV080_OK;
  bmv080_status_code_t final_status = E_BMV080_OK;
  if (!device_is_ready(bmv080_dev.bus)) {
    printk("I2C bus is not ready!\n");
    return 0;
  }
  printk("BMV080 I2C address: 0x%02X\n", bmv080_dev.addr);

  status = bmv080_open(&_bmv080_handle, (bmv080_sercom_handle_t)&bmv080_dev, bmv080_i2c_read, bmv080_i2c_write, bmv080_delay_ms);
  if (status != E_BMV080_OK) {
    printk("Failed to open BMV080 sensor. Error: %d\n", status);
    final_status = status;
  }

  //bmv080_status_code_t bmv080_status = bmv080_open(&_bmv080_handle, (bmv080_sercom_handle_t)&bmv080_dev, read_16bit_CB,write_16bit_CB,bmv080DelayCb);
  /*
  if(bmv080_status == E_BMV080_ERROR_PRECONDITION_UNSATISFIED) {
    printk("bmv080_open failed, status is: %x \r\n" ,bmv080_status);
  }else{
    printk("it works: %i\r\n",bmv080_status);
    if(_bmv080_handle == NULL){
      printk("still nulls\r\n");
      }else{
        printk("not null\r\n");
      }
  }
  */
}
