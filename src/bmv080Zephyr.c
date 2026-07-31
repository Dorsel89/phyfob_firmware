#include "bmv080Zephyr.h"
#include <zephyr/drivers/gpio.h>
#include "sensors.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <zephyr/drivers/i2c.h>

BMV080 bmv080_data;

/* true while a continuous measurement is running (set/cleared on the main
 * thread in bmv080_apply_config()). */
static bool bmv080_running = false;
/* set by submit_config_bmv080() (BT thread), consumed by bmv080_poll() (main). */
static volatile bool bmv080_config_pending = false;
/* set by bmv080_request_stop() (BT connection-callback thread, on disconnect),
 * consumed by bmv080_poll() (main) - same reason as above, library calls must
 * stay on the main thread. */
static volatile bool bmv080_stop_pending = false;

static bmv080_handle_t _bmv080_handle = NULL;  // Handle for the BMV080 sensor.
static struct device *bmv_dev = DEVICE_DT_GET(DT_ALIAS(i2c));
#define BMV080_NODE DT_ALIAS(bmv080)
static const struct i2c_dt_spec bmv080_dev = I2C_DT_SPEC_GET(BMV080_NODE);

/* nRF52832's TWIM EasyDMA length counter is only 8 bits wide, so a single
 * I2C transaction can move at most 255 bytes (enforced by the Zephyr driver,
 * i2c_nrfx_twim_common.c: "Trying to transfer more than the maximum size").
 * The BMV080's event/FIFO burst reads can ask for far more than that in one
 * bmv080_i2c_read() call once a backlog has built up, so large reads are
 * split into <255-byte chunks here, each re-issuing the header - standard
 * practice for I2C FIFO burst reads, since the sensor's read pointer is
 * ASIC-side state that isn't disturbed by the bus STOP between chunks. */
#define BMV080_I2C_MAX_CHUNK_BYTES 252 /* < 255 HW limit, multiple of 2 */

static int8_t bmv080_i2c_read(bmv080_sercom_handle_t sercom_handle, uint16_t header, uint16_t *payload, uint16_t payload_length)
{
    const struct i2c_dt_spec *dev = (const struct i2c_dt_spec *)sercom_handle;
    uint16_t register_address = (uint16_t)(header << 1);
    uint8_t header_buf[2];
    uint8_t chunk_buf[BMV080_I2C_MAX_CHUNK_BYTES];

    /* Big Endian */
    sys_put_be16(register_address, header_buf);

    uint16_t words_done = 0;
    while (words_done < payload_length) {
        uint16_t words_this_chunk = payload_length - words_done;
        if (words_this_chunk > BMV080_I2C_MAX_CHUNK_BYTES / 2) {
            words_this_chunk = BMV080_I2C_MAX_CHUNK_BYTES / 2;
        }
        size_t bytes_this_chunk = (size_t)words_this_chunk * 2;

        int32_t rc = i2c_write_read_dt(dev, header_buf, sizeof(header_buf), chunk_buf, bytes_this_chunk);
        if (rc != 0) {
            printk("bmv080_i2c_read: i2c_write_read_dt failed: %d (word %u/%u)\n", rc, words_done, payload_length);
            return E_BMV080_ERROR_HW_READ;
        }

        /* Convert the read buffer (Big Endian) back into 16-bit words */
        for (uint16_t i = 0; i < words_this_chunk; i++) {
            payload[words_done + i] = sys_get_be16(&chunk_buf[i * 2]);
        }

        words_done += words_this_chunk;
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
  
  if(status == E_BMV080_ERROR_PRECONDITION_UNSATISFIED) {
    printk("bmv080_open failed, status is: %x \r\n" ,status);
  }
  
  /* bmv080_get_sensor_id() (called via getBmv080ID()) writes a 13-byte
   * array - a 1-byte buffer here overflows the stack by 12 bytes, smashing
   * whatever's adjacent (likely init_bmv080()'s own return address) with
   * the ASCII sensor ID string. That's exactly what caused the bus fault
   * jumping to a "PC" that decoded as readable text. */
  char my_id[13] = {0};
  getBmv080ID(my_id);
  printk("bmv080 id: %s\r\n", my_id);

  /* Do NOT start measuring here - like the other sensors, BMV080 stays idle
   * until phyphox starts it via the bmv080_cnfg BLE characteristic. */
  return final_status;
}

static void bmv080_data_ready_cb(bmv080_output_t output, void *callback_parameters)
{
  bmv080_data.pm1 = output.pm1_mass_concentration;
  bmv080_data.pm2_5 = output.pm2_5_mass_concentration;
  bmv080_data.pm10 = output.pm10_mass_concentration;

  bmv080_data.array[0] = bmv080_data.pm1;
  bmv080_data.array[1] = bmv080_data.pm2_5;
  bmv080_data.array[2] = bmv080_data.pm10;
  bmv080_data.array[3] = (k_uptime_get() / 1000.0f) - global_timestamp;

  printk("bmv080: PM1=%.2f PM2.5=%.2f PM10=%.2f ug/m3 (obstructed=%d)\r\n",
         (double)bmv080_data.pm1, (double)bmv080_data.pm2_5, (double)bmv080_data.pm10,
         output.is_obstructed);

  send_data(SENSOR_BMV080_ID, bmv080_data.array, 4 * 4);
}

/* Flag the freshly written config for application by bmv080_poll(). Runs on
 * the BT RX thread, so it must not touch the library (wrong/too-small stack)
 * - it only sets a flag. */
void submit_config_bmv080(void)
{
  bmv080_config_pending = true;
}

/* Called from ble.c's disconnected() (BT connection-callback thread) when the
 * phone/phyphox disconnects. Must not touch the library here (wrong thread/
 * stack) - only flags the stop for bmv080_poll() on the main thread, same as
 * submit_config_bmv080() above. */
void bmv080_request_stop(void)
{
  bmv080_stop_pending = true;
}

/* Apply bmv080_data.config[]. MUST run on the main thread (see header note on
 * the ~10 kB stack requirement). Stops a running measurement first, because
 * bmv080_set_parameter() must be called before starting a measurement. */
static void bmv080_apply_config(void)
{
  bmv080_config_pending = false;

  uint8_t mode = bmv080_data.config[0];
  uint8_t algo = bmv080_data.config[1];
  uint16_t duty_cycling_period_s;
  memcpy(&duty_cycling_period_s, &bmv080_data.config[2], sizeof(duty_cycling_period_s));

  if (bmv080_running) {
    bmv080_stop_measurement(_bmv080_handle);
    bmv080_running = false;
    printk("bmv080: measurement stopped\r\n");
  }

  if (mode == BMV080_MODE_CONTINUOUS) {
    if (algo >= FAST_RESPONSE && algo <= HIGH_PRECISION) {
      bmv080_measurement_algorithm_t a = (bmv080_measurement_algorithm_t)algo;
      bmv080_status_code_t pst = bmv080_set_parameter(_bmv080_handle, "measurement_algorithm", &a);
      if (pst != E_BMV080_OK) {
        printk("bmv080: set measurement_algorithm failed, status=%d\r\n", pst);
      }
    }

    bmv080_status_code_t status = bmv080_start_continuous_measurement(_bmv080_handle);
    if (status != E_BMV080_OK) {
      printk("bmv080: start_continuous_measurement failed, status=%d\r\n", status);
      return;
    }
    bmv080_running = true;
    printk("bmv080: continuous measurement started (algo=%d)\r\n", algo);
    return;
  }

  if (mode == BMV080_MODE_DUTY_CYCLE) {
    if (duty_cycling_period_s > 0) {
      bmv080_status_code_t pst = bmv080_set_parameter(_bmv080_handle, "duty_cycling_period", &duty_cycling_period_s);
      if (pst != E_BMV080_OK) {
        printk("bmv080: set duty_cycling_period failed, status=%d\r\n", pst);
      }
    }

    bmv080_status_code_t status = bmv080_start_duty_cycling_measurement(
        _bmv080_handle, bmv080_get_tick_ms, E_BMV080_DUTY_CYCLING_MODE_0);
    if (status != E_BMV080_OK) {
      printk("bmv080: start_duty_cycling_measurement failed, status=%d\r\n", status);
      return;
    }
    bmv080_running = true;
    printk("bmv080: duty-cycling measurement started (period=%us)\r\n", duty_cycling_period_s);
    return;
  }

  /* mode == BMV080_MODE_STOP (or unknown) -> stay idle */
}

/* Call regularly (see BMV080_POLL_INTERVAL_MS) from main()'s loop - this is
 * NOT run from a k_timer/k_work on the shared system workqueue on purpose:
 * the BMV080 library itself needs ~10kB of stack per its datasheet, and
 * bloating the shared workqueue stack by that much for every unrelated
 * work item would waste a lot of our very limited RAM. Polling from the
 * (already appropriately-sized, see CONFIG_MAIN_STACK_SIZE) main thread
 * avoids that entirely. */
void bmv080_poll(void)
{
  if (bmv080_stop_pending) {
    bmv080_stop_pending = false;
    bmv080_config_pending = false; /* discard any not-yet-applied config */
    if (bmv080_running) {
      bmv080_stop_measurement(_bmv080_handle);
      bmv080_running = false;
      printk("bmv080: measurement stopped (BLE disconnected)\r\n");
    }
    /* Require phyphox to explicitly restart the measurement (config[0]=1)
     * after reconnecting, same as the other sensors' behaviour. */
    bmv080_data.config[0] = 0;
  }

  if (bmv080_config_pending) {
    bmv080_apply_config();
  }

  if (!bmv080_running) {
    return;
  }

  bmv080_status_code_t status = bmv080_serve_interrupt(_bmv080_handle, bmv080_data_ready_cb, NULL);
  if (status != E_BMV080_OK) {
    printk("bmv080: serve_interrupt failed, status=%d\r\n", status);
  }
}
