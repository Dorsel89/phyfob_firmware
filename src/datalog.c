#include "datalog.h"

#include "sensors.h"
#include "ble.h"
#include "bmpZephyr.h"

#include <zephyr/storage/flash_map.h>
#include <pm_config.h>
#include <string.h>

#define DATALOG_SLOT_SIZE   32
#define DATALOG_SECTOR_SIZE 4096

static const struct flash_area *datalog_fa;
static uint32_t ring_size;
static uint32_t write_offset;
static uint8_t active_mask;
static uint8_t active_record_size;

static struct k_timer timer_datalog;
static struct k_work work_datalog;
static bool dump_in_progress;

static uint8_t datalog_record_size(uint8_t mask)
{
    uint8_t fields = 0;
    for (uint8_t bit = 0; bit < 4; bit++) {
        if (mask & (1 << bit)) {
            fields++;
        }
    }
    return 4 + 4 * fields;
}

void datalog_erase(void)
{
    printk("datalog: erasing ring buffer (%u bytes)\r\n", ring_size);
    flash_area_erase(datalog_fa, 0, ring_size);
    write_offset = 0;
}

static void datalog_tick(struct k_work *work)
{
    /* Pause background logging while any phone is connected: BMP/HDC/STCC4
     * share their power-mode state with the live-view experiments, so
     * logging and live streaming would otherwise fight over the same
     * sensor resources. */
    if (!logging.enable || active_mask == 0) {
        printk("datalog: tick skipped (logging.enable=%d, mask=0x%02x)\r\n",
               logging.enable, active_mask);
        return;
    }
    if (dump_in_progress) {
        printk("datalog: tick skipped, dump in progress\r\n");
        return;
    }

    /* Trigger each selected sensor's measurement right here, synchronously,
     * instead of reading whatever a separately-timed background sample
     * last cached - that could be up to a full interval stale depending on
     * the (uncontrolled) phase between the two timers. */
    float co2 = 0, temperature = 0, humidity = 0, pressure = 0;

    if (active_mask & DATALOG_SENSOR_CO2) {
        stcc4_read_once(&co2);
    }
    if (active_mask & (DATALOG_SENSOR_TEMP | DATALOG_SENSOR_HUMIDITY)) {
        hdc_read_once(&temperature, &humidity);
    }
    if (active_mask & DATALOG_SENSOR_PRESSURE) {
        bmp_read_once(&pressure, NULL);
    }

    uint8_t buf[DATALOG_SLOT_SIZE];
    uint32_t ts = (uint32_t)(k_uptime_get() / 1000);
    uint8_t pos = 0;

    memcpy(&buf[pos], &ts, 4);
    pos += 4;

    if (active_mask & DATALOG_SENSOR_CO2) {
        memcpy(&buf[pos], &co2, 4);
        pos += 4;
    }
    if (active_mask & DATALOG_SENSOR_TEMP) {
        memcpy(&buf[pos], &temperature, 4);
        pos += 4;
    }
    if (active_mask & DATALOG_SENSOR_HUMIDITY) {
        memcpy(&buf[pos], &humidity, 4);
        pos += 4;
    }
    if (active_mask & DATALOG_SENSOR_PRESSURE) {
        memcpy(&buf[pos], &pressure, 4);
        pos += 4;
    }

    if (write_offset % DATALOG_SECTOR_SIZE == 0) {
        printk("datalog: erasing sector at offset %u\r\n", write_offset);
        flash_area_erase(datalog_fa, write_offset, DATALOG_SECTOR_SIZE);
    }
    flash_area_write(datalog_fa, write_offset, buf, pos);
    printk("datalog: wrote record @%u ts=%u co2=%f t=%f rh=%f p=%f\r\n",
           write_offset, ts, co2, temperature, humidity, pressure);

    write_offset += DATALOG_SLOT_SIZE;
    if (write_offset >= ring_size) {
        printk("datalog: ring buffer wrapped around\r\n");
        write_offset = 0;
    }
}

static void timer_datalog_handler(struct k_timer *timer)
{
    k_work_submit(&work_datalog);
}

static void datalog_dump(uint16_t max_age_minutes)
{
    if (active_mask == 0) {
        printk("datalog: dump requested but no active session (mask=0), nothing to send\r\n");
        return;
    }

    uint32_t total_slots = ring_size / DATALOG_SLOT_SIZE;
    uint32_t last_slot_offset = (total_slots - 1) * DATALOG_SLOT_SIZE;
    uint32_t ts;

    flash_area_read(datalog_fa, last_slot_offset, &ts, 4);
    bool wrapped = (ts != 0xFFFFFFFF);

    uint32_t start_slot = wrapped ? (write_offset / DATALOG_SLOT_SIZE) : 0;
    uint32_t count = wrapped ? total_slots : (write_offset / DATALOG_SLOT_SIZE);

    /* max_age_minutes == 0 means "no limit, dump everything". Otherwise
     * only records whose raw (boot-relative) timestamp is within the last
     * max_age_minutes get sent - the rest are still read (records aren't
     * indexed by time) but skipped before packing them into a packet. */
    bool filter_by_age = (max_age_minutes > 0);
    uint32_t cutoff_ts = 0;
    if (filter_by_age) {
        uint32_t now = (uint32_t)(k_uptime_get() / 1000);
        uint32_t max_age_s = (uint32_t)max_age_minutes * 60;
        cutoff_ts = (max_age_s < now) ? (now - max_age_s) : 0;
    }

    printk("datalog: dumping up to %u records (wrapped=%d, record_size=%u, start_slot=%u, max_age=%u min)\r\n",
           count, wrapped, active_record_size, start_slot, max_age_minutes);

    dump_in_progress = true;

    uint8_t packet[180];
    uint16_t packet_pos = 0;
    uint8_t record[DATALOG_SLOT_SIZE];
    uint32_t sent = 0;

    for (uint32_t i = 0; i < count; i++) {
        uint32_t slot = (start_slot + i) % total_slots;
        flash_area_read(datalog_fa, slot * DATALOG_SLOT_SIZE, record, active_record_size);

        /* Stored timestamps are raw "seconds since boot" (monotonic within
         * a boot, which is all that matters since the ring is erased on
         * every boot - see init_datalog()). Convert to "seconds since the
         * phyphox experiment was started" here, at dump time, matching how
         * every other sensor stream in this project reports time - global_
         * timestamp is only meaningful for the currently running session,
         * not while a record was written in the background. */
        uint32_t raw_ts;
        memcpy(&raw_ts, record, 4);

        if (filter_by_age && raw_ts < cutoff_ts) {
            continue;
        }

        /* Signed int, not float: raw_ts only ever has whole-second
         * resolution, so float's fractional precision buys nothing here,
         * while float32's 24-bit mantissa would start rounding whole
         * seconds once raw_ts exceeds ~194 days of uptime. int32 stays
         * exact up to ~68 years. */
        int32_t rel_ts = (int32_t)raw_ts - (int32_t)global_timestamp;
        memcpy(record, &rel_ts, 4);

        if (packet_pos + active_record_size > sizeof(packet)) {
            send_data(SENSOR_DATALOG_ID, (float *)packet, packet_pos);
            k_sleep(K_MSEC(100));
            packet_pos = 0;
        }
        memcpy(&packet[packet_pos], record, active_record_size);
        packet_pos += active_record_size;
        sent++;
    }
    if (packet_pos > 0) {
        send_data(SENSOR_DATALOG_ID, (float *)packet, packet_pos);
    }
    dump_in_progress = false;
    printk("datalog: dump complete, sent %u of %u records\r\n", sent, count);
}

void datalog_configure(uint8_t cmd, uint8_t sensors_mask, uint16_t interval_s)
{
    printk("CMD: %i\r\n",cmd);
    switch (cmd) {
        case DATALOG_CMD_START:
            printk("START DATALOGGING CMD: mask=0x%02x interval=%us (previous mask=0x%02x)\r\n",
                   sensors_mask, interval_s, active_mask);
            if (sensors_mask != active_mask) {
                printk("datalog: sensor mask changed, erasing previous log\r\n");
                datalog_erase();
                active_mask = sensors_mask;
                active_record_size = datalog_record_size(active_mask);
            }
            if (interval_s == 0) {
                interval_s = 30;
            }
            /* logging.interval_s still feeds the older BTHome/live-view
             * disconnect-triggered path in ble.c; datalog itself now
             * triggers every sensor's measurement directly (see
             * datalog_tick()) instead of relying on their own timers, so
             * it no longer needs hdc_logging()/stcc4_logging()/BMP power
             * mode calls here. */
            logging.interval_s = interval_s;
            printk("datalog: logging active, record_size=%u bytes, ring capacity=%u records\r\n",
                   active_record_size, ring_size / DATALOG_SLOT_SIZE);
            k_timer_start(&timer_datalog, K_SECONDS(interval_s), K_SECONDS(interval_s));
            break;
        case DATALOG_CMD_STOP:
            printk("STOP DATALOGGING CMD\r\n");
            k_timer_stop(&timer_datalog);
            break;
        case DATALOG_CMD_DUMP:
            /* Here, the "interval_s" parameter is reinterpreted as the
             * maximum age of records to send, in minutes (0 = no limit). */
            printk("DUMP DATALOGGING CMD, max_age=%u min\r\n", interval_s);
            datalog_dump(interval_s);
            break;
        case DATALOG_CMD_ERASE:
            printk("ERASE DATALOGGING CMD\r\n");
            datalog_erase();
            break;
        default:
            break;
    }
}

void init_datalog(void)
{
    int err = flash_area_open(PM_DATALOG_ID, &datalog_fa);
    if (err) {
        printk("datalog: failed to open flash area (%d)\r\n", err);
        return;
    }

    ring_size = datalog_fa->fa_size;
    printk("datalog: flash area opened, id=%d size=%u\r\n", PM_DATALOG_ID, ring_size);

    write_offset = 0;
    active_mask = 0;
    active_record_size = 0;
    k_work_init(&work_datalog, datalog_tick);
    k_timer_init(&timer_datalog, timer_datalog_handler, NULL);

    /* Stored timestamps are seconds since boot, so data from a previous
     * boot would have overlapping/non-monotonic timestamps mixed in with
     * new records. Always start with a clean slate rather than resuming a
     * prior session. */
    datalog_erase();
    printk("datalog: ring buffer erased at boot, waiting for start command\r\n");
}
