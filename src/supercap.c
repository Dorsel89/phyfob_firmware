#include "supercap.h"

SUPERCAP supercap_data;

static const struct adc_dt_spec supercap_adc = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0);

void submit_config_supercap(void)
{
    k_work_submit(&config_work_supercap);
}

void set_config_supercap(void)
{
    supercap_data.timer_interval = supercap_data.config[1] * 100;
    if (supercap_data.timer_interval < 100) { supercap_data.timer_interval = 100; }

    if (supercap_data.config[0]) {
        k_timer_start(&timer_supercap, K_MSEC(supercap_data.timer_interval), K_MSEC(supercap_data.timer_interval));
    } else {
        k_timer_stop(&timer_supercap);
    }
}

extern int8_t init_supercap(void)
{
    if (!adc_is_ready_dt(&supercap_adc)) {
        printk("supercap: ADC device not ready\r\n");
        return -1;
    }

    int err = adc_channel_setup_dt(&supercap_adc);
    if (err) {
        printk("supercap: adc_channel_setup failed, err=%d\r\n", err);
        return err;
    }

    supercap_data.timer_interval = 1000;
    k_work_init(&work_supercap, send_data_supercap);
    k_work_init(&config_work_supercap, set_config_supercap);
    k_timer_init(&timer_supercap, supercap_data_ready, NULL);
    return 0;
}

void send_data_supercap(void)
{
    int16_t raw;
    struct adc_sequence sequence = {
        .buffer = &raw,
        .buffer_size = sizeof(raw),
    };

    int err = adc_sequence_init_dt(&supercap_adc, &sequence);
    if (err) {
        printk("supercap: adc_sequence_init failed, err=%d\r\n", err);
        return;
    }

    err = adc_read_dt(&supercap_adc, &sequence);
    if (err) {
        printk("supercap: adc_read failed, err=%d\r\n", err);
        return;
    }

    int32_t val_mv = raw;
    adc_raw_to_millivolts_dt(&supercap_adc, &val_mv);

    supercap_data.voltage = (val_mv / 1000.0f) * SUPERCAP_DIVIDER_RATIO;
    supercap_data.array[0] = supercap_data.voltage;

    printk("supercap: %.2f V\r\n", (double)supercap_data.voltage);

    send_data(SENSOR_SUPERCAP_ID, supercap_data.array, 2 * 4);
}

void supercap_data_ready(void)
{
    supercap_data.array[1] = (k_uptime_get() / 1000.0f) - global_timestamp;
    k_work_submit(&work_supercap);
}
