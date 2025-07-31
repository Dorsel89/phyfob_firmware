#include "stcc4Zephyr.h"
void submit_config_stcc4(){
    k_work_submit(&config_work_stcc4);
}
void set_config_stcc4() 
{
    sleep_stcc4(true);
    stcc4_data.timer_interval = stcc4_data.config[1];
    printk("stcc4 config received \n");
    printk("stcc4 interval: %i\n",stcc4_data.timer_interval);
    //Ensure minimum of ? ms
    if (stcc4_data.timer_interval < 3) {stcc4_data.timer_interval = 3;}
    sleep_stcc4(!stcc4_data.config[0]);
}

extern int8_t init_stcc4(){
    stcc4_data.enable = &stcc4_data.config[0];
    stcc4_data.logging = false;
    if(!device_is_ready(stcc4_dev)){
        printk("Device stcc4_dev not ready or not found");
        return false;
    }

    stcc4_init(STCC4_I2C_ADDR_64);

    int16_t error = NO_ERROR;
    sensirion_i2c_hal_init();
    
    error = stcc4_stop_continuous_measurement();
    
    if (error != NO_ERROR) {
            printk("error executing stop_continuous_measurement(): %i\n", error);
            return error;
    }

    stcc4_data.timer_interval = 10; //10*100ms
    k_work_init(&work_stcc4, send_data_stcc4);
	k_work_init(&config_work_stcc4, set_config_stcc4);
    k_timer_init(&timer_stcc4, stcc4_data_ready, NULL);

    return true;
}

extern uint8_t sleep_stcc4(bool SLEEP){
    if(SLEEP){
        k_timer_stop(&timer_stcc4);
        stcc4_stop_continuous_measurement();
        stcc4_enter_sleep_mode();
    }else{
        stcc4_exit_sleep_mode();
        stcc4_start_continuous_measurement();
        k_timer_start(&timer_stcc4, K_MSEC(stcc4_data.timer_interval*100), K_MSEC(stcc4_data.timer_interval*100));
    }    
}

extern uint8_t stcc4_compensate(float t, float rh){
    uint16_t t_u16 = (t+45)*(pow(2,16)-1)/175;
    uint16_t rh_u16 = (rh+6)*(pow(2,16)-1)/125;
    printk("compensation with: t: %i rh: %i \r\n",t_u16,rh_u16);
    return stcc4_set_rht_compensation(t_u16,rh_u16);
}

extern void stcc4_logging(bool l){
    if(l){
        k_timer_start(&timer_stcc4, K_MSEC(logging.interval_ms), K_MSEC(logging.interval_ms));
    }else{
        k_timer_stop(&timer_stcc4);
    }
}

void send_data_stcc4()
{
    int16_t co2_concentration_raw = 0;
    uint16_t temperature_raw = 0;
    uint16_t relative_humidity_raw = 0;
    uint16_t sensor_status_raw = 0;

    if(logging.enable){
        //logging mode
        //sensor is not running - start, measure, sleep
        printk("stcc4 logging mode\r\n");
        stcc4_exit_sleep_mode();
        stcc4_measure_single_shot();
        stcc4_read_measurement_raw(
            &co2_concentration_raw, &temperature_raw, &relative_humidity_raw,
            &sensor_status_raw);
        stcc4_enter_sleep_mode();

    }else{
        //phyphox live mode
        stcc4_read_measurement_raw(
            &co2_concentration_raw, &temperature_raw, &relative_humidity_raw,
            &sensor_status_raw);
        
        printk("stcc4 live mode \r\n");
    }
    stcc4_data.co2 = co2_concentration_raw;
    stcc4_data.array[0]=stcc4_data.co2;
    stcc4_data.array[1]=(k_uptime_get()/1000.0)-global_timestamp;
    printk("send new co2 data: co: %i, t: %i, RH: %i \r\n",co2_concentration_raw,temperature_raw,relative_humidity_raw);

    send_data(SENSOR_STCC4_ID, &stcc4_data.array, 4*2);
}

void stcc4_data_ready()
{
	k_work_submit(&work_stcc4);
}