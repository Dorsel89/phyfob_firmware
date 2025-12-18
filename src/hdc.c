#include "hdc.h"
//HDC hdc_data;

void bthome_mode(){
    hdc_data.timer_interval = logging.interval_s*1000;
    k_timer_start(&timer_hdc, K_MSEC(hdc_data.timer_interval), K_MSEC(hdc_data.timer_interval));
}
extern bool init_hdc() 
{   
    if(!device_is_ready(hdc_dev)){
        printk("Device not ready or not found");
        return false;
    }
    hdc_data.timer_interval = logging.interval_s*1000;
    k_work_init(&work_hdc, send_data_hdc);
	k_work_init(&config_work_hdc, set_config_hdc);
    k_timer_init(&timer_hdc, hdc_data_ready, NULL);
    //OPERATING_MODE = MODE_BTHOME;
    //bthome_mode();
    sleep_hdc(!logging.enable);

    return true;
}

extern void sleep_hdc(bool sleep) 
{
    if (sleep) {
        k_timer_stop(&timer_hdc);
    }
    else{
        k_timer_start(&timer_hdc, K_MSEC(hdc_data.timer_interval), K_MSEC(hdc_data.timer_interval));
    }
}

extern void hdc_logging(bool l){
    if(l){
        k_timer_start(&timer_hdc, K_MSEC(logging.interval_s*1000), K_MSEC(logging.interval_s*1000));
    }else{
        k_timer_stop(&timer_hdc);
    }
}

void hdc_data_ready()
{
	k_work_submit(&work_hdc);
}

void send_data_hdc()
{
    sensor_sample_fetch(hdc_dev);
    sensor_channel_get(hdc_dev, SENSOR_CHAN_AMBIENT_TEMP, &hdc_temp);
    sensor_channel_get(hdc_dev, SENSOR_CHAN_HUMIDITY, &hdc_humid);
    
    hdc_data.temperature = sensor_value_to_float(&hdc_temp);
    hdc_data.humidity = sensor_value_to_float(&hdc_humid);
    if(OPERATING_MODE == MODE_BTHOME){
        int16_t temp = (int16_t)hdc_data.temperature*100;
        uint16_t hum = (int16_t)hdc_data.humidity*100;
        printk("new data hdc t: %i h: %i \r\n",temp,hum);
        uint8_t dat[4];
        memcpy(&dat[0],&temp,2);
        memcpy(&dat[0+2],&hum,2);
        update_advertising(dat[0],dat[1],dat[2],dat[3]);
        return;
    }

    if(logging.enable){
        stcc4_compensate(hdc_data.temperature,hdc_data.humidity);
        return;
    }

    float timestamp = k_uptime_get() /1000.0;
    hdc_data.timestamp = timestamp;
    hdc_data.array[0] = hdc_data.temperature;
    hdc_data.array[1] = hdc_data.humidity;
    hdc_data.array[2] = hdc_data.timestamp-global_timestamp;

    
    send_data(SENSOR_HDC_ID, &hdc_data.array, 4*3);
}

void set_config_hdc() 
{
    sleep_hdc(true);
    hdc_data.timer_interval = hdc_data.config[1]*100;
    printk("hdc config received \n");
    printk("hdc interval: %i\n",hdc_data.timer_interval);
    //Ensure minimum of 30ms
    if (hdc_data.timer_interval < 300) {hdc_data.timer_interval = 300;}
    sleep_hdc(!hdc_data.config[0]);
}

extern void submit_config_hdc()
{	
    k_work_submit(&config_work_hdc);
}