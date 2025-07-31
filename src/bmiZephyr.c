#include "bmiZephyr.h"
static const struct gpio_dt_spec bmiInt = GPIO_DT_SPEC_GET_OR(BMI_INT, gpios,{0});
static struct gpio_callback bmiInt_cb_data;

static void bmiDataReady(const struct device *dev, struct gpio_callback *cb,uint32_t pins)
{
	k_work_submit(&work_bmi);
}


static struct k_poll_signal spi_done_sig = K_POLL_SIGNAL_INITIALIZER(spi_done_sig);

struct spi_cs_control spim_cs = {
	.gpio = SPI_CS_GPIOS_DT_SPEC_GET(DT_NODELABEL(reg_my_spi_master)),
	.delay = 0,
};
static const struct spi_config spi_cfg = {
	.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB |
				 SPI_MODE_CPOL | SPI_MODE_CPHA,
	.frequency = 125000,
	.slave = 1,
	.cs = &spim_cs,
};

int count = 0;
static BMI3_INTF_RET_TYPE app_spi_read(uint8_t reg_addr, uint8_t *read_data, uint32_t len, void *intf_ptr) {
	uint8_t ret;
	//ret = i2c_write_read(intf_ptr,BMI384_I2C_ADDR,&reg_addr,1,read_data,len);
    uint8_t address[1];
	struct spi_buf transmit_buffer;
	struct spi_buf_set transmit_buffer_set;
	struct spi_buf receive_buffers[2];
	struct spi_buf_set receive_buffers_set;

	address[0] = reg_addr;
	//address[1] = 0x00;

	transmit_buffer.buf = address;
	transmit_buffer.len = sizeof(address);

	transmit_buffer_set.buffers = &transmit_buffer;
	transmit_buffer_set.count = 1;

	receive_buffers[0].buf = NULL;
	receive_buffers[0].len = 1;
	receive_buffers[1].buf = read_data;
	receive_buffers[1].len = len;

	receive_buffers_set.buffers = receive_buffers;
	receive_buffers_set.count = 2;

	//ret = spi_transceive(spi_dev,&spi_cfg, &transmit_buffer_set, &receive_buffers_set);
    ret = spi_transceive(intf_ptr,&spi_cfg, &transmit_buffer_set, &receive_buffers_set);
    //ret = spi_read(intf_ptr,&spi_cfg,&receive_buffers_set);
	k_usleep(2);

	return ret;
}

static BMI3_INTF_RET_TYPE app_spi_write(uint8_t reg_addr, const uint8_t *write_data, uint32_t len, void *intf_ptr) {
    /*
	uint8_t dataBuffer[len+1];
	dataBuffer[0]=reg_addr;
	memcpy(&dataBuffer[0]+1,write_data,len);
	return i2c_write(intf_ptr, &dataBuffer, len+1, BMI384_I2C_ADDR);
    */
    uint8_t address;
	struct spi_buf transmit_buffers[2];
	struct spi_buf_set transmit_buffer_set;
	int ret;

	address = reg_addr;

	transmit_buffers[0].buf = &address;
	transmit_buffers[0].len = 1;
	transmit_buffers[1].buf = write_data;
	transmit_buffers[1].len = len;


	transmit_buffer_set.buffers = transmit_buffers;
	transmit_buffer_set.count = 2;
    printf("test4\r\n");
	ret = spi_write(intf_ptr,&spi_cfg, &transmit_buffer_set);
    printf("test5\r\n");
	k_usleep(2);

	return ret;
}

static void app_us_delay(uint32_t period, void *intf_ptr) {
	return k_busy_wait(period);
}
static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width)
{
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    return (dps / (half_scale)) * (val);
}
extern void send_data_bmi(void){
    return;
    uint16_t int_status = 0;
    bmiResult = bmi323_get_int1_status(&int_status, &bmi3_dev);
    if (int_status & BMI3_INT_STATUS_GYR_DRDY){
        
        if(bmi_data.event_number==0){
            bmi_data.acc_time[0]=(k_uptime_ticks()/32768.0)-global_timestamp;
        }

        //bmiResult = bmi323_get_sensor_data(sensor_data, 2, &bmi3_dev);
        bmiResult = bmi323_get_acc_gyr_fast(sensor_data, 2, &bmi3_dev);
        int currentSample = 0;

        if(BMI_X & *bmi_axis){
            bmi_data.gyr_array[bmi_data.event_number*bmi_data.nOutputs+currentSample+2]=sensor_data[0].sens_data.gyr.x;
            bmi_data.acc_array[bmi_data.event_number*bmi_data.nOutputs+currentSample+2]=sensor_data[1].sens_data.acc.x;
            currentSample+=1;
            }
        if(BMI_Y & *bmi_axis){
            bmi_data.gyr_array[bmi_data.event_number*bmi_data.nOutputs+currentSample+2]=sensor_data[0].sens_data.gyr.y;
            bmi_data.acc_array[bmi_data.event_number*bmi_data.nOutputs+currentSample+2]=sensor_data[1].sens_data.acc.y;
            currentSample+=1;
            }
        if(BMI_Z & *bmi_axis){
            bmi_data.gyr_array[bmi_data.event_number*bmi_data.nOutputs+currentSample+2]=sensor_data[0].sens_data.gyr.z;
            bmi_data.acc_array[bmi_data.event_number*bmi_data.nOutputs+currentSample+2]=sensor_data[1].sens_data.acc.z;
            currentSample+=1;
            }         

        bmi_data.event_number++;
        if(bmi_data.event_number == bmi_data.event_size){
            //printf("TICKS: %d\r\n",k_uptime_ticks());
            int16_t buffer[2];

            memcpy(&buffer[0],&bmi_data.acc_time[0],4);
            bmi_data.gyr_array[0] = buffer[0];
            bmi_data.gyr_array[1] = buffer[1];         
            bmi_data.acc_array[0] = buffer[0];
            bmi_data.acc_array[1] = buffer[1];         
            send_data(SENSOR_BMI323_GYR_ID, &bmi_data.gyr_array, bmi_data.event_size*bmi_data.nOutputs*2+4);           
            send_data(SENSOR_BMI323_ACC_ID, &bmi_data.acc_array, bmi_data.event_size*bmi_data.nOutputs*2+4);           
            bmi_data.event_number=0;
            bmi_data.package_number +=1;
        }
        
    }


}

static int8_t apply_bmi_config(struct bmi3_dev *dev, uint8_t rate,uint8_t acc_range,uint8_t gyr_range, uint8_t average, uint8_t mode)
{
    printf("apply bmi config\r\n");
    /*
    printf("rate: %x \r\n",rate);
    printf("acc_range: %x \r\n",acc_range);
    printf("BMI3_ACC_BW_ODR_QUARTER: %x \r\n",BMI3_ACC_BW_ODR_QUARTER);
    printf("average: %x \r\n",average);
    printf("mode: %x \r\n",mode);
*/
    struct bmi3_map_int map_int = { 0 };

    /* Status of API are returned to this variable. */
    int8_t rslt;

    /* Structure to define the type of sensor and its configurations. */
    //struct bmi3_sens_config config;

    /* Configure the type of feature. */
    config[0].type = BMI323_GYRO;
    config[1].type = BMI323_ACCEL;

    /* Get default configurations for the type of feature selected. */
    rslt = bmi323_get_sensor_config(config, 2, dev);
    //bmi3_error_codes_print_result("Get sensor config", rslt);

    if (rslt == BMI323_OK)
    {
        map_int.gyr_drdy_int = BMI3_INT1;
        
        /* Map data ready interrupt to interrupt pin. */
        rslt = bmi323_map_interrupt(map_int, dev);
        
        //bmi3_error_codes_print_result("Map interrupt", rslt);

        if (rslt == BMI323_OK)
        {
            /* The user can change the following configuration parameters according to their requirement. */
            /* Output Data Rate. By default ODR is set as 100Hz for gyro. */
            config[0].cfg.gyr.odr = rate;// BMI3_GYR_ODR_100HZ;

            /* Gyroscope Angular Rate Measurement Range. By default the range is 2000dps. */
            config[0].cfg.gyr.range = gyr_range;// BMI3_GYR_RANGE_250DPS;
            
            /*  The Gyroscope bandwidth coefficient defines the 3 dB cutoff frequency in relation to the ODR
             *  Value   Name      Description
             *    0   odr_half   BW = gyr_odr/2
             *    1  odr_quarter BW = gyr_odr/4
             */
            config[0].cfg.gyr.bwp = BMI3_GYR_BW_ODR_HALF;

            /* By default the gyro is disabled. Gyro is enabled by selecting the mode. */
            config[0].cfg.gyr.gyr_mode = mode; //BMI3_GYR_MODE_HIGH_PERF;
            
            /* Value    Name    Description
             *  000     avg_1   No averaging; pass sample without filtering
             *  001     avg_2   Averaging of 2 samples
             *  010     avg_4   Averaging of 4 samples
             *  011     avg_8   Averaging of 8 samples
             *  100     avg_16  Averaging of 16 samples
             *  101     avg_32  Averaging of 32 samples
             *  110     avg_64  Averaging of 64 samples
             */
            config[0].cfg.gyr.avg_num = average;//BMI3_GYR_AVG1;

            
            //bmi3_error_codes_print_result("Set sensor config", rslt);

            
            config[1].cfg.acc.odr =rate ;//BMI3_ACC_ODR_100HZ;
            //CRASHES HERE
            config[1].cfg.acc.range = acc_range;// BMI3_ACC_RANGE_2G;
            
            config[1].cfg.acc.bwp = BMI3_ACC_BW_ODR_QUARTER;
            config[1].cfg.acc.avg_num = average; //BMI3_ACC_AVG1;
            config[1].cfg.acc.acc_mode = mode;//BMI3_ACC_MODE_HIGH_PERF;
            
            /* Set the con figurations. */
            rslt = bmi323_set_sensor_config(config, 1, dev);
        }
    }

    return rslt;
}

void submit_config_bmi(){
    /*
    *   config[]
    *   byte 0: enable
    *   byte 1: rate
    *   byte 2: range acc
    *   byte 3: range gyr
    *   byte 4: average
    *   byte 5: enable axis (abs²|z|y|x) 0 is default for x/y/z
    *   byte 6: n-times repeating
    */
   bmi_data.event_size = bmi_data.config[6];
   if(DEBUG){printk("new config:\n\r");};
   if(DEBUG){printk("0 enable: %i\n\r",*bmi_en);};
   if(DEBUG){printk("1 rate: %i\n\r", *bmi_rate);};
   if(DEBUG){printk("2 range acc: %i\n\r",*bmi_range_acc);};
   if(DEBUG){printk("3 range gyr: %i\n\r",*bmi_range_gyr);};
   if(DEBUG){printk("4 average: %i\n\r",*bmi_average);};
   if(DEBUG){printk("5 enable axis: %i\n\r",*bmi_axis);};
   if(DEBUG){printk("6 n-times repeating %i\n\r",*bmi_event_size);};

    bmi_data.package_number = 0;

    if(*bmi_axis == 0){
        *bmi_axis = 0x07; 
    }
    //count how many outputs we have
    bmi_data.nOutputs=0;
    for (int pos = 0; pos <= 3; pos++)
    {
        if((*bmi_axis & (1 << pos)) != 0){
            bmi_data.nOutputs+=1;
        };
    }
      
   if(bmi_data.config[0]){
    apply_bmi_config(&bmi3_dev,*bmi_rate,*bmi_range_acc ,*bmi_range_gyr,*bmi_average,BMI3_GYR_MODE_HIGH_PERF);
   }else{
    apply_bmi_config(&bmi3_dev,BMI3_GYR_ODR_100HZ,BMI3_ACC_RANGE_16G,BMI3_GYR_RANGE_2000DPS,BMI3_GYR_AVG1,BMI3_GYR_MODE_SUSPEND);//sleep
   }

     
    

}

uint8_t sleep_bmi(bool SLEEP){
    if(!SLEEP){
        apply_bmi_config(&bmi3_dev,*bmi_rate,*bmi_range_acc ,*bmi_range_gyr,*bmi_average,BMI3_GYR_MODE_HIGH_PERF);
    }else{
        apply_bmi_config(&bmi3_dev,BMI3_GYR_ODR_100HZ,BMI3_ACC_RANGE_16G,BMI3_GYR_RANGE_2000DPS,BMI3_GYR_AVG1,BMI3_GYR_MODE_SUSPEND);//sleep
        bmi_data.package_number = 0;
   }
}

int8_t configure_int(){
    k_work_init(&work_bmi, send_data_bmi);
    k_work_init(&config_work_bmi, submit_config_bmi);

    bmiResult = gpio_pin_configure_dt(&bmiInt, GPIO_INPUT);
    if (bmiResult != 0) {
		printk("Error %d: failed to configure %s pin %d\n\r",
		       bmiResult, bmiInt.port->name, bmiInt.pin);
		return bmiResult;
	}
    bmiResult = gpio_pin_interrupt_configure_dt(&bmiInt,GPIO_INT_EDGE_FALLING);
	if (bmiResult != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n\r",
			bmiResult, bmiInt.port->name, bmiInt.pin);
		return bmiResult;
	}
    gpio_init_callback(&bmiInt_cb_data, bmiDataReady, BIT(bmiInt.pin));
    gpio_add_callback(bmiInt.port, &bmiInt_cb_data);

}



int8_t init_bmi(){   
    bmi3_dev.intf = BMI3_SPI_INTF;
	bmi3_dev.intf_ptr = bmi_dev;
	bmi3_dev.intf_rslt = bmiResult;
	bmi3_dev.dummy_byte = dByte;
	bmi3_dev.read = app_spi_read;
	bmi3_dev.write = app_spi_write;
	bmi3_dev.delay_us = app_us_delay;
    printf("bmiInitStart\r\n");
	bmiResult = bmi3_init(&bmi3_dev);
    
    bmi_data.event_size = 1;
    bmi_data.event_number = 0;
    bmi_data.package_number = 0;
    
    bmi_en = &bmi_data.config[0];
    bmi_rate = &bmi_data.config[1];
    bmi_range_acc = &bmi_data.config[2];
    bmi_range_gyr = &bmi_data.config[3];
    bmi_average = &bmi_data.config[4];
    bmi_axis = &bmi_data.config[5];
    bmi_event_size = &bmi_data.event_size;
    bmi_data.nOutputs=3;

    /*
    *   config[]
    *   byte 0: enable
    *   byte 1: rate
    *   byte 2: range acc
    *   byte 3: range gyr
    *   byte 4: average
    *   byte 5: enable axis (abs²|z|y|x) 0 is default for x/y/z
    *   byte 6: n-times repeating
    */


    if(DEBUG && bmiResult!=BMI323_OK){printk("bmi3_init error: %i\n\r",bmiResult);}

    if (bmi3_dev.chip_id == BMI323_CHIP_ID)
        {

            bmi3_dev.resolution = BMI323_16_BIT_RESOLUTION;
        }
        else
        {
            bmiResult = BMI323_E_DEV_NOT_FOUND;
        }

   // configure interrupt
   
    configure_int();
    sensor_data[0].type = BMI323_GYRO;
    sensor_data[1].type = BMI323_ACCEL;
    struct bmi3_int_pin_config int_cfg = { 0 };
    bmiResult = bmi323_get_int_pin_config(&int_cfg, &bmi3_dev);
    
    int_cfg.pin_type = BMI3_INT1;
    int_cfg.pin_cfg[0].output_en = BMI3_INT_OUTPUT_ENABLE;
    int_cfg.pin_cfg[0].lvl = BMI3_INT_ACTIVE_LOW;

    int_cfg.pin_cfg[0].od = BMI3_INT_OPEN_DRAIN;
    bmiResult = bmi323_set_int_pin_config(&int_cfg, &bmi3_dev);
    
    bmiResult = apply_bmi_config(&bmi3_dev,BMI3_ACC_ODR_50HZ,BMI3_ACC_RANGE_16G,BMI3_GYR_RANGE_2000DPS,BMI3_GYR_AVG1,BMI3_GYR_MODE_SUSPEND); //mh this should be reworked

    return bmiResult;
};