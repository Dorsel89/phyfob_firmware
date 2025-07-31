#ifndef SENSORS_H
#define SENSORS_H

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>

#define DEBUG				true
#define PRINT_SENSOR_DATA 	true

#define SENSOR_BMP581_ID		1
#define SENSOR_LSM6DSR_ACC_ID	2
#define SENSOR_LSM6DSR_GYR_ID	3
#define SENSOR_HDC_ID			4
#define SENSOR_STCC4_ID			5

#define LOG_MULTIPLIER 70


/**
 * @brief Helper function for converting struct sensor_value to float.
 *
 * @param val A pointer to a sensor_value struct.
 * @return The converted value.
 */
/*
static inline float sensor_value_to_float(const struct sensor_value *val) 
{
	return (float)val->val1 + (float)val->val2 / 1000000;
}
*/

extern uint8_t OPERATING_MODE;
#define MODE_SLEEPING	0
#define MODE_PHYPHOX	1
#define MODE_BTHOME		2


typedef struct {
	int max_events;
	int current_event;
	float pressure;
	float temperature;
	float timestamp;
	float array[3*10];
	uint8_t config[20];
	uint8_t *enable;
	uint8_t *oversampling_p;
	uint8_t *iir;
	bool logging;
	bool live;
}BMP;

typedef struct {
	int event_size;
	int event_number;
	int nOutputs;
	int16_t acc_array[3*40+2];//x,y,z,t
	float acc_time[40];
	int16_t gyr_array[3*40+2];//x,y,z,t
	float gyr_time[40];
	uint8_t config[20];
	uint32_t package_number;
}LSM;

typedef struct {
	uint8_t *enable;
	bool logging;
	float co2;
	float array[2];
	uint16_t timer_interval;
	uint8_t config[20];
}STCC4;

typedef struct {
	bool enable;
	uint16_t interval_ms;
}LOGGING;

typedef struct {
	uint8_t config[20];
}PHYPHOX_EVENT;

extern BMP bmp_data;
extern LSM lsm_data;
extern LOGGING logging;
extern STCC4 stcc4_data;
//extern SHTC shtc_data;

extern PHYPHOX_EVENT event_data;
extern float global_timestamp;


typedef struct {
	int arraysize;
	int current_event;
	int max_events;
	uint32_t logging_start;
	uint8_t data[176*LOG_MULTIPLIER];
	int write_to_position;
	float average_sum;
	uint16_t average_n;
	float last_save;
}DATALOGGING;

extern DATALOGGING LOG;
#endif // SENSORS_H