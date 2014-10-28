/*
 * gyro.c
 *
 *  Created on: Oct 24, 2014
 *      Author: jcobb
 */

#include <limits.h>
#include <avr/pgmspace.h>
//#include <util/delay.h>
#include "imu.h"
#include "gyro.h"
#include "../util/clock.h"
#include "../util/log.h"

static const char _tag[] PROGMEM = "gyro: ";

t_sensor_orientation_def sensor_def = {
    {{0, 1}, {1, 1}, {2, 1}},    // Gyro
    {{0, 1}, {1, 1}, {2, 1}}     // Acc
};

// local prototypes
void init_sensor_orientation_default();

// helper functions for swaping orientation
void swap_char(char *a, char *b) {
	char tmp = *a;
	*a = *b;
	*b = tmp;
}

void swap_int(int *a, int * b){
	int tmp = *a;
	*a = *b;
	*b = tmp;
}


void init_resolution_divder()
{
	if(MPU6050_GYRO_FS == 0x00) resolution_divider = 131.0;
	if(MPU6050_GYRO_FS == 0x01) resolution_divider = 65.5;
	if(MPU6050_GYRO_FS == 0x02) resolution_divider = 32.8;
	if(MPU6050_GYRO_FS == 0x03) resolution_divider = 16.4;
}

// initial gyro offset calibration
// motion detection
// keep board still until complete

#define 					TOL	64
#define GYRO_ITERATIONS		4000

void gyro_offset_calibration()
{
	uint8_t i;

	int16_t prev_gyro[3];
	int16_t gyro[3];
	float gyro_offset[3];

	int8_t tilt_detected = 0;
	int16_t gyro_calibration_counter = GYRO_ITERATIONS;

	// TODO: Implement following function in spike_328p_i2c
	set_dlpf_mode(MPU6050_DLPF_BW_5);

	// TODO: Possibly implement in a tick process
	// wait 2 seconds
	//_delay_ms(2000);
	delay_millis(2000);

	while(gyro_calibration_counter > 0)
	{
		if(gyro_calibration_counter == GYRO_ITERATIONS)
		{
			// TODO: Possibly implement in a tick process
			//_delay_ms(700);
			delay_millis(2000);

			// TODO: Implement following function in spike_328p_i2c
			get_rotation(&gyro[0], &gyro[1], &gyro[2]);

			for(i=0; i<3; i++)
			{
				gyro_offset[i] = 0;
				prev_gyro[i] = gyro[i];
			}
		}

		get_rotation(&gyro[0], &gyro[1], &gyro[2]);

		for (i=0; i<3; i++)
		{
			if(abs(prev_gyro[i]) - gyro[i] > TOL)
			{
				tilt_detected++;
				break;
			}
		}

		for (i=0; i<3; i++)
		{
			gyro_offset[i] += (float)gyro[i]/GYRO_ITERATIONS;
			prev_gyro[i] = gyro[i];
		}

		gyro_calibration_counter--;
		if(tilt_detected >= 1)
		{
			LOG("gyro calibration failed, retrying...\r\n");
			gyro_calibration_counter = GYRO_ITERATIONS;
			tilt_detected = 0;
		}
	}

	// Put results into integer
	config.gyro_offset_x = gyro_offset[0];
	config.gyro_offset_y = gyro_offset[1];
	config.gyro_offset_z = gyro_offset[2];

	// TODO: Review
	imu_init();
}


#define ACC_ITERATIONS 500
#define ACC_THRESH_FAIL 1000
#define ACC_THRESH_GMIN 3000

uint8_t accl_calibration()
{
	int16_t dev_val[3];
	int16_t min_acc[3] = {INT_MAX, INT_MAX, INT_MAX};
	int16_t max_acc[3] = {INT_MIN, INT_MIN, INT_MIN};

	float acc_offset[3] = {0,};

	delay_millis(500); // delay 0.5 seconds

	for (uint8_t i=0; i<ACC_ITERATIONS; i++){

		get_acceleration(&dev_val[0], &dev_val[1], &dev_val[2]);

		for (uint8_t j=0; j<3; j++){
			acc_offset[j] += (float) dev_val[j]/ACC_ITERATIONS;

			if(dev_val[j] > max_acc[j]){
				max_acc[j] = dev_val[j];
			}

			if(dev_val[j] < min_acc[j]){
				min_acc[j] = dev_val[j];
			}
		}
		delay_millis(2); // 2ms
	}

// Debugging
#ifdef GYRO_DEBUG
	for(uint8_t j=0; j<3; j++) {
		LOG("gyro: avg/max/min[");
		LOG("%d", (uint8_t)j);
		LOG(("] "));
		LOG("%d", acc_offset[j], 3);
		LOG((" / "));
		LOG("%d", max_acc[j]);
		LOG((" / "));
		LOG("%d", min_acc[j]);
		LOG("\r\n");
	}
#endif

	for (uint8_t i=0; i<3; i++){
		if((max_acc[i] - min_acc[i]) > ACC_THRESH_FAIL){
			return -1; // failed
		}
	}

	// store calibration
	if(abs(acc_offset[0]) < ACC_THRESH_GMIN){
		config.acc_offset_x = acc_offset[0];
	}

	if(abs(acc_offset[1]) < ACC_THRESH_GMIN){
		config.acc_offset_y = acc_offset[1];
	}

	if(abs(acc_offset[2]) < ACC_THRESH_GMIN){
		config.acc_offset_z = acc_offset[2];
	}

	return 0;

}

void init_sensor_orientation_default()
{
	// channel assignment
	sensor_def.gyro[ROLL].idx = 0;
	sensor_def.gyro[PITCH].idx = 1;
	sensor_def.gyro[YAW].idx = 2;

	sensor_def.acc[ROLL].idx = 1;
	sensor_def.acc[PITCH].idx = 0;
	sensor_def.acc[YAW].idx = 2;

	// direction
	sensor_def.gyro[ROLL].dir = 1;
	sensor_def.gyro[PITCH].dir = -1;
	sensor_def.gyro[YAW].dir = 1;

	sensor_def.acc[ROLL].dir = 1;
	sensor_def.acc[PITCH].dir = 1;
	sensor_def.acc[YAW].dir = 1;

}

void init_sensor_orientation()
{
	init_sensor_orientation_default();

	if(config.axis_reverse_z){
		// flip over roll
		sensor_def.acc[YAW].dir *= -1;
		sensor_def.acc[ROLL].dir *= -1;
		sensor_def.gyro[PITCH].dir *= -1;
		sensor_def.gyro[YAW].dir *= -1;
	}

	if(config.axis_swap_xy){
		// swap gyro axis
		swap_char(&sensor_def.gyro[ROLL].idx, &sensor_def.gyro[PITCH].idx);
		swap_int(&sensor_def.gyro[ROLL].dir, &sensor_def.gyro[PITCH].dir);
		// swap acc axis
	    swap_char(&sensor_def.acc[ROLL].idx, &sensor_def.acc[PITCH].idx);
	    swap_int(&sensor_def.acc[ROLL].dir, &sensor_def.acc[PITCH].dir);
	}
}

void init_pids()
{
	roll_pid_par.kp = config.gyro_roll_kp/10;
	roll_pid_par.ki = config.gyro_roll_ki/1000;
	roll_pid_par.kd = config.gyro_roll_kd/10/250; // TODO: research need for /250

	pitch_pid_par.kp = config.gyro_pitch_kp/10;
	pitch_pid_par.ki = config.gyro_pitch_ki/1000;
	pitch_pid_par.kd = config.gyro_pitch_kd/10/250; // TODO: research need for /250
}


