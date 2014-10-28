/*
 * gyro.h
 *
 *  Created on: Oct 24, 2014
 *      Author: jcobb
 */

#ifndef GYRO_H_
#define GYRO_H_
#include <stdint.h>

//#include "../util/defines.h"

#define GYRO_DEBUG		1

enum axis_def {
	ROLL,
	PITCH,
	YAW
};
//********************
// sensor orientation
//********************
typedef struct sensor_axis_def {
  char idx;
  int  dir;
} t_sensor_axis_def;

typedef struct sensor_orientation_def {
  t_sensor_axis_def gyro[3];
  t_sensor_axis_def acc[3];
} t_sensor_orientation_def;

extern t_sensor_orientation_def sensor_def;

static float gyro_scale=0;
static int16_t gyro_adc[3];
static int16_t acc_adc[3];


typedef struct pid_data {
  int32_t   kp, ki, kd;
} pid_data_t;

pid_data_t pitch_pid_par;
pid_data_t roll_pid_par;

//#define GRAVITY 16384.0f
#define GRAVITY	15500.0f;

void init_resolution_divder();
void gyro_offset_calibration();
uint8_t accl_calibration();
void init_sensor_orientation();
void init_pids();

#endif /* GYRO_H_ */
