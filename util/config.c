/*
 * config.c
 *
 *  Created on: Oct 24, 2014
 *      Author: jcobb
 */


#include "config.h"

void config_init()
{
	config.gyro_pitch_kp = 20000;
	config.gyro_pitch_ki = 10000;
	config.gyro_pitch_kd = 40000;
	config.gyro_roll_kp = 20000;
	config.gyro_roll_ki = 8000;
	config.gyro_roll_kd = 30000;
	config.axis_reverse_z = true;
	config.axis_swap_xy = false;
	config.motor_number_pitch = 0;
	config.motor_number_roll = 1;
	config.gyro_calibrate = true;
}
