/*
 * config.h
 *
 *  Created on: Oct 24, 2014
 *      Author: jcobb
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#include <stdint.h>
#include "defines.h"

typedef uint8_t crc;

struct
{
	int32_t 	gyro_pitch_kp;
	int32_t		gyro_pitch_ki;
	int32_t		gyro_pitch_kd;
	int32_t 	gyro_roll_kp;
	int32_t		gyro_roll_ki;
	int32_t		gyro_roll_kd;

	bool 		axis_reverse_z;
	bool		axis_swap_xy;

	bool 		gyro_calibrate;
	uint16_t	gyro_offset_x;
	uint16_t	gyro_offset_y;
	uint16_t	gyro_offset_z;
	uint16_t 	acc_offset_x;
	uint16_t 	acc_offset_y;
	uint16_t	acc_offset_z;
	uint8_t		motor_number_pitch;
	uint8_t		motor_number_roll;
	uint8_t 	crc8;
} config;


float resolution_divider;


void config_init();

#endif /* CONFIG_H_ */
