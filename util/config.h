/*
 * config.h
 *
 *  Created on: Oct 24, 2014
 *      Author: jcobb
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#include <stdint.h>
#include <stdbool.h>

typedef uint8_t crc;

typedef struct
{
	int32_t 	gyro_pitch_kp;
	int32_t		gyro_pitch_ki;
	int32_t		gyro_pitch_kd;
	int32_t 	gyro_roll_kp;
	int32_t		gyro_roll_ki;
	int32_t		gyro_roll_kd;
	int16_t 	acc_time_constant;
	int16_t 	angle_offset_pitch;   // angle offset, deg*100
	int16_t 	angle_offset_roll;
	int8_t		dir_motor_pitch;
	int8_t		dir_motor_roll;
	uint8_t		motor_number_pitch;
	uint8_t		motor_number_roll;
	int8_t		max_pwm_motor_pitch;
	int8_t		max_pwm_motor_roll;
	uint16_t	ref_voltage_bat;
	uint16_t	cuttoff_voltage;
	bool 		motor_power_scale;
	bool		enable_gyro;
	bool		enable_acc;
	bool 		axis_reverse_z;
	bool		axis_swap_xy;
	bool		fpv_freeze_pitch;
	bool		fpv_freeze_roll;
	uint8_t 	max_pwm_fpv_pitch;
	uint8_t 	max_pwm_fpv_roll;
	bool 		gyro_calibrate;
	int16_t		gyro_offset_x;
	int16_t		gyro_offset_y;
	int16_t		gyro_offset_z;
	int16_t 	acc_offset_x;
	int16_t 	acc_offset_y;
	int16_t		acc_offset_z;
	uint8_t 	crc8;
} config_t;

extern config_t config;


extern float resolution_divider;


void config_init();
void load_config();
void write_config();

#endif /* CONFIG_H_ */
