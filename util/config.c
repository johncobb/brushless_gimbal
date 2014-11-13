/*
 * config.c
 *
 *  Created on: Oct 24, 2014
 *      Author: jcobb
 */


#include "config.h"
#include "../eeprom/eeprom.h"
#include "../math/fast_math.h"

config_t config;
float resolution_divider;

void config_init()
{
	config.gyro_pitch_kp = 20000;
	config.gyro_pitch_ki = 10000;
	config.gyro_pitch_kd = 40000;
	config.gyro_roll_kp = 20000;
	config.gyro_roll_ki = 8000;
	config.gyro_roll_kd = 30000;
	config.acc_time_constant = 7;
	config.angle_offset_pitch = 0;
	config.angle_offset_roll = 0;
	config.dir_motor_pitch = 1;
	config.dir_motor_roll = -1;
	config.motor_number_pitch = 0;
	config.motor_number_roll = 1;
	config.max_pwm_motor_pitch = 80;
	config.max_pwm_motor_roll = 80;
	config.ref_voltage_bat = 800;
	config.cuttoff_voltage = 600;
	config.motor_power_scale = 0;
	config.enable_gyro = true;
	config.enable_acc = true;
	config.axis_reverse_z = true;
	config.axis_swap_xy = false;
	config.fpv_freeze_pitch = false;
	config.fpv_freeze_roll = false;
	config.max_pwm_fpv_pitch = 80;
	config.max_pwm_fpv_roll = 80;
	config.gyro_calibrate = true;
}


void load_config()
{
	//eeprom_read(0, config);

	if (config.crc8 == crc_slow((crc *)&config, sizeof(config)-1))
	{
		//updateAllParameters();
	} else {
		// crc failed intialize directly here, as readEEPROM is void
		//printMessage(MSG_WARNING, F("EEPROM CRC failed, initialize to default"));
		//setDefaultParameters();
		config.crc8 = crc_slow((crc *)&config, sizeof(config)-1); // set proper CRC
		//eeprom_write(0, config);
	}
}

void write_config()
{
	config.crc8 = crc_slow((crc *)&config, sizeof(config)-1); // set proper CRC
	//eeprom_write(0, config);
}
