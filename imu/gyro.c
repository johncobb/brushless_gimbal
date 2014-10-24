/*
 * gyro.c
 *
 *  Created on: Oct 24, 2014
 *      Author: jcobb
 */



#include <avr/pgmspace.h>
//#include <util/delay.h>
#include "imu.h"
#include "gyro.h"
#include "../util/clock.h"
#include "../util/log.h"

static const char _tag[] PROGMEM = "gyro: ";

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

	uint16_t prev_gyro[3];
	uint16_t gyro[3];
	float gyro_offset[3];

	uint8_t tilt_detected = 0;
	uint16_t gyro_calibration_counter = GYRO_ITERATIONS;

	// TODO: Implement following function in imu
	// Also implement in spike_328p_i2c
	// set to slow mode during calibration
	// mpu.setDLPFMode(MPU6050_DLPF_BW_5);

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

			// TODO: Implement following function in imu
			// Also implement in spike_328p_i2c
			// get rotation
			// mpu.getRotation(&gyro[0], &gyro[1], &gyro[2]);

			for(i=0; i<3; i++)
			{
				gyro_offset[i] = 0;
				prev_gyro[i] = gyro[i];
			}
		}

		// TODO: Implement following function in imu
		// Also implement in spike_328p_i2c
		// get rotation
		// mpu.getRotation(&gyro[0], &gyro[1], &gyro[2]);

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
