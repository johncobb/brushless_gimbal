/*
 * imu.c
 *
 *  Created on: Oct 23, 2014
 *      Author: jcobb
 */
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "../util/log.h"
#include "../i2c/i2c_driver.h"
#include "imu.h"
#include "gyro.h"


#define IMU_6050	1
#define IMU_9150	2
#define IMU_TYPE	IMU_9150

static const char _tag[] PROGMEM = "imu: ";

uint8_t imu_address = IMU_ADDRESS;

void imu_init()
{
	LOG("imu_init\r\n");

	mpu6050_init();
}

bool imu_test()
{
	return mpu6050_test();
}

void imu_read6(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz)
{
	imu_getmotion6(ax, ay, az, gx, gy, gz);
}

void imu_read9(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz, int16_t *mx, int16_t *my, int16_t *mz)
{
	imu_getmotion9(ax, ay, az, gx, gy, gz, mx, my, mz);
}

void rotation(int16_t *x, int16_t *y, int16_t *z)
{
	get_rotation(x, y, z);
}
void acceleration(int16_t *x, int16_t *y, int16_t *z)
{
	get_acceleration(x, y, z);
}

void read_gyros()
{
	int16_t axis_rotation[3];

	int idx;
	// 414 us

	//read gyros
	rotation(&axis_rotation[0], &axis_rotation[1], &axis_rotation[2]);

	axis_rotation[0] -= config.gyro_offset_x;
	axis_rotation[1] -= config.gyro_offset_y;
	axis_rotation[2] -= config.gyro_offset_z;

	idx = sensor_def.gyro[0].idx;
	gyro_adc[ROLL] = axis_rotation[idx];
	gyro_adc[ROLL] *= sensor_def.gyro[0].dir;

	idx = sensor_def.gyro[1].idx;
	gyro_adc[PITCH] = axis_rotation[idx];
	gyro_adc[PITCH] *= sensor_def.gyro[1].dir;

	idx = sensor_def.gyro[2].idx;
	gyro_adc[YAW] = axis_rotation[idx];
	gyro_adc[YAW] *= sensor_def.gyro[2].dir;

}


