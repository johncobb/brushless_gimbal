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

void imu_read6(uint16_t *ax, uint16_t *ay, uint16_t *az, uint16_t *gx, uint16_t *gy, uint16_t *gz)
{
	imu_getmotion6(ax, ay, az, gx, gy, gz);
}

void imu_read9(uint16_t *ax, uint16_t *ay, uint16_t *az, uint16_t *gx, uint16_t *gy, uint16_t *gz, uint16_t *mx, uint16_t *my, uint16_t *mz)
{
	imu_getmotion9(ax, ay, az, gx, gy, gz, mx, my, mz);
}


