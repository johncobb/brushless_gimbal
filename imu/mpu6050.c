/*
 * mpu6050.c
 *
 *  Created on: Oct 24, 2014
 *      Author: jcobb
 */

#include <string.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "../util/log.h"
#include "../i2c/i2c_driver.h"
#include "imu.h"
#include "mpu6050.h"

static const char _tag[] PROGMEM = "mpu6050: ";

void mpu6050_init()
{
	mpu6050_set_clock_source(MPU6050_CLOCK_PLL_XGYRO);
	mpu6050_set_full_scale_gyro_range(MPU6050_GYRO_FS_250);
	mpu6050_set_full_scale_accel_range(MPU6050_ACCEL_FS_2);
	mpu6050_set_sleep_enabled(false);
}

bool mpu6050_test()
{
	uint8_t device_id = mpu6050_get_device_id();

	LOG("mpu6050_test: (0x")
	LOGHEX(&device_id, 1);
	LOG(")\r\n");

	return device_id == 0x34;
}

void mpu6050_set_clock_source(uint8_t source)
{
	bool status = writeBits(imu_address, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);

	if(status == true)
	{
		LOG("set_clock_source: success\r\n");
	}
	else
		LOG("set_clock_source: failed\r\n");


}

void mpu6050_set_full_scale_gyro_range(uint8_t range)
{
	bool status = writeBits(imu_address, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);

	if(status == true)
	{
		LOG("set_full_scale_gyro_range: success\r\n");
	}
	else
		LOG("set_full_scale_gyro_range: failed\r\n");
}

void mpu6050_set_full_scale_accel_range(uint8_t range)
{
	bool status = writeBits(imu_address, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);

	if(status == true)
	{
		LOG("set_full_scale_accel_range: success\r\n");
	}
	else
		LOG("set_full_scale_accel_range: failed\r\n");
}

void mpu6050_set_sleep_enabled(bool enabled)
{
	bool status = writeBit(imu_address, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);

	if(status == true)
	{
		LOG("set_sleep_enabled: success\r\n");
	}
	else
		LOG("set_sleep_enabled: failed\r\n");
}

uint8_t mpu6050_get_device_id()
{
	uint8_t imu_buffer[IMU_BUFFER_LENGTH];
	memset(imu_buffer, 0, sizeof(imu_buffer));

	readBits(imu_address, MPU6050_RA_WHO_AM_I, MPU6050_WHO_AM_I_BIT, MPU6050_WHO_AM_I_LENGTH, imu_buffer, I2CDEV_DEFAULT_READ_TIMEOUT);
	return imu_buffer[0];
}

void mpu6050_getmotion6(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz)
{
	uint8_t imu_buffer[IMU_BUFFER_LENGTH];
	memset(imu_buffer, 0, sizeof(imu_buffer));

	readBytes(imu_address, MPU6050_RA_ACCEL_XOUT_H, 14, imu_buffer, I2CDEV_DEFAULT_READ_TIMEOUT);
    *ax = (((int16_t)imu_buffer[0]) << 8) | imu_buffer[1];
    *ay = (((int16_t)imu_buffer[2]) << 8) | imu_buffer[3];
    *az = (((int16_t)imu_buffer[4]) << 8) | imu_buffer[5];
    *gx = (((int16_t)imu_buffer[8]) << 8) | imu_buffer[9];
    *gy = (((int16_t)imu_buffer[10]) << 8) | imu_buffer[11];
    *gz = (((int16_t)imu_buffer[12]) << 8) | imu_buffer[13];
}

void mpu6050_getmotion9(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz, int16_t *mx, int16_t *my, int16_t *mz)
{
	uint8_t imu_buffer[IMU_BUFFER_LENGTH];
	memset(imu_buffer, 0, sizeof(imu_buffer));

	// get accel and gyro
	mpu6050_getmotion6(ax, ay, az, gx, gy, gz);

	// set i2c bypass to access magnetometer
	writeByte(imu_address, MPU6050_RA_INT_PIN_CFG, 0x02);
	_delay_ms(10);

	// enable magnetometer
	writeByte(MPU9150_RA_MAG_ADDRESS, 0x0A, 0x01);
	_delay_ms(10);

	// read magnetometer
	readBytes(MPU9150_RA_MAG_ADDRESS, MPU9150_RA_MAG_XOUT_L, 6, imu_buffer, I2CDEV_DEFAULT_READ_TIMEOUT);
	*mx = (((int16_t)imu_buffer[1]) << 8) | imu_buffer[0];
	*my = (((int16_t)imu_buffer[3]) << 8) | imu_buffer[2];
	*mz = (((int16_t)imu_buffer[5]) << 8) | imu_buffer[4];

}

void mpu6050_set_dlpf_mode(uint8_t mode)
{
	writeBits(imu_address, MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, mode);
}

void mpu6050_get_rotation(int16_t *x, int16_t *y, int16_t *z)
{
	uint8_t imu_buffer[IMU_BUFFER_LENGTH];
	memset(imu_buffer, 0, sizeof(imu_buffer));

	readBytes(imu_address, MPU6050_RA_GYRO_XOUT_H, 6, imu_buffer, I2CDEV_DEFAULT_READ_TIMEOUT);
    *x = (((int16_t)imu_buffer[0]) << 8) | imu_buffer[1];
    *y = (((int16_t)imu_buffer[2]) << 8) | imu_buffer[3];
    *z = (((int16_t)imu_buffer[4]) << 8) | imu_buffer[5];
}

void mpu6050_get_acceleration(int16_t *x, int16_t *y, int16_t *z)
{
	uint8_t imu_buffer[IMU_BUFFER_LENGTH];
	memset(imu_buffer, 0, sizeof(imu_buffer));

	readBytes(imu_address, MPU6050_RA_ACCEL_XOUT_H, 6, imu_buffer, I2CDEV_DEFAULT_READ_TIMEOUT);
    *x = (((int16_t)imu_buffer[0]) << 8) | imu_buffer[1];
    *y = (((int16_t)imu_buffer[2]) << 8) | imu_buffer[3];
    *z = (((int16_t)imu_buffer[4]) << 8) | imu_buffer[5];
}

void mpu6050_get_mag(int16_t *x, int16_t *y, int16_t *z)
{
	uint8_t imu_buffer[IMU_BUFFER_LENGTH];
	memset(imu_buffer, 0, sizeof(imu_buffer));

	// set i2c bypass to access magnetometer
	writeByte(imu_address, MPU6050_RA_INT_PIN_CFG, 0x02);
	_delay_ms(10);

	// enable magnetometer
	writeByte(MPU9150_RA_MAG_ADDRESS, 0x0A, 0x01);
	_delay_ms(10);

	// read magnetometer
	readBytes(MPU9150_RA_MAG_ADDRESS, MPU9150_RA_MAG_XOUT_L, 6, imu_buffer, I2CDEV_DEFAULT_READ_TIMEOUT);
	*x = (((int16_t)imu_buffer[1]) << 8) | imu_buffer[0];
	*y = (((int16_t)imu_buffer[3]) << 8) | imu_buffer[2];
	*z = (((int16_t)imu_buffer[5]) << 8) | imu_buffer[4];
}
