/*
 * mpu6050.c
 *
 *  Created on: Oct 24, 2014
 *      Author: jcobb
 */

#include <avr/pgmspace.h>
#include <util/delay.h>
#include "../util/log.h"
#include "../i2c/i2c_driver.h"
#include "imu.h"
#include "mpu6050.h"

static const char _tag[] PROGMEM = "mpu6050: ";

void mpu6050_init()
{
	set_clock_source(MPU6050_CLOCK_PLL_XGYRO);
	set_full_scale_gyro_range(MPU6050_GYRO_FS_250);
	set_full_scale_accel_range(MPU6050_ACCEL_FS_2);
	set_sleep_enabled(false);
}

bool mpu6050_test()
{
	return get_device_id() == 0x34;
}

void set_clock_source(uint8_t source)
{
	bool status = writeBits(imu_address, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);

	if(status == true)
	{
		LOG("set_clock_source: success\r\n");
	}
	else
		LOG("set_clock_source: failed\r\n");


}

void set_full_scale_gyro_range(uint8_t range)
{
	bool status = writeBits(imu_address, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);

	if(status == true)
	{
		LOG("set_full_scale_gyro_range: success\r\n");
	}
	else
		LOG("set_full_scale_gyro_range: failed\r\n");
}

void set_full_scale_accel_range(uint8_t range)
{
	bool status = writeBits(imu_address, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);

	if(status == true)
	{
		LOG("set_full_scale_accel_range: success\r\n");
	}
	else
		LOG("set_full_scale_accel_range: failed\r\n");
}

void set_sleep_enabled(bool enabled)
{
	bool status = writeBit(imu_address, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);

	if(status == true)
	{
		LOG("set_sleep_enabled: success\r\n");
	}
	else
		LOG("set_sleep_enabled: failed\r\n");
}

uint8_t get_device_id()
{
	readBits(imu_address, MPU6050_RA_WHO_AM_I, MPU6050_WHO_AM_I_BIT, MPU6050_WHO_AM_I_LENGTH, imu_buffer, I2CDEV_DEFAULT_READ_TIMEOUT);
	LOG("get_device_id: %c\r\n", imu_buffer[0]);
	return imu_buffer[0];
}

void imu_getmotion6(uint16_t *ax, uint16_t *ay, uint16_t *az, uint16_t *gx, uint16_t *gy, uint16_t *gz)
{
	readBytes(imu_address, MPU6050_RA_ACCEL_XOUT_H, 14, imu_buffer, I2CDEV_DEFAULT_READ_TIMEOUT);
    *ax = (((int16_t)imu_buffer[0]) << 8) | imu_buffer[1];
    *ay = (((int16_t)imu_buffer[2]) << 8) | imu_buffer[3];
    *az = (((int16_t)imu_buffer[4]) << 8) | imu_buffer[5];
    *gx = (((int16_t)imu_buffer[8]) << 8) | imu_buffer[9];
    *gy = (((int16_t)imu_buffer[10]) << 8) | imu_buffer[11];
    *gz = (((int16_t)imu_buffer[12]) << 8) | imu_buffer[13];
}

void imu_getmotion9(uint16_t *ax, uint16_t *ay, uint16_t *az, uint16_t *gx, uint16_t *gy, uint16_t *gz, uint16_t *mx, uint16_t *my, uint16_t *mz)
{
	// get accel and gyro
	imu_getmotion6(ax, ay, az, gx, gy, gz);

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

