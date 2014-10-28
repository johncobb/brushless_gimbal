/*
 * imu.h
 *
 *  Created on: Oct 23, 2014
 *      Author: jcobb
 */

#ifndef IMU_H_
#define IMU_H_

#include "../util/defines.h"
#include "mpu6050.h"

#define MPU6050_GYRO_FS		MPU6050_GYRO_FS_250

#define IMU_ADDRESS			0x68
#define IMU_BUFFER_LENGTH	14

uint8_t imu_address;
uint8_t imu_buffer[IMU_BUFFER_LENGTH];

void imu_init();
void imu_tick();
bool imu_test();
void imu_read6(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz);
void imu_read9(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz, int16_t *mx, int16_t *my, int16_t *mz);
void read_gyros();
void get_rotation(int16_t *x, int16_t *y, int16_t *z);
void get_acceleration(int16_t *x, int16_t *y, int16_t *z);
#endif /* IMU_H_ */
