/*
 * imu.h
 *
 *  Created on: Oct 23, 2014
 *      Author: jcobb
 */

#ifndef IMU_H_
#define IMU_H_

#include "../util/defines.h"


#define GYRO_FS_250         0x00
#define GYRO_FS_500         0x01
#define GYRO_FS_1000        0x02
#define GYRO_FS_2000        0x03
#define MPU6050_GYRO_FS		GYRO_FS_250

#define IMU_ADDRESS			0x68
#define IMU_BUFFER_LENGTH	14

extern uint8_t imu_address;

enum axis_def {
	ROLL = 0,
	PITCH,
	YAW
};

typedef struct fp_vector {
  float X;
  float Y;
  float Z;
} t_fp_vector_def;

typedef union {
  float   A[3];
  t_fp_vector_def V;
} t_fp_vector;

//********************
// sensor orientation
//********************
typedef struct sensor_axis_def {
  char	idx;
  int  	dir;
} t_sensor_axis_def;

typedef struct sensor_orientation_def {
  t_sensor_axis_def gyro[3];
  t_sensor_axis_def acc[3];
} t_sensor_orientation_def;

extern t_sensor_orientation_def sensor_def;

extern float gyro_scale;
extern int16_t gyro_adc[3];
extern int16_t acc_adc[3];
extern t_fp_vector est_g;
extern float acc_lpf[3];
extern float acc_mag;
extern bool disable_acc_gtest;

extern float acc_compl_filter_const;  // filter constant for complementary filter
extern int16_t acc_25deg;      //** TODO: check
extern int32_t angle[2];  // absolute angle inclination in multiple of 0.01 degree    180 deg = 18000



void imu_init();
void imu_tick();
bool imu_test();
void imu_read6(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz);
void imu_read9(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz, int16_t *mx, int16_t *my, int16_t *mz);
void imu_read_gyros();
void imu_get_rotation(int16_t *x, int16_t *y, int16_t *z);
void imu_get_acceleration(int16_t *x, int16_t *y, int16_t *z);
void imu_get_mag(int16_t *x, int16_t *y, int16_t *z);
void imu_set_dlpf();
void imu_update_gyro_attitude();
void imu_update_acc_attitude();
void imu_get_attitude_angles();
#endif /* IMU_H_ */
