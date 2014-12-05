/*
 * imu.c
 *
 *  Created on: Oct 23, 2014
 *      Author: jcobb
 */
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "../i2c/i2c_driver.h"
#include "imu.h"
#include "mpu6050.h"
#include "../util/config.h"
#include "../util/log.h"
#include "../math/fast_math.h"
#include "gyro.h"

static const char _tag[] PROGMEM = "imu: ";

#define IMU_6050	1
#define IMU_9150	2
#define IMU_TYPE	IMU_9150

uint8_t imu_address = IMU_ADDRESS;

float gyro_scale = 0;
int16_t gyro_adc[3] = {0,0,0};
int16_t acc_adc[3] = {0,0,0};
t_fp_vector est_g;
float acc_lpf[3] = {0.0,0.0,0.0};
float acc_mag = 0;
bool disable_acc_gtest = false;

float acc_compl_filter_const = 0;  // filter constant for complementary filter
int16_t acc_25deg = 25;      //** TODO: check
int32_t angle[2]    = {0,0};  // absolute angle inclination in multiple of 0.01 degree    180 deg = 18000

void rotate_v(struct fp_vector * v, float *delta);

void imu_init()
{
	mpu6050_init();
}

bool imu_test()
{
	return mpu6050_test();
}

void imu_read6(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz)
{
	mpu6050_getmotion6(ax, ay, az, gx, gy, gz);
}

void imu_read9(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz, int16_t *mx, int16_t *my, int16_t *mz)
{
	mpu6050_getmotion9(ax, ay, az, gx, gy, gz, mx, my, mz);
}

void imu_get_rotation(int16_t *x, int16_t *y, int16_t *z)
{
	mpu6050_get_rotation(x, y, z);
}
void imu_get_acceleration(int16_t *x, int16_t *y, int16_t *z)
{
	mpu6050_get_acceleration(x, y, z);
}

void imu_get_mag(int16_t *x, int16_t *y, int16_t *z)
{
	mpu6050_get_mag(x, y, z);
}

void imu_read_gyros()
{
	int16_t axis_rotation[3];

	int idx;
	// 414 us

	//read gyros
	imu_get_rotation(&axis_rotation[0], &axis_rotation[1], &axis_rotation[2]);

	// The following output can be confusing because it's the value before subtracting the offset
	//LOG("axis_rotation[x,y,z]: %d %d %d\r\n", axis_rotation[0], axis_rotation[1], axis_rotation[2]);

	//LOG("applying offset...\r\n");
	axis_rotation[0] -= config.gyro_offset_x;
	axis_rotation[1] -= config.gyro_offset_y;
	axis_rotation[2] -= config.gyro_offset_z;
	// The following output will be more rational due to the offsets being applied
	//LOG("axis_rotation[x,y,z]: %d %d %d\r\n", axis_rotation[0], axis_rotation[1], axis_rotation[2]);

	// set sensor orientation
	idx = sensor_def.gyro[0].idx;
	gyro_adc[ROLL] = axis_rotation[idx];
	gyro_adc[ROLL] *= sensor_def.gyro[0].dir;

	idx = sensor_def.gyro[1].idx;
	gyro_adc[PITCH] = axis_rotation[idx];
	gyro_adc[PITCH] *= sensor_def.gyro[1].dir;

	idx = sensor_def.gyro[2].idx;
	gyro_adc[YAW] = axis_rotation[idx];
	gyro_adc[YAW] *= sensor_def.gyro[2].dir;
	// The following output shows sensor data based on real-world orientation
	//LOG("gyro_adc[ROLL,PITCH,YAW]: %d %d %d\r\n", gyro_adc[ROLL], gyro_adc[PITCH], gyro_adc[YAW]);


}

void imu_set_dlpf()
{
	mpu6050_set_dlpf_mode(MPU6050_DLPF_BW_5);
}

void rotate_v(struct fp_vector * v, float *delta)
{
	struct fp_vector v_tmp = *v;
	v->Z -= delta[ROLL]  * v_tmp.X + delta[PITCH] * v_tmp.Y;
	v->X += delta[ROLL]  * v_tmp.Z - delta[YAW]   * v_tmp.Y;
	v->Y += delta[PITCH] * v_tmp.Z + delta[YAW]   * v_tmp.X;
}

void imu_update_gyro_attitude()
{
	//LOG("\r\nstart imu_update_gyro_attitude:\r\n");
	uint8_t axis;

	float delta_gyro_angle[3];

	//LOG("gyro_adc x,y,z: %d %d %d\r\n", gyro_adc[0], gyro_adc[1], gyro_adc[2]);

	// 43us
	for(axis=0; axis<3; axis++){
		delta_gyro_angle[axis] = gyro_adc[axis] * gyro_scale;
		//LOG("delta_gyro_angle[%d] = %d\r\n", axis, (delta_gyro_angle[axis] * 1000));
	}

	rotate_v(&est_g.V, delta_gyro_angle);
	//LOG("est_g.V.X,Y,Z: %d %d %d\r\n", (est_g.V.X * 1000), (est_g.V.Y * 1000), (est_g.V.Z * 1000))
	//LOG("end imu_update_gyro_attitude:\r\n\r\n");
}

void imu_update_acc_attitude()
{
	//LOG("\r\nstart imu_update_acc_attitude:\r\n");
	uint8_t axis;

	// 80 us
	// Apply complimentary filter (Gyro drift correction)
	// If accel magnitude >1.4G or <0.6G and ACC vector outside of the limit range => we neutralize the effect of accelerometers in the angle estimation.
	// To do that, we just skip filter, as EstV already rotated by Gyro
	if (( 36 < acc_mag && acc_mag < 196 ) || disable_acc_gtest) {
		for (axis = 0; axis < 3; axis++) {
		  //utilLP_float(&EstG.A[axis], accLPF[axis], AccComplFilterConst);

			est_g.A[axis] = est_g.A[axis] * (1.0 - acc_compl_filter_const) + acc_lpf[axis] * acc_compl_filter_const; // note: this is different from MultiWii (wrong brackets postion in MultiWii ??.
			//LOG("est_g.A[%d]: %d\r\n", axis, (est_g.A[axis] *1000));
		}
	}

	//LOG("est_g.A[x,y,z]: %d %d %d\r\n", (est_g.A[0] *1000), (est_g.A[1] *1000), (est_g.A[2] *1000));

	//LOG("end imu_update_acc_attitude:\r\n\r\n");

}

void imu_get_attitude_angles()
{

	// good up to this point
	//LOG("est_g.V x,y,z: %d %d %d\r\n", est_g.V.X, est_g.V.Y, est_g.V.Z);
	//LOG("\r\nstart imu_get_attitude_angles:\r\n");

	// attitude of the estimated vector
	// 200us
	angle[ROLL] = config.angle_offset_roll + fast_arc_tan2_deg1000(est_g.V.X , sqrt(est_g.V.Z*est_g.V.Z+est_g.V.Y*est_g.V.Y));
	// 400us
	angle[PITCH] = config.angle_offset_pitch + fast_arc_tan2_deg1000(est_g.V.Y , est_g.V.Z);

	//LOG("roll: %d pitch: %d\r\n", angle[ROLL], angle[PITCH]);

	//LOG("end imu_get_attitude_angles:\r\n\r\n");
}



