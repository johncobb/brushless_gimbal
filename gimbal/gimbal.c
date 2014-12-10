/*

 * gimbal.c
 *
 *  Created on: Oct 28, 2014
 *      Author: jcobb
 */
#include <avr/pgmspace.h>
#include <avr/io.h>
#include <util/delay.h>
#include "../util/config.h"
#include "../util/clock.h"
#include "../util/log.h"
#include "../math/fast_math.h"
#include "../imu/imu.h"
#include "../imu/gyro.h"
#include "../blc/blc.h"
#include "../adc/adc.h"
#include "gimbal.h"

// https://github.com/sparkfun/MPU-9150_Breakout/blob/master/firmware/MPU6050/Examples/MPU9150_AHRS.ino

static const char _tag[] PROGMEM = "gimbal: ";

volatile int8_t gimbal_state = GIM_IDLE;

volatile bool motor_update = false; // driven by isr
volatile bool enable_motor_updates = false; // driven by state machine based on sensor settling

int32_t pitch_pid_val;
int32_t roll_pid_val;
uint16_t max_pwm_motor_pitch_scaled = 0;
uint16_t max_pwm_motor_roll_scaled = 0;


//static char p_out_cnt = 0;
//static char t_out_cnt = 0;
//static char t_out_cnt_sub = 0;
//static int state = 0;

// *** FPV (First Person Video) Variables ***
volatile bool fpv_mode_pitch = false;
volatile bool fpv_mode_roll = false;
volatile bool fpv_mode_freeze_pitch = false;
volatile bool fpv_mode_freeze_roll = false;

int32_t pitch_error_sum = 0;
int32_t roll_error_sum = 0;
int32_t pitch_error_old = 0;
int32_t roll_error_old = 0;

float pitch_phi_set = 0;
float roll_phi_set = 0;
float pitch_angle_set = 0;
float roll_angle_set = 0;

int pitch_motor_drive = 0;
int roll_motor_drive = 0;


// battery voltage
float voltage_bat = 0;
float u_bat_value_f = 0;

void voltage_compensation();

#define GyroMeasError PI * (40.0f / 180.0f)       // gyroscope measurement error in rads/s (shown as 3 deg/s)
#define GyroMeasDrift PI * (0.0f / 180.0f)      // gyroscope measurement drift in rad/s/s (shown as 0.0 deg/s/s)
#define beta sqrt(3.0f / 4.0f) * GyroMeasError   // compute beta
#define zeta sqrt(3.0f / 4.0f) * GyroMeasDrift   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f
void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);

void log_application_data();

enum gimbal_task {
	READACC = 0,
	UPDATEACC = 1,
	VOLTAGECOMP = 2
};

pid_data_t pitch_pid_par;
pid_data_t roll_pid_par;

// task management
static int8_t task_id = 0;
static void task_handler();
static void enter_task(int8_t index);

// overall state management
static void state_handler();
static void enter_state(int8_t state);

// state timeout management
static volatile clock_time_t future = 0;
static bool timeout();
static void set_timer(clock_time_t timeout);

static void enter_task(int8_t index)
{
	task_id = index;
}

static void enter_state(int8_t state)
{
	gimbal_state = state;

	// only GIM_IDLE and GIM_UNLOCKED have timeouts
	if(state == GIM_IDLE) {
		set_timer(1000);
	}
	else if (state == GIM_UNLOCKED) {
		set_timer(LOCK_TIME_SEC);
	}
}

void set_acc_time_constant(int16_t acc_time_constant){
	acc_compl_filter_const = (float)DT_FLOAT/(acc_time_constant + DT_FLOAT);
}

void gimbal_init()
{
	// resolution=131, scale = 0.000133
	gyro_scale = 1.0 / resolution_divider/ 180.0 * PI * DT_FLOAT;
	//LOG("gyro_scale: %d\r\n", gyro_scale*1000);
	set_acc_time_constant(config.acc_time_constant);
	acc_mag = ACC_1G*ACC_1G; // magnitude of 1G initially

	est_g.V.X = 0.0f;
	est_g.V.Y = 0.0f;
	est_g.V.Z = ACC_1G;

	LOG("enter_task: READACC\r\n");
	LOG("enter_state: GIM_IDLE\r\n");
	enter_task(READACC);
	enter_state(GIM_IDLE);
}

static clock_time_t f_timeout = 0;
static clock_time_t f_log_timeout = 0;

void gimbal_tick()
{
	// throttle refresh rate
	if(clock_time() >= f_timeout) {
		f_timeout = clock_time() + 10;
	}
	else {
		return;
	}

	// flag set in pwm isr
	if(motor_update)
	{
		//LOG("motor_update=true\r\n");
		motor_update = false;

		imu_read_gyros();


		if(config.enable_gyro) imu_update_gyro_attitude();
		if(config.enable_acc) imu_update_acc_attitude();


		imu_get_attitude_angles();

		// Throttle output to 10x per second
		if(clock_time() >= f_log_timeout) {
			// attitude estimates
			//LOG("est_g.V x,y,z: %f %f %f\r\n", est_g.V.X, est_g.V.Y, est_g.V.Z);
			LOG("roll/pitch: %d:%d\r\n", angle[ROLL], angle[PITCH]);

			f_log_timeout = clock_time() + 100;
		}



		// Pitch PID
		if(fpv_mode_freeze_pitch == false){
			pitch_pid_val = compute_pid(DT_INT_MS, DT_INT_INV, angle[PITCH], pitch_angle_set *1000, &pitch_error_sum, &pitch_error_old, pitch_pid_par.kp, pitch_pid_par.ki, pitch_pid_par.kd);
			//LOG("pitch_pid_val: %d\r\n", pitch_pid_val);
			pitch_motor_drive = pitch_pid_val * config.dir_motor_pitch;
		}

		// Roll PID
		if(fpv_mode_freeze_roll == false){
			roll_pid_val = compute_pid(DT_INT_MS, DT_INT_INV, angle[ROLL], roll_angle_set *1000, &roll_error_sum, &roll_error_old, roll_pid_par.kp, roll_pid_par.ki, roll_pid_par.kd);
			//LOG("roll_pid_val: %d\r\n", roll_pid_val);
			roll_motor_drive = roll_pid_val * config.dir_motor_roll;
		}

		if(enable_motor_updates){

			//LOG("move_motor pitch/roll pid: %d %d pitch/roll motor: %d %d\r\n", pitch_pid_val, roll_pid_val, pitch_motor_drive, roll_motor_drive);

			move_motor_position_speed(config.motor_number_pitch, pitch_motor_drive, max_pwm_motor_pitch_scaled);
			move_motor_position_speed(config.motor_number_roll, roll_motor_drive, max_pwm_motor_roll_scaled);
		}

		// Evaluate RC signals
		//util_lowpass_filter(&roll_angle_set, roll_phi_set, 0.01);
		//util_lowpass_filter(&pitch_angle_set, pitch_phi_set, 0.01);




		//LOG("pitch_pid_val=%lu pitch_motor_drive=%lu\r\n", pitch_pid_val, pitch_motor_drive);
		//LOG("roll_pid_val=%lu roll_motor_drive=%lu\r\n", roll_pid_val, roll_motor_drive);
		// TODO:
		// Evaluate RC Singals

		task_handler();
		state_handler();
	}
}

const float alpha = 0.5f;
double fXg = 0;
double fYg = 0;
double fZg = 0;


void gimbal_tick2()
{

	if(clock_time() >= f_timeout) {
		f_timeout = clock_time() + 10;
	}
	else {
		return;
	}

	double roll = 0;
	double pitch = 0;
	double Xg = 0;
	double Yg = 0;
	double Zg = 0;

	imu_get_acceleration(&Xg, &Yg, &Zg);

	//Xg -= config.gyro_offset_x;
	//Yg -= config.gyro_offset_y;
	//Zg -= config.gyro_offset_z;

	fXg = Xg * alpha + (fXg * (1.0 - alpha));
	fYg = Yg * alpha + (fYg * (1.0 - alpha));
	fZg = Zg * alpha + (fZg * (1.0 - alpha));

	roll = (atan2(-fYg, fZg)*180.0)/PI;
	pitch = (atan2(fXg, sqrt(fYg*fYg + fZg*fZg))*180.0)/PI;

	// Throttle output to 10x per second
	if(clock_time() >= f_log_timeout) {
		LOG("roll/pitch: %f:%f\r\n", roll, pitch);
		//LOG("%f:%f\r\n", roll, pitch);
		f_log_timeout = clock_time() + 100;
	}

}





int16_t a1=0, a2=0, a3=0, g1=0, g2=0, g3=0, m1=0, m2=0, m3=0;     // raw data arrays reading
uint16_t count = 0;  // used to control display output rate
uint16_t delt_t = 0; // used to control display output rate
uint16_t mcount = 0; // used to control display output rate
uint8_t MagRate = 10;     // read rate for magnetometer data

float roll=0.0f, pitch=0.0f, yaw=0.0f;


float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
clock_time_t now = 0;
clock_time_t last_update = 0;
float deltat = 0.0;
double g_sensitivity = 131.0; // for 250 deg/s, check datasheet




void read_sensor_data();

void read_sensor_data()
{
	// *** ACCEL ***
	imu_get_acceleration(&a1, &a2, &a3);

	// apply calibration offsets
	a1 -= config.acc_offset_x;
	a2 -= config.acc_offset_y;
	a3 -= config.acc_offset_z;



	ax = a1*2.0f/32768.0f; // 2 g full range for accelerometer
	ay = a2*2.0f/32768.0f; // 2 g full range for accelerometer
	az = a3*2.0f/32768.0f; // 2 g full range for accelerometer
	// *** END ACCEL ***

	// *** GYRO ***
	imu_get_rotation(&g1, &g2, &g3);

	// TODO: REVIEW
	g1 = (g1 - config.gyro_offset_x) / g_sensitivity;
	g2 = (g2 - config.gyro_offset_y) / g_sensitivity;
	g3 = (g3 - config.gyro_offset_z) / g_sensitivity;

	gx = g1*250.0f/32768.0f; // 250 deg/s full range for gyroscope
	gy = g2*250.0f/32768.0f; // 250 deg/s full range for gyroscope
	gz = g3*250.0f/32768.0f; // 250 deg/s full range for gyroscope
	// *** END GYRO ***

	// *** MAG ***
	if (mcount > 1000/MagRate) {
		imu_get_mag(&m1, &m2, &m3);
		mx = m1*10.f*1229.0f/4096.0f + 18.0f; // milliGauss (1229 microTesla per 2^12 bits, 10 mG per microTesla)
		my = m2*10.f*1229.0f/4096.0f + 70.0f; // apply calibration offsets in mG that correspond to your environment and magnetometer
		mz = m3*10.f*1229.0f/4096.0f + 270.0f;
		mcount = 0;
	}
	// *** END MAG ***
}

/*
void gimbal_tick3()
{

		mcount++;

		read_sensor_data();

		float accelY = atan2(ax, sqrt( pow(ay, 2) + pow(az, 2))) * 180 / M_PI;
		float accelX = atan2(ay, sqrt( pow(ax, 2) + pow(az, 2))) * 180 / M_PI;



		// Throttle output to .1x per second
		if(clock_time() >= f_log_timeout) {
			f_log_timeout = clock_time() + 100;
			LOG("roll/pitch/yaw %f:%f\r\n", accelY, accelX, 0);
		}
}
*/

#define FREQ	30.0 // sample freq in Hz

clock_time_t start_time;
clock_time_t end_time;
int delay;
void gimbal_tick3()
{


	if(clock_time() >= f_timeout) {
		f_timeout = clock_time() + 33;
	}
	else {
		return;
	}


	start_time = clock_time();

	mcount++;

	read_sensor_data();

	// angles based on accelerometer
	float ay = atan2(ax, sqrt( pow(ay, 2) + pow(az, 2))) * 180 / M_PI;
	float ax = atan2(ay, sqrt( pow(ax, 2) + pow(az, 2))) * 180 / M_PI;

	// angles based on gyro (deg/s)
	double gyroX = gyroX + gx / FREQ;
	double gyroY = gyroY - gy / FREQ;
	double gyroZ = gyroZ + gz / FREQ;

	// complementary filter
	  // tau = DT*(A)/(1-A)
	  // = 0.48sec
	gyroX = gyroX * 0.96 + ax * 0.04;
	gyroY = gyroY * 0.96 + ay * 0.04;

	roll = gyroY * 180.0f / M_PI;
	pitch = gyroX * 180.0f /M_PI;
	yaw = gyroZ * 180.0f / M_PI;


	end_time = clock_time();




	// Throttle output to .1x per second
	if(clock_time() >= f_log_timeout) {
		f_log_timeout = clock_time() + 100;
		//LOG("roll/pitch/yaw %f:%f:%f\r\n", gyroY, gyroX, gyroZ);
		LOG("roll/pitch/yaw %f:%f:%f\r\n", roll, pitch, yaw);
	}

	return;





	// poor mans throtteling

	now = clock_time();

	deltat = ((now - last_update)/1000000.0f);
	last_update = now;


	// Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of the magnetometer;
	// the magnetometer z-axis (+ down) is opposite to z-axis (+ up) of accelerometer and gyro!
	// We have to make some allowance for this orientationmismatch in feeding the output to the quaternion filter.
	// For the MPU-9150, we have chosen a magnetic rotation that keeps the sensor forward along the x-axis just like
	// in the LSM9DS0 sensor. This rotation can be modified to allow any convenient orientation convention.
	// This is ok by aircraft orientation standards!
	// Pass gyro rate as rad/s
	//MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz);





		/*
		LOG("ax: %f", ax);
		LOG(" ay: %f", ay);
		LOG(" az: %f", az);
		LOG(" gx: %f", gx);
		LOG(" gy: %f", gy);
		LOG(" gz: %f", gz);
		LOG(" mx: %f", mx);
		LOG(" my: %f", my);
		LOG(" mz: %f", mz);

		LOG(" q0: %f", q[0]);
		LOG(" qx: %f", q[1]);
		LOG(" qy: %f", q[2]);
		LOG(" qz: %f", q[3]);
		*/

		// Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
		// In this coordinate system, the positive z-axis is down toward Earth.
		// Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
		// Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
		// Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
		// These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
		// Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
		// applied in the correct order which for this configuration is yaw, pitch, and then roll.
		// For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
		yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
		pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
		roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
		pitch *= 180.0f / PI;
		yaw   *= 180.0f / PI - 13.8; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
		roll  *= 180.0f / PI;
		//LOG("yaw/pitch/roll: %.2f %.2f %.2f", yaw, pitch, roll);
		//LOG(" rate: %.2f Hz\r\n", 1.0f/deltat);
		log_application_data();
		f_log_timeout = clock_time() + 100;


}

void log_application_data()
{

	LOG("%f:", ax*1000);
	LOG("%f:", ay*1000);
	LOG("%f:", az*1000);
	LOG("%f:", gx);
	LOG("%f:", gy);
	LOG("%f:", gz);
	LOG("%d:", (int)mx);
	LOG("%d:", (int)my);
	LOG("%d:", (int)mz);
	LOG("%f:", q[1]);
	LOG("%f:", q[2]);
	LOG("%f:", q[3]);
	LOG("%f:", pitch); // pitch
	LOG("%f:", roll); // pitch
	LOG("%f", yaw); // roll
	LOG("\r\n");


	/*
	LOG(" q0: %f", q[0]);
	LOG(" qx: %f", q[1]);
	LOG(" qy: %f", q[2]);
	LOG(" qz: %f", q[3]);
	*/
}

void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
	float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
	float norm;
	float hx, hy, _2bx, _2bz;
	float s1, s2, s3, s4;
	float qDot1, qDot2, qDot3, qDot4;

	// Auxiliary variables to avoid repeated arithmetic
	float _2q1mx;
	float _2q1my;
	float _2q1mz;
	float _2q2mx;
	float _4bx;
	float _4bz;
	float _2q1 = 2.0f * q1;
	float _2q2 = 2.0f * q2;
	float _2q3 = 2.0f * q3;
	float _2q4 = 2.0f * q4;
	float _2q1q3 = 2.0f * q1 * q3;
	float _2q3q4 = 2.0f * q3 * q4;
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q1q4 = q1 * q4;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q2q4 = q2 * q4;
	float q3q3 = q3 * q3;
	float q3q4 = q3 * q4;
	float q4q4 = q4 * q4;

	// Normalise accelerometer measurement
	norm = sqrt(ax * ax + ay * ay + az * az);
	if (norm == 0.0f) return; // handle NaN
	norm = 1.0f/norm;
	ax *= norm;
	ay *= norm;
	az *= norm;

	// Normalise magnetometer measurement
	norm = sqrt(mx * mx + my * my + mz * mz);
	if (norm == 0.0f) return; // handle NaN
	norm = 1.0f/norm;
	mx *= norm;
	my *= norm;
	mz *= norm;

	// Reference direction of Earth's magnetic field
	_2q1mx = 2.0f * q1 * mx;
	_2q1my = 2.0f * q1 * my;
	_2q1mz = 2.0f * q1 * mz;
	_2q2mx = 2.0f * q2 * mx;
	hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
	hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
	_2bx = sqrt(hx * hx + hy * hy);
	_2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
	_4bx = 2.0f * _2bx;
	_4bz = 2.0f * _2bz;

	// Gradient decent algorithm corrective step
	s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
	norm = 1.0f/norm;
	s1 *= norm;
	s2 *= norm;
	s3 *= norm;
	s4 *= norm;

	// Compute rate of change of quaternion
	qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
	qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
	qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
	qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

	// Integrate to yield quaternion
	q1 += qDot1 * deltat;
	q2 += qDot2 * deltat;
	q3 += qDot3 * deltat;
	q4 += qDot4 * deltat;
	norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
	norm = 1.0f/norm;
	q[0] = q1 * norm;
	q[1] = q2 * norm;
	q[2] = q3 * norm;
	q[3] = q4 * norm;

}



static void task_handler()
{
	switch(task_id)
	{
	case READACC:
		//LOG("read_accs\r\n");
		read_accs();
		break;
	case UPDATEACC:
		//LOG("update_acc\r\n");
		update_acc();
		break;
	case VOLTAGECOMP:
		//LOG("voltage_compensation\r\n");
		//voltage_compensation();
		break;
	}

	task_id++;
	if(task_id == 3) task_id = 0;
}


static uint8_t prev_state = 0;
static bool log_state = false;

static void state_handler()
{
	if(gimbal_state != prev_state)
		log_state = true;
	else
		log_state = false;

	prev_state = gimbal_state;

	switch(gimbal_state)
	{
	case GIM_IDLE:
		if(log_state)
			LOG("GIM_IDLE\r\n");
		enable_motor_updates = false;
		//set_acc_tc(2.0);
		disable_acc_gtest = true;
		if(timeout()){
			enter_state(GIM_UNLOCKED);
		}
		break;
	case GIM_UNLOCKED:
		if(log_state)
			LOG("GIM_UNLOCKED\r\n");
		enable_motor_updates = true;
		disable_acc_gtest = true;
		//set_acc_tc(2.0);
		disable_acc_gtest = true;
		if(timeout()){
			enter_state(GIM_LOCKED);
		}
		break;
	case GIM_LOCKED:
		if(log_state)
			LOG("GIM_LOCKED\r\n");
		enable_motor_updates = true;
		disable_acc_gtest = false;
//		if (altModeAccTime) { // alternate time constant mode switch
//		            setACCtc(config.accTimeConstant2);
//		          } else {
//		            setACCtc(config.accTimeConstant);
//		          }
		break;
	case GIM_ERROR:
		LOG("GIM_ERROR\r\n");
		enable_motor_updates = false;
		motor_power_off();
		break;
	}

	task_id++;
	if(task_id == 3) task_id = 0;
}



void voltage_compensation()
{
	int u_bat_value;
	float pwm_motor_scale;

	// measure uBat, 190 us
	u_bat_value = adc_read(PC3); // 118 us

	u_bat_value_f = (float)u_bat_value * UBAT_ADC_SCALE * UBAT_SCALE;

	util_lowpass_filter(&voltage_bat, u_bat_value_f, LOWPASS_K_FLOAT(0.1)); // tau = 1 sec

	if (config.motor_power_scale) {
		// calculate scale factor for motor power (70us)
		if (voltage_bat*100 > config.cuttoff_voltage) {  // switch off if battery voltage below cutoff
			pwm_motor_scale = (config.ref_voltage_bat * 0.01)/voltage_bat;
		} else {
			pwm_motor_scale = 0;
		}
	} else {
		pwm_motor_scale = 1.0;
	}

	// 44us
	if (fpv_mode_freeze_pitch==true) {
		max_pwm_motor_pitch_scaled = config.max_pwm_fpv_pitch * pwm_motor_scale;  // fpv freeze mode
	} else {
		max_pwm_motor_pitch_scaled = config.max_pwm_motor_pitch * pwm_motor_scale;
	}

	max_pwm_motor_pitch_scaled = constrain_int(max_pwm_motor_pitch_scaled, 0, 255);

	if (fpv_mode_freeze_roll == true) {
		max_pwm_motor_roll_scaled = config.max_pwm_fpv_roll * pwm_motor_scale; // fpv freeze mode
	} else {
		max_pwm_motor_roll_scaled = config.max_pwm_motor_roll * pwm_motor_scale;
	}
	max_pwm_motor_roll_scaled = constrain_int(max_pwm_motor_roll_scaled, 0, 255);
}



static void set_timer(clock_time_t timeout)
{
	future = clock_time() + timeout;
}

// timeout routine to demonstrate clock_time
// being kept by pwm isr interrupt
static bool timeout()
{
	bool timeout = false;

	if(clock_time() >= future)
	{
		set_timer(1000);
		timeout = true;

	}

	return timeout;
}



