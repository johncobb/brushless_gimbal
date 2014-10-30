/*
 * gimbal.c
 *
 *  Created on: Oct 28, 2014
 *      Author: jcobb
 */
#include <avr/io.h>
#include "../util/clock.h"
#include "../math/fast_math.h"
#include "../imu/imu.h"
#include "../imu/gyro.h"
#include "../blc/blc.h"
#include "../adc/adc.h"
#include "gimbal.h"

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

// battery voltage
float voltage_bat = 0;
float u_bat_value_f = 0;

void voltage_compensation();


enum gimbal_task {
	READACC = 0,
	UPDATEACC = 1,
	VOLTAGECOMP = 2
};

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


void gimbal_init()
{
	enter_task(READACC);
	enter_state(GIM_IDLE);
}

void gimbal_tick()
{
	// flag set in pwm isr
	if(motor_update)
	{
		motor_update = false;

		imu_read_gyros();

		if(config.enable_gyro) imu_update_gyro_attitude();
		if(config.enable_acc) imu_update_acc_attitude();

		imu_get_attitude_angles();

		// Pitch PID
		if(fpv_mode_freeze_pitch == false){
			pitch_pid_val = compute_pid(DT_INT_MS, DT_INT_INV, angle[PITCH], pitch_angle_set *1000, &pitch_error_sum, &pitch_error_old, pitch_pid_par.kp, pitch_pid_par.ki, pitch_pid_par.kd);
			pitch_motor_drive = pitch_pid_val * config.dir_motor_pitch;
		}

		// Roll PID
		if(fpv_mode_freeze_pitch == false){
			roll_pid_val = compute_pid(DT_INT_MS, DT_INT_INV, angle[ROLL], roll_angle_set *1000, &roll_error_sum, &roll_error_old, roll_pid_par.kp, roll_pid_par.ki, roll_pid_par.kd);
			roll_motor_drive = roll_pid_val * config.dir_motor_roll;
		}

		if(enable_motor_updates){
			move_motor_position_speed(config.motor_number_pitch, pitch_motor_drive, max_pwm_motor_pitch_scaled);
			move_motor_position_speed(config.motor_number_roll, roll_motor_drive, max_pwm_motor_roll_scaled);
		}

		// TODO:
		// Evaluate RC Singals

		task_handler();
		state_handler();
	}
}

static void task_handler()
{
	switch(task_id)
	{
	case READACC:
		read_accs();
		break;
	case UPDATEACC:
		update_acc();
		break;
	case VOLTAGECOMP:
		voltage_compensation();
		break;
	}

	task_id++;
	if(task_id == 3) task_id = 0;
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

static void state_handler()
{
	switch(gimbal_state)
	{
	case GIM_IDLE:
		enable_motor_updates = false;
		//set_acc_tc(2.0);
		disable_acc_gtest = true;
		if(timeout()){
			enter_state(GIM_UNLOCKED);
		}
		break;
	case GIM_UNLOCKED:
		enable_motor_updates = true;
		disable_acc_gtest = true;
		//set_acc_tc(2.0);
		disable_acc_gtest = true;
		if(timeout()){
			enter_state(GIM_LOCKED);
		}
		break;
	case GIM_LOCKED:
		enable_motor_updates = true;
		disable_acc_gtest = false;
//		if (altModeAccTime) { // alternate time constant mode switch
//		            setACCtc(config.accTimeConstant2);
//		          } else {
//		            setACCtc(config.accTimeConstant);
//		          }
		break;
	case GIM_ERROR:
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


