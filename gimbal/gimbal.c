/*

 * gimbal.c
 *
 *  Created on: Oct 28, 2014
 *      Author: jcobb
 */
#include <avr/pgmspace.h>
#include <avr/io.h>
#include "../util/config.h"
#include "../util/clock.h"
#include "../util/log.h"
#include "../math/fast_math.h"
#include "../imu/imu.h"
#include "../imu/gyro.h"
#include "../blc/blc.h"
#include "../adc/adc.h"
#include "gimbal.h"

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

#define MAX_DECIMALS	4
int find_i_part(float num);
int find_real_part(float num);

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

	est_g.V.X = 0;
	est_g.V.Y = 0;
	est_g.V.Z = ACC_1G;

	LOG("enter_task: READACC\r\n");
	LOG("enter_state: GIM_IDLE\r\n");
	enter_task(READACC);
	enter_state(GIM_IDLE);
}

void gimbal_tick()
{
	// flag set in pwm isr
	if(motor_update)
	{
		//LOG("motor_update=true\r\n");
		motor_update = false;

		imu_read_gyros();


		if(config.enable_gyro) imu_update_gyro_attitude();
		if(config.enable_acc) imu_update_acc_attitude();

		//LOG("gyro_attitude x:y:z\t%d\t%d\t%d\r\n", find_i_part(est_g.V.X), find_i_part(est_g.V.Y), find_i_part(est_g.V.Z));

		imu_get_attitude_angles();

		//LOG("angle[ROLL,PITCH]: %d %d\r\n", (int16_t) angle[ROLL], (int16_t) angle[PITCH]);


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

			LOG("move_motor pitch/roll pid: %d %d pitch/roll motor: %d %d\r\n", pitch_pid_val, roll_pid_val, pitch_motor_drive, roll_motor_drive);
			move_motor_position_speed(config.motor_number_pitch, pitch_motor_drive, max_pwm_motor_pitch_scaled);
			move_motor_position_speed(config.motor_number_roll, roll_motor_drive, max_pwm_motor_roll_scaled);
		}

		util_lowpass_filter(&pitch_angle_set, pitch_phi_set, 0.01);
		util_lowpass_filter(&roll_angle_set, roll_phi_set, 0.01);


		//LOG("pitch_pid_val=%lu pitch_motor_drive=%lu\r\n", pitch_pid_val, pitch_motor_drive);
		//LOG("roll_pid_val=%lu roll_motor_drive=%lu\r\n", roll_pid_val, roll_motor_drive);
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

int find_i_part(float num)
{
	int i = 10,j;
	int real_part = find_real_part(num);
	//truncate real part
	float temp_float = num - real_part;

	//adjust decimal for MAX_DECIMALS
	for(j = 0; j< MAX_DECIMALS; j++)
	{
		temp_float = temp_float*i;
	}
	//return just the MAX_DECIMAL length of the float
	return (int)(temp_float*i);
}

int find_real_part(float num)
{
	//truncate decimal
	return (int)num;
}



