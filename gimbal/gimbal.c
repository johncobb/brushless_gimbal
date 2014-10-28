/*
 * gimbal.c
 *
 *  Created on: Oct 28, 2014
 *      Author: jcobb
 */

#include "../math/fast_math.h"
#include "../imu/imu.h"
#include "../imu/gyro.h"
#include "gimbal.h"


volatile bool motor_update = false;

int32_t pitch_pid_val;
int32_t roll_pid_val;

//static char p_out_cnt = 0;
//static char t_out_cnt = 0;
//static char t_out_cnt_sub = 0;
//static int state = 0;


void gimbal_init()
{

}

void gimbal_tick()
{
	// flag set in pwm isr
	if(motor_update)
	{
		motor_update = false;

		read_gyros();

		// TODO:
	}
}
