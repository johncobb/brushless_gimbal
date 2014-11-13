/*
 * blc.c
 *
 *  Created on: Oct 28, 2014
 *      Author: jcobb
 */

#include <math.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include "../util/config.h"
#include "../pwm/pwm.h"
#include "../math/fast_math.h"
#include "blc.h"

#define N_SIN	256

uint8_t motor_channel_map[2][3] = {{0,1,2}, {3,4,5}};

int8_t pwm_sinus_array[256];

void calc_sinus_array();

void blc_init()
{
	// TODO: Research need for cli and sei here
	//cli();
	calc_sinus_array();
	//sei();
}

void blc_tick()
{

}

void move_motor_position_speed(uint8_t motor, uint8_t motor_pos, uint16_t max_pwm)
{
	//pwm_setval(motor, motor_pos, max_pwm);
	uint16_t pos_step;

	pos_step = motor_pos & 0xff;

	uint16_t pwm_a = pwm_sinus_array[(uint8_t) pos_step];
	uint16_t pwm_b = pwm_sinus_array[(uint8_t) pos_step + 85];
	uint16_t pwm_c = pwm_sinus_array[(uint8_t) pos_step + 170];

	// TODO: Research power factor requirement
	/*
	pwm_a = max_pwm * pwm_a;
	pwm_a = pwm_a >> 8;
	pwm_a += 128;

	pwm_b = max_pwm * pwm_b;
	pwm_b = pwm_b >> 8;
	pwm_b += 128;

	pwm_c = max_pwm * pwm_c;
	pwm_c = pwm_c >> 8;
	pwm_c += 128;
	*/

	// lookup pwm channel for motor (n) and set its duty cycle
	pwm_setval(pwm_a, motor_channel_map[motor][0]);
	pwm_setval(pwm_b, motor_channel_map[motor][1]);
	pwm_setval(pwm_c, motor_channel_map[motor][2]);

}

void calc_sinus_array()
{
	for(int i=0; i<N_SIN; i++)
	{
		pwm_sinus_array[i] = sin(2.0 * i / N_SIN * 3.14159265) * 127.0;
	}
}

void motor_power_off()
{
	move_motor_position_speed(0, 0, 0);
	move_motor_position_speed(0, 0, 0);
}






