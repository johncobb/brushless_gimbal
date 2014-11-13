/*
 * blc.h
 *
 *  Created on: Oct 28, 2014
 *      Author: jcobb
 */

#ifndef BLC_H_
#define BLC_H_


#define PWM_A_MOTOR0	OCR0A
#define PWM_B_MOTOR0	OCR0B
#define PWM_C_MOTOR0	OCR2B

#define PWM_A_MOTOR1	0CR2A
#define PMW_B_MOTOR1	OCR1B
#define PWM_C_MOTOR1	OCR1A

extern int8_t pwm_sinus_array[256];

void blc_init();
void blc_tick();
void move_motor_position_speed(uint8_t motor, uint8_t motor_pos, uint16_t max_pwm);
void calc_sinus_array();
void motor_power_off();

#endif /* BLC_H_ */
