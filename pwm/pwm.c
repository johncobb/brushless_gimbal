/*
 * pwm.c
 *
 *  Created on: Oct 13, 2014
 *      Author: jcobb
 */

#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>
#include <stdbool.h>
#include "../util/clock.h"
#include "../gimbal/gimbal.h"
#include "pwm.h"



void timer0_isr_emulation();

volatile uint8_t duty_cycle_channel0 = 0;
volatile uint8_t duty_cycle_channel1 = 0;
volatile uint8_t duty_cycle_channel2 = 0;
volatile uint8_t duty_cycle_channel3 = 0;
volatile uint8_t duty_cycle_channel4 = 0;
volatile uint8_t duty_cycle_channel5 = 0;

// Hadware PMW
// Since we are using TCCR0A and TCCR0B in clock they need to be commented
// Another option is to use soft_pwm

void pwm_init()
{
	duty_cycle_channel0 = 0;
	duty_cycle_channel1 = 0;


	DDRD |= (1<<DDD3);
	DDRD |= (1<<DDD5);
	DDRD |= (1<<DDD6);

	DDRB |= (1<<DDB1);
	DDRB |= (1<<DDB2);
	DDRB |= (1<<DDB3);


	// Fast PWM overflow on REG A and REG B

	// Commented due to clock
	TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM00) | _BV(WGM00);
	TCCR0B = _BV(CS00);
	TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM00) | _BV(WGM10);
	TCCR1B = _BV(CS10);
	TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM00) | _BV(WGM20);
	TCCR2B = _BV(CS20);

	// Enable 3 Timers

	// Commented due to clock
	//TIMSK0 = _BV(TOIE0);
	TIMSK1 = _BV(TOIE1);
	//TIMSK2 = _BV(TOIE2);

	// Set interrupt enable
	sei();

	// Set duty cycle
	OCR0A = 0;
	OCR0B = 0;
	OCR1A = 0;
	OCR1B = 0;
	OCR2A = 0;
	OCR2B = 0;
}

void pwm_setval(uint8_t val, uint8_t channel)
{
	if(channel == 0){
		duty_cycle_channel0 = val;
	} else if(channel == 1) {
		duty_cycle_channel1 = val;
	} else if (channel == 2) {
		duty_cycle_channel2 = val;
	} else if (channel == 3) {
			duty_cycle_channel3 = val;
	} else if (channel == 4) {
			duty_cycle_channel4 = val;
	} else if (channel == 5) {
			duty_cycle_channel5 = val;
	}
}


volatile uint8_t frequency_counter = 0;

ISR(TIMER1_OVF_vect)
{
	frequency_counter++;

	if(frequency_counter == 32)
	{
		// Commented due to clock

		OCR0A = duty_cycle_channel0;
		OCR0B = duty_cycle_channel1;
		OCR1A = duty_cycle_channel5;
		OCR1B = duty_cycle_channel4;
		OCR2A = duty_cycle_channel3;
		OCR2B = duty_cycle_channel2;

		frequency_counter = 0;

		// flag to let gimbal.c know we're ready
		motor_update = true;
	}

	// emulate timer
	timer0_isr_emulation();

}

void timer0_isr_emulation()
{
	// if 31 == 31 bit-wise
	if((frequency_counter & 0x1f) == 0)
	{
		isr_tick();
	}
}


void pwm_tick()
{
	_delay_ms(50);
}

/*
int8_t pwm_sinus_array[256];
for(int i=0; i<10; i++)
{
	pwm_sinus_array[i] = sin(2.0 * i / 10 * 3.14159265) * 127;
}
*/

/*
void pwm_init()
{

	OCR0A = 128;

	TCCR0A |= _BV(COM0A1);
	TCCR0A |= _BV(WGM01) | _BV(WGM00);

	TCCR0B |= _BV(CS01);
}

ISR(PCINT0_vect)
{

}
*/
