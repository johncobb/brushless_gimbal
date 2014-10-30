/*
 * main.c
 *
 *  Created on: Sep 11, 2014
 *      Author: jcobb
 */

#define F_CPU	8000000

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <util/twi.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include "i2c/i2c.h"
#include "util/log.h"
#include "util/clock.h"
#include "util/config.h"
#include "imu/imu.h"
#include "imu/gyro.h"
#include "pwm/pwm.h"
#include "eeprom/eeprom.h"
#include "blc/blc.h"
#include "gimbal/gimbal.h"



// log debugging
static const char _tag[] PROGMEM = "main: ";
volatile char term_in = 0;

// local porototypes
void read_imu();

void terminal_in_cb(uint8_t c)
{
	term_in = c;
	LOG("input=%c\r\n", c);


}

// IMU variables
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;


void main()
{

	debug_init(terminal_in_cb);
	//clock_init();

	/*
	 * load configuration
	 */
	LOG("config_init...\r\n");
	config_init();

	LOG("brushless gimbal started...\r\n");

	/*
	 * init pwm driver
	 */
	LOG("pwm_init...\r\n");
	pwm_init(); // May interfere with clock or other clock driven sources
	sei();

	/*
	 * init blc motor driver
	 */
	LOG("blc_init...\r\n");
	blc_init(); // InitMotorStuff
	motor_power_off();

	/* --- i2c/gyro/imu initialization ---*/
	// First we need to join the I2C bus
	LOG("joining i2c bus...\r\n");
	i2c_begin();
	LOG("initializing resolution divider...\r\n");
	init_resolution_divider();
	LOG("imu_init...\r\n");
	imu_init();
	// Make sure we are able to communicate with imu
	LOG("running imu test...\r\n");
	if(imu_test()){
		LOG("imu test pass.\r\n");
		if(config.gyro_calibrate == true){
			LOG("running gyro offset calibration...\r\n");
			gyro_offset_calibration();
		}
	} else {
		LOG("imu test failed!\r\n");
	}

	/* --- end i2c/gyro/imu initialization ---*/


	// set sensor orientation
	LOG("init_sensor_orientation...\r\n");
	init_sensor_orientation();

	/*
	 * init pid parameters
	 */
	LOG("init_pids...\r\n");
	init_pids();

	// TODO: Not implemented
	// Init rc variables
	//init_rc();
	// Init rc-input
	//init_rc_pins();

	/*
	 * gimbal is the main state machine
	 * for processing stabilization
	 */
	LOG("gimbal_init...\r\n");
	gimbal_init();


	LOG("starting loop...\r\n");

	while(1)
	{
		_delay_ms(50);
		//gimbal_tick();
		read_imu();
		//(100);
	}
}



void read_imu()
{
	// Rotations
//	imu_get_rotation(&ax, &ay, &az);
//	LOG("rotation x/y/z:\t");
//	LOG("%d\t", ax);
//	LOG("%d\t", ay);
//	LOG("%d\t", az);
//	LOG("\r\n");
//	_delay_ms(100);
//	return;

	// Accelerations
	imu_get_acceleration(&ax, &ay, &az);
	LOG("acceleration x/y/z:\t");
	LOG("%d\t", ax);
	LOG("%d\t", ay);
	LOG("%d\t", az);
	LOG("\r\n");
	_delay_ms(100);
	return;

//	imu_read9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
//	LOG("a/g/m:\t");
//	LOG("%d\t", ax);
//	LOG("%d\t", ay);
//	LOG("%d\t", az);
//	LOG("%d\t", gx);
//	LOG("%d\t", gy);
//	LOG("%d\t", gz);
//	LOG("%d\t", mx);
//	LOG("%d\t", my);
//	LOG("%d\t", mz);
//	LOG("\r\n");
//	_delay_ms(100);
}



/*
void i2c_template()
{
	// TODO: Initial research for basic i2c driver

	i2c_begin_transmission(0x68);
	i2c_write_byte(0x02);

	i2c_end_transmission(0);

	_delay_ms(70); // delay 70 milliseconds


	i2c_request_from(11, 2, 0);




	while(i2c_available() > 2)
	{
		reading = i2c_read();
		reading = reading <<8;
		reading |= i2c_read();
		LOG("i2c_reading: %c\r\n", reading);

		_delay_ms(100);
	}

}
*/


