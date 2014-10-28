/*
 * gimbal.h
 *
 *  Created on: Oct 28, 2014
 *      Author: jcobb
 */

#ifndef GIMBAL_H_
#define GIMBAL_H_

#include <stdbool.h>

volatile extern bool motor_update;

void gimbal_init();
void gimbal_tick();


#endif /* GIMBAL_H_ */
