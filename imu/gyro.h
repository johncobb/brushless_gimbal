/*
 * gyro.h
 *
 *  Created on: Oct 24, 2014
 *      Author: jcobb
 */

#ifndef GYRO_H_
#define GYRO_H_

#include "../util/defines.h"

//#define GRAVITY 16384.0f
#define GRAVITY	15500.0f;

void init_resolution_divder();
void gyro_offset_calibration();
void accl_calibration();

#endif /* GYRO_H_ */
