/*
 * config.h
 *
 *  Created on: Oct 24, 2014
 *      Author: jcobb
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#include <stdint.h>
#include "defines.h"

typedef uint8_t crc;

struct
{
	bool gyro_calibrated;
	uint16_t	gyro_offset_x;
	uint16_t	gyro_offset_y;
	uint16_t	gyro_offset_z;


	uint8_t crc8;
} config;


float resolution_divider;


#endif /* CONFIG_H_ */
