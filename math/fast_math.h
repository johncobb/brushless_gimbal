/*
 * fast_math.h
 *
 *  Created on: Oct 24, 2014
 *      Author: jcobb
 */

#ifndef FAST_MATH_H_
#define FAST_MATH_H_

#include <math.h>
#include <stdint.h>
#include "../util/config.h"

#define PI	3.14159265

// CRC definitions
#define POLYNOMIAL 0xD8  /* 11011 followed by 0's */

int constrain_int(int x, int l, int h);
int16_t constrain_int16(int16_t x, int16_t l, int16_t h);
int32_t constrain_int32(int32_t x, int32_t l, int32_t h);


float fast_arc_tan(float x);
float fast_arc_tan2(float y, float x);
uint32_t fast_arc_tan2_deg1000(float y, float x);

// Low pass filters
void util_lowpass_filter(float *q, float i, float coeff);
float util_lowpass3rd_filter(float *q, float i, float coeff);

int32_t compute_pid(int32_t dt_ms, int32_t dt_inv, int32_t in, int32_t set_point, int32_t *error_sum, int32_t *error_old, int32_t kp, int16_t ki, int32_t kd);

crc crc_slow(uint8_t const message[], uint8_t size);


#endif /* FAST_MATH_H_ */
