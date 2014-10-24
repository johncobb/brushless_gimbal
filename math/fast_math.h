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

#define PI	3.14159265

uint32_t constrain_uint32(uint32_t x, uint32_t l, uint32_t h);
float fast_arc_tan(float x);
float fast_arc_tan2(float y, float x);
uint32_t fast_arc_tan2_deg1000(float y, float x);

// Low pass filters
void util_lowpass_filter(float *q, float i, float coeff);
float util_lowpass3rd_filter(float *q, float i, float coeff);


#endif /* FAST_MATH_H_ */
