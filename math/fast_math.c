/*
 * fast_math.c
 *
 *  Created on: Oct 24, 2014
 *      Author: jcobb
 */

#include "../util/defines.h"
#include "fast_math.h"


int constrain_int(int x, int l, int h)
{
	if(x <= l){
		return l;
	}
	else if(x >= h){
		return h;
	}
	else{
		return x;
	}
}

int16_t constrain_int16(int16_t x, int16_t l, int16_t h)
{
	if(x <= l){
		return l;
	}
	else if(x >= h){
		return h;
	}
	else{
		return x;
	}
}

int32_t constrain_int32(int32_t x, int32_t l, int32_t h)
{
	if(x <= l){
		return l;
	}
	else if(x >= h){
		return h;
	}
	else{
		return x;
	}
}

//***************************************************************
// ÒEfficient approximations for the arctangent functionÓ,
// Rajan, S. Sichun Wang Inkol, R. Joyal, A., May 2006
//***************************************************************
float fast_arc_tan(float x)
{
	return PI/4.0*x - x*(fabs(x) - 1)*(0.2447 + 0.0663*fabs(x));
}

float fast_arc_tan2(float y, float x)
{

	uint8_t qCode;
	const float pi_2 = PI/2.0;

	float q;
	float z;

	// 6us

	bool swap45 = (fabs(y) > fabs(x));

	// 22us
	if ((y >= 0) && (x >= 0)) { qCode = 0; }
	if ((y >= 0) && (x <= 0)) { qCode = 1; }
	if ((y <= 0) && (x <= 0)) { qCode = 2; }
	if ((y <= 0) && (x >= 0)) { qCode = 3; }

	// 54 us
	if(swap45){
		q = x/y;
	}
	else {
		q = y/x;
	}

	// 92 us

	z = fast_arc_tan(q);

	if (swap45) {
		switch (qCode) {
			case 0: z = pi_2 - z;  break;
			case 1: z = pi_2 - z;  break;
			case 2: z = -pi_2 - z; break;
			case 3: z = -pi_2 - z; break;
		}
	} else {
		switch (qCode) {
			case 0: break;
			case 1: z = PI + z;    break;
			case 2: z = -PI + z;   break;
			case 3: break;
		}
	}

	return z;
}

uint32_t fast_arc_tan2_deg1000(float y, float x)
{
	return 180/PI * 1000 * fast_arc_tan2(y, x);
}

void util_lowpass_filter(float *q, float i, float coeff)
{
	*q += (i - *q) * coeff; // faster version on ATmega
	//*q = *q * (1.0f-coeff) + i * coeff;
}

float util_lowpass3rd_filter(float *q, float i, float coeff)
{
	util_lowpass_filter(&q[2], i, coeff);
	util_lowpass_filter(&q[1], q[2], coeff);
	util_lowpass_filter(&q[0], q[1], coeff);
	return q[0];
}

#define CRC_WIDTH	(8 * sizeof(crc))
#define CRC_TOPBIT	(1 << (CRC_WIDTH -1))


// todo: fix code below
crc crc_slow(uint8_t const message[], uint8_t size)
{
	crc remainder = 0;

	for (uint8_t byte=0; byte<size; ++byte)
	{
		remainder ^= (message[byte] <<(CRC_WIDTH-8));

		for(uint8_t bit = 8; bit > 0; --bit)
		{

			remainder = (remainder << 1) ^ (remainder & CRC_TOPBIT)*POLYNOMIAL;
		}
	}

	return (remainder);
}

int32_t compute_pid(int32_t dt_ms, int32_t dt_inv, int32_t in, int32_t set_point, int32_t *error_sum, int32_t *error_old, int32_t kp, int16_t ki, int32_t kd)
{
	int32_t error = set_point - in;
	int32_t i_error;

	i_error = error * ki * dt_ms;
	i_error = constrain_int32(i_error, -(int32_t) 1000*100, (int32_t)1000*100);
	*error_sum += i_error;

	// Compute PID output
	int32_t out = (kp * error) + *error_sum + kd * (error - *error_old) * dt_inv;
	*error_old = error;

	out = out/4096/8;
	return out;

}




