/*
 * gyro.h
 *
 *  Created on: Oct 24, 2014
 *      Author: jcobb
 */

#ifndef GYRO_H_
#define GYRO_H_
#include <stdint.h>

//#include "../util/defines.h"

#define GYRO_DEBUG		1

/* Set the Low Pass Filter factor for ACC Magnitude */
#define ACC_LPF_FACTOR 40

#define ACC_1G 16384.0f

typedef struct pid_data {
  int32_t   kp, ki, kd;
} pid_data_t;

extern pid_data_t pitch_pid_par;
extern pid_data_t roll_pid_par;

//#define GRAVITY 16384.0f
#define GRAVITY	15500.0f;

void init_resolution_divider();
void gyro_offset_calibration();
uint8_t accl_calibration();
void init_sensor_orientation();
void init_pids();
void read_accs();
void update_acc();

#endif /* GYRO_H_ */
