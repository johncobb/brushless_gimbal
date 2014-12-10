/*
 * gimbal.h
 *
 *  Created on: Oct 28, 2014
 *      Author: jcobb
 */

#ifndef GIMBAL_H_
#define GIMBAL_H_

#include <stdbool.h>

// Gimbal State
enum gimbal_states {
 GIM_IDLE = 1,      // no PID
 GIM_UNLOCKED = 2,    // PID on, fast ACC
 GIM_LOCKED = 3,      // PID on, slow ACC
 GIM_ERROR = 4        // error condition
};

volatile extern int8_t gimbal_state;

#define LOCK_TIME_SEC 5000   // gimbal fast lock time at startup

#define MOTORUPDATE_FREQ 500                 // in Hz, 1000 is default
#define LOOPUPDATE_FREQ MOTORUPDATE_FREQ     // loop control sample rate equals motor update rate
#define DT_FLOAT (1.0/LOOPUPDATE_FREQ*1.024) // loop controller sample period dT
#define DT_INT_MS (1000/MOTORUPDATE_FREQ)    // dT, integer, (ms)
#define DT_INT_INV (MOTORUPDATE_FREQ)        // dT, integer, inverse, (Hz)
// LP filter coefficient
#define LOWPASS_K_FLOAT(TAU) (DT_FLOAT/(TAU+DT_FLOAT))

// input VCC/Ubat measurement

#define UBAT_ADC_SCALE (5.0 / 1023.0)
// voltage divider
#define UBAT_R1 10000.0
#define UBAT_R2 2200.0
#define UBAT_SCALE ( (UBAT_R1 + UBAT_R2) / UBAT_R2 )




volatile extern bool motor_update; // driven by isr
volatile bool enable_motor_updates; // driven by state machine based on sensor settling

extern int32_t pitch_error_sum;
extern int32_t roll_error_sum;
extern int32_t pitch_error_old;
extern int32_t roll_error_old;

extern float pitch_phi_set;
extern float roll_phi_set;
extern float pitch_angle_set;
extern float roll_angle_set;

extern int pitch_motor_drive;
extern int roll_motor_drive;


// *** FPV (First Person Video) Variables ***
volatile extern bool fpv_mode_pitch;
volatile extern bool fpv_mode_roll;
volatile extern bool fpv_mode_freeze_pitch;
volatile extern bool fpv_mode_freeze_yaw;

void gimbal_init();
void gimbal_tick();
void gimbal_tick2();
void gimbal_tick3();
void gimbal_tick4();




#endif /* GIMBAL_H_ */
