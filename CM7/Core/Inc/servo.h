/*
 * servo.h
 *
 * This module provides functions to control a servo motor using PWM.
 * It allows setting the servo angle within a specified range,
 * translating the angle into corresponding PWM pulse widths.
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_

// Include

#include "main.h"

// Defines

#define SERVO_MIN_ANGLE				-90.0f
#define SERVO_MAX_ANGLE				90.0f
#define SERVO_BASE_PULSE_WIDTH_MS	1.5f
#define SERVO_PWM_PERIOD_MS			20.0f


// Extern variables

extern TIM_HandleTypeDef htim13;	// Timer handler for PWM control

// Function prototypes

void Turning_SetAngle(float angle);

#endif /* INC_SERVO_H_ */
