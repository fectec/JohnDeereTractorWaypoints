/*
 * ESC.h
 *
 * This module provides functions to control the motor speed via PWM
 * using Timer 14. The speed can be set between -1.0 (full reverse)
 * and 1.0 (full forward). It includes handling for PWM duty cycle
 * calculations and manages the initialization of the PWM signal.
 */

#ifndef INC_ESC_H_
#define INC_ESC_H_

// Includes

#include "main.h"

// Defines

#define PWM_PERIOD_MS				20.0f
#define NEUTRAL_PULSE_WIDTH_MS		1.5f
#define FORWARD_PULSE_WIDTH_RANGE	0.5f
#define REVERSE_PULSE_WIDTH_RANGE	0.5f

// Extern variables

extern TIM_HandleTypeDef htim14;	// Timer handler for PWM control

// Function prototypes

void SetMotorSpeed(float speed);

#endif /* INC_ESC_H_ */
