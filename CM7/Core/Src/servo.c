/*
 * servo.c
 */

#include "main.h"
#include "servo.h"

void Turning_SetAngle(float Angle)
{
	// Check if the angle is within the valid range

	if (Angle < -90.0f || Angle > 90.0f)
	{
		return;
	}

	// Calculate the pulse width corresponding to the angle
	// Base pulse width is 1.5 ms, and we adjust it based on the angle

	float pulseWidth = 1.5f + (Angle / 180.0f);

	// Get the current value of the auto-reload register for the timer

	uint32_t autoreload_register_value = __HAL_TIM_GET_AUTORELOAD(&htim13);

	// Calculate the compare value based on the pulse width and timer settings

	uint32_t value = (uint32_t)((pulseWidth / 20.0f) * (autoreload_register_value + 1));

	// Set the PWM duty cycle for TIM_CHANNEL_1 of the timer

	__HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, value);

	// Start the PWM signal generation on TIM_CHANNEL_1

	HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1);
}
