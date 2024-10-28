/*
 * servo.c
 */

#include "main.h"
#include "servo.h"

/**
 * @brief Sets the servo angle using PWM.
 *
 * This function adjusts the PWM pulse width to set the servo position
 * based on the specified angle, which should be within the range of
 * -90.0 to 90.0 degrees. The base pulse width is 1.5 ms, and the pulse
 * width is adjusted proportionally to the angle. If the angle is out
 * of range, the function exits without making changes.
 *
 * @param angle The desired servo angle in degrees (-90.0 to 90.0).
 */

void Turning_SetAngle(float angle)
{
	// Check if the angle is within the valid range

	if (angle < SERVO_MIN_ANGLE || angle > SERVO_MAX_ANGLE) {
		return;
	}

	// Calculate the pulse width corresponding to the angle
	// Base pulse width is SERVO_BASE_PULSE_WIDTH_MS, adjusted based on the angle

	float pulseWidth = SERVO_BASE_PULSE_WIDTH_MS + (angle / 180.0f);

	// Get the current value of the auto-reload register for the timer

	uint32_t autoreload_register_value = __HAL_TIM_GET_AUTORELOAD(&htim13);

	// Calculate the compare value based on the pulse width and timer settings

	uint32_t value = (uint32_t)((pulseWidth / SERVO_PWM_PERIOD_MS) * (autoreload_register_value + 1));

	// Set the PWM duty cycle for TIM_CHANNEL_1 of the timer

	__HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, value);

    // Start the PWM signal generation on TIM_CHANNEL_1 if not already started

    HAL_TIM_StateTypeDef pwmState = HAL_TIM_PWM_GetState(&htim13);

    if (pwmState != HAL_TIM_STATE_READY) {
        HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1);
    }
}
