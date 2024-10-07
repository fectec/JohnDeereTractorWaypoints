/*
 * ESC.c
 */

#include <main.h>
#include <ESC.h>

void SetMotorSpeed(float speed)
{

    // Ensure speed is within the valid range

	if (speed < -1.0f || speed > 1.0f)
    {
        return;
    }

	// Calculate the pulse width based on the input speed

	float pulseWidth;

    if (speed > 0.0f)		// Forward
    {
        pulseWidth = 1.5f + (speed * 0.5f); // Map speed to range (1.5ms to 2ms)
    }
    else if (speed < 0.0f)	// Reverse
    {
        pulseWidth = 1.5f + (speed * 0.5f); // Map speed to range (1ms to 1.5ms)
    }
    else					// Neutral
    {
        pulseWidth = 1.5f;
    }

    // Get the current value of the auto-reload register for the timer

    uint32_t autoreload_register_value = __HAL_TIM_GET_AUTORELOAD(&htim14);

    // Calculate the compare value based on the pulse width and timer settings

    uint32_t value = (uint32_t)((pulseWidth / 20.0f) * (autoreload_register_value + 1));

    // Set the PWM duty cycle for TIM_CHANNEL_1 of the timer

    __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, value);

    // Start the PWM signal generation on TIM_CHANNEL_1

    HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);
}
