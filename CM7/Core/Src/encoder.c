/*
 * encoder.c
 */

#include "main.h"
#include "encoder.h"

volatile int encoderPos = 0;
volatile int lastEncoded = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	if (GPIO_Pin == ENCODER_A_Pin)							// Triggered by the interrupt on PA0
	{
		int currentEncodedA = HAL_GPIO_ReadPin(ENCODER_A_GPIO_Port, ENCODER_A_Pin);
		int currentEncodedB = HAL_GPIO_ReadPin(ENCODER_B_GPIO_Port, ENCODER_B_Pin);

		if (lastEncoded == 1 && currentEncodedA == 0)		// Rising to falling transition
		{
			encoderPos += (currentEncodedB == 1) ? 1 : -1;	// Determine direction
		}

		lastEncoded = currentEncodedA; 						// Update last state
	}
}
