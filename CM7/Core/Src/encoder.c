/*
 * encoder.c
 */

#include "main.h"
#include "encoder.h"

// Initialize the global encoder instance

Encoder encoder = {0, 0};

// Initialization function for the encoder

void Encoder_Init(void) {
	encoder.pos = 0;
	encoder.lastEncoded = 0;
}

/**
 * @brief  External Interrupt Callback for Encoder Input
 *
 * This function is called when an external interrupt occurs on the
 * specified GPIO pin. It processes the encoder's state by reading the
 * current states of the encoder pins and updating the encoder position
 * accordingly.
 *
 * @param  GPIO_Pin: The GPIO pin number that triggered the interrupt.
 *                   This should correspond to the defined encoder pin,
 *                   typically ENCODER_A_Pin.
 *
 * @note   This function is triggered by the EXTI (External Interrupt)
 *         mechanism, specifically configured for the encoder input.
 *         It is important to ensure that this function executes quickly
 *         to avoid delays in handling other interrupts.
 *
 * @retval None
 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	if (GPIO_Pin == ENCODER_A_Pin) {														// Triggered by the interrupt on ENCODER_A_Pin

		int currentEncodedA = HAL_GPIO_ReadPin(ENCODER_A_GPIO_Port, ENCODER_A_Pin);
		int currentEncodedB = HAL_GPIO_ReadPin(ENCODER_B_GPIO_Port, ENCODER_B_Pin);

		// Handle rising to falling transition

		if (encoder.lastEncoded == ENCODER_A_HIGH && currentEncodedA == ENCODER_A_LOW) {
			encoder.pos += (currentEncodedB == ENCODER_A_HIGH) ? 1 : -1;						// Determine direction
		}

		encoder.lastEncoded = currentEncodedA; 												// Update last state
	}
}
