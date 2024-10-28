/*
 * encoder.h
 *
 * This module provides functions and variables to manage rotary encoder
 * inputs. It tracks the current position of the encoder and updates
 * the position based on external interrupt signals. The encoder position
 * can be used for motor control or other applications requiring
 * rotational feedback.
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

// Includes

#include "main.h"

// Defines

// Encoder states

#define ENCODER_A_HIGH 1
#define ENCODER_A_LOW 0

// Extern variables

typedef struct {
	volatile int pos;
	volatile int lastEncoded;
} Encoder;

extern Encoder encoder;

// Function prototypes

void Encoder_Init(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

#endif /* INC_ENCODER_H_ */
