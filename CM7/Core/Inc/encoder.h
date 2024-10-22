/*
 * encoder.h
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

extern volatile int encoderPos;
extern volatile int lastEncoded;

// Function prototypes

void HAL_GPIO_EXTI_Callback(uint16_t);

#endif /* INC_ENCODER_H_ */
