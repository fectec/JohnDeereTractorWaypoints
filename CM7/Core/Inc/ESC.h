/*
 * ESC.h
 */

#ifndef INC_ESC_H_
#define INC_ESC_H_

extern TIM_HandleTypeDef htim14;

// Function prototypes

void SetMotorSpeed(float);	// Function to set motor speed based on
							// input scale from -1.0 to 1.0

#endif /* INC_ESC_H_ */
