/*
 * servo.h
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_

extern TIM_HandleTypeDef htim13;

// Function prototypes

void Turning_SetAngle(float);	// Sets the servo angle for turning based
								// on the specified angle from -90 to 90
								// degrees

#endif /* INC_SERVO_H_ */
