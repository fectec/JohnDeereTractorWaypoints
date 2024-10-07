/*
 * joystick.h
 */

#ifndef INC_JOYSTICK_H_
#define INC_JOYSTICK_H_

// Defines

#define JOYSTICK_INPUTS 6

// Function prototypes

void parseJoystickData(const char*);	// Parses a comma-separated string of joystick data
										// and stores the integer values in the global
										// joystick_data array

#endif /* INC_JOYSTICK_H_ */
