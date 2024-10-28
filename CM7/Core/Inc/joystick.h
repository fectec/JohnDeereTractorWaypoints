/*
 * joystick.h
 *
 * This module provides functions to parse joystick input data from a
 * string format. It handles the extraction and conversion of joystick
 * values, storing them in a global array for further processing.
 */

#ifndef INC_JOYSTICK_H_
#define INC_JOYSTICK_H_

// Includes

#include "main.h"
#include <string.h>
#include <stdlib.h>

// Defines

#define JOYSTICK_INPUTS 6

// Function prototypes

void parseJoystickData(const char* data);

#endif /* INC_JOYSTICK_H_ */
