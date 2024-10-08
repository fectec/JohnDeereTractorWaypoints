/*
 * joystick.c
 */

#include "main.h"
#include "joystick.h"

int joystick_data[JOYSTICK_INPUTS];

/**
 * @brief Parses joystick input data from a comma-separated string.
 *
 * This function takes a string of joystick data, tokenizes it by commas,
 * converts each token to an integer, and stores the values in the
 * joystick_data array. It stops processing either when there are no more
 * tokens or when the maximum number of inputs (JOYSTICK_INPUTS) is reached.
 *
 * @param data A comma-separated string containing joystick input values.
 */

void parseJoystickData(const char* data)
{
	// Declare a pointer for tokenization and create a
	// modifiable copy of the input data

	char* token;
	char* dataCopy = strdup(data);

	// Initialize an index to track the position in joystick_data

	int index = 0;

	// Use strtok to split the input string by commas

	token = strtok(dataCopy, ",");

	// Loop through the tokens until there are no more or
	// the index exceeds JOYSTICK_INPUTS

	while (token != NULL && index < JOYSTICK_INPUTS)
	{
		// Convert the token to an integer and store it in
		// the joystick_data array

		joystick_data[index++] = atoi(token);

		// Get the next token

		token = strtok(NULL, ",");
	}

	// Free the memory allocated for the duplicated string
	// to prevent memory leaks

	free(dataCopy);
}
