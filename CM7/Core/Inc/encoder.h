/*
 * encoder.h
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

// Defines

#define WHEEL_RADIUS 0.034                               // Meters
#define ENCODER_CPR  210
#define WHEEL_CIRCUMFERENCE (2 * 3.14159 * WHEEL_RADIUS) // Meters

// Function prototypes

float EncoderGetDistance(uint16_t);

#endif /* INC_ENCODER_H_ */
