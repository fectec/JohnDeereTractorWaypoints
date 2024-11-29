/*
 * encoder.h
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

// Define constants

#define WHEEL_RADIUS 0.034                                  // Wheel radius in meters
#define ENCODER_CPR 210                                     // Encoder counts per revolution (CPR)
#define WHEEL_CIRCUMFERENCE (2 * 3.14159 * WHEEL_RADIUS)    // in meters

// Functions prototypes

float EncoderGetDistance(uint16_t); // Get distance traveled based on counts

#endif /* INC_ENCODER_H_ */
