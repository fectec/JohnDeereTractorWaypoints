/*
 * control.h
 */

#ifndef INC_CONTROL_H_
#define INC_CONTROL_H_

#include "main.h"

// Define the maximum speed and steering angle limits

#define MAX_SPEED 1.0  // Maximum speed is 1
#define MAX_ANGLE 90.0 // Maximum steering angle is 90 degrees

// Structure to store a waypoint (coordinate)
typedef struct {
    float x;
    float y;
} Waypoint;

// Function declarations
void control_Init(void);
void control_SpeedAndSteering(Waypoint target, Coordinates current_position);
void update_SpeedAndAngle(float speed, float angle);

#endif /* INC_CONTROL_H_ */
