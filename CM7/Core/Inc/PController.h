/*
 * PController.h
 */

#ifndef INC_PCONTROLLER_H_
#define INC_PCONTROLLER_H_

#include "nRF24.h"

#include <stdint.h>
#include <stdbool.h>

// Defines

#define DEFAULT_KP_STEERING 0.6f
#define DEFAULT_KP_SPEED 3.0f
#define DEFAULT_WAYPOINT_TOLERANCE 50

// TypeDefs

typedef struct {

    // Proportional gains for steering and speed

    float Kp_steering;
    float Kp_speed;

    // Waypoint coordinates

    struct {
        uint16_t x;
        uint16_t y;
    } waypoint;

    // Acceptable proximity to waypoint (in units)

    uint16_t waypoint_tolerance;
} PControllerParams;

// Enum for debug output levels

typedef enum {
    DEBUG_MINIMAL,   // Only key information
    DEBUG_NORMAL,    // Detailed controller state
    DEBUG_VERBOSE    // Extremely detailed information
} DebugLevel;

// Functions protoypes

void PController_Init(void);
void PController_SetWaypoint(uint16_t x, uint16_t y);
bool PController_Update(Coordinates current_pos);
void PController_Debug(DebugLevel level);

#endif /* INC_PCONTROLLER_H_ */
