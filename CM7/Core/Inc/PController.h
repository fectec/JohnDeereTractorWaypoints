/*
 * PController.h
 */

#ifndef INC_PCONTROLLER_H_
#define INC_PCONTROLLER_H_

#include "nRF24.h"

#include <stdint.h>
#include <stdbool.h>

// Defines

#define DEFAULT_KP_STEERING 0.3f
#define DEFAULT_KP_SPEED 2.0f
#define DEFAULT_WAYPOINT_TOLERANCE 5

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
} DebugLevel;

// Functions protoypes

void PControllers_Init(void);
void PControllers_SetWaypoint(uint16_t x, uint16_t y, Coordinates initial_pos);
bool OutPController_Update(Coordinates current_pos);
void OutPController_Debug(DebugLevel level);

#endif /* INC_PCONTROLLER_H_ */
