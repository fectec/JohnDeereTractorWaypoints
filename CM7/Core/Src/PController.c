/*
 * PController.c
 */

#include "PController.h"
#include "motor.h"
#include "servo.h"

#include <math.h>

// Static variables to maintain controller state

static PControllerParams controller_params;
static float current_steering_angle;
static float current_motor_speed;
static bool waypoint_reached;

// Debug-related static variables

static Coordinates last_position;
static float last_distance;
static float last_heading_error;

void PController_Init(void)
{
    // Set default P controller gains

    controller_params.Kp_steering = DEFAULT_KP_STEERING;
    controller_params.Kp_speed = DEFAULT_KP_SPEED;

    // Set default waypoint tolerance

    controller_params.waypoint_tolerance = DEFAULT_WAYPOINT_TOLERANCE;

    // Initialize waypoint to (0,0)

    controller_params.waypoint.x = 0;
    controller_params.waypoint.y = 0;

    // Reset control outputs

    current_steering_angle = 0.0f;
    current_motor_speed = 0.0f;
    waypoint_reached = false;
}

void PController_SetWaypoint(uint16_t x, uint16_t y)
{
    controller_params.waypoint.x = x;
    controller_params.waypoint.y = y;
    waypoint_reached = false;
}

bool PController_Update(Coordinates current_pos)
{
    // Store last position for debugging

    last_position = current_pos;

    // Calculate the vector to the waypoint

    int16_t dx = (int16_t)controller_params.waypoint.x - (int16_t)current_pos.x;
    int16_t dy = (int16_t)controller_params.waypoint.y - (int16_t)current_pos.y;

    // Calculate distance to waypoint

    last_distance = sqrtf(dx * dx + dy * dy);

    // Check if waypoint is reached

    if (last_distance <= controller_params.waypoint_tolerance)
    {
        waypoint_reached = true;

        current_motor_speed = 0.0f;
        current_steering_angle = 0.0f;

        Turning_SetAngle(current_steering_angle);
        SetMotorSpeed(current_motor_speed);

        return true;
    }

    // Calculate desired heading (target angle)

    float desired_heading = atan2f(dy, dx) * 180.0f / M_PI;

    // Use the current position's angle from NRF24

    float current_heading = (int16_t)current_pos.angle;

    // ** P controller for steering **

    // Calculate heading error, taking into account angle wrap-around

    last_heading_error = desired_heading - current_heading;

    // Normalize heading error to [-180, 180]

    if (last_heading_error > 180.0f) last_heading_error -= 360.0f;
    if (last_heading_error < -180.0f) last_heading_error += 360.0f;

    // Limit steering angle between -90 and 90

    current_steering_angle = controller_params.Kp_steering * last_heading_error;

    if (current_steering_angle > 90.0f) current_steering_angle = 90.0f;
    if (current_steering_angle < -90.0f) current_steering_angle = -90.0f;

    // ** P controller for speed **

    // Speed proportional to distance, scaled to 0-1

    current_motor_speed = controller_params.Kp_speed * (last_distance / 1000.0f);

    // Limit motor speed between 0 and 1

    if (current_motor_speed > 1.0f) current_motor_speed = 1.0f;
    if (current_motor_speed < 0.0f) current_motor_speed = 0.0f;

    // Apply controls

    Turning_SetAngle(current_steering_angle);
    SetMotorSpeed(current_motor_speed);

    return false;
}

void PController_Debug(DebugLevel level)
{
    switch(level) {

        case DEBUG_MINIMAL:
            printf("CONTROLLER STATUS: %s\r\n",
                   waypoint_reached ? "WAYPOINT REACHED" : "NAVIGATING");
            break;

        case DEBUG_NORMAL:

            printf("--- CONTROLLER DEBUG ---\r\n");
            printf("Current Position: (x: %u, y: %u, angle: %u)\r\n",
                   last_position.x, last_position.y, last_position.angle);
            printf("Waypoint: (x: %u, y: %u)\r\n",
                   controller_params.waypoint.x, controller_params.waypoint.y);
            printf("Distance to Waypoint: %.2f\r\n", last_distance);
            printf("Applied Signals:\r\n");
            printf("  Steering Angle: %.2f degrees\r\n", current_steering_angle);
            printf("  Motor Speed: %.2f\r\n", current_motor_speed);
            break;

        case DEBUG_VERBOSE:
            printf("--- DETAILED CONTROLLER DEBUG ---\r\n");
            printf("Controller Parameters:\r\n");
            printf("  Kp Steering: %.2f\r\n", controller_params.Kp_steering);
            printf("  Kp Speed: %.2f\r\n", controller_params.Kp_speed);
            printf("  Waypoint Tolerance: %u\r\n", controller_params.waypoint_tolerance);
            printf("\r\n");

            printf("Current State:\r\n");
            printf("  Current Position: (x: %u, y: %u, angle: %u)\r\n",
                   last_position.x, last_position.y, last_position.angle);
            printf("  Waypoint: (x: %u, y: %u)\r\n",
                   controller_params.waypoint.x, controller_params.waypoint.y);
            printf("  Distance to Waypoint: %.2f\r\n", last_distance);
            printf("  Heading Error: %.2f degrees\r\n", last_heading_error);
            printf("\r\n");

            printf("Applied Control Signals:\r\n");
            printf("  Steering Angle: %.2f degrees\r\n", current_steering_angle);
            printf("  Motor Speed: %.2f\r\n", current_motor_speed);
            printf("  Waypoint Reached: %s\r\n",
                   waypoint_reached ? "YES" : "NO");
            break;
    }
}
