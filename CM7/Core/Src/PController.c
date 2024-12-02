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

// Debugging-related variables

static Coordinates last_position;
static float last_distance;
static float last_heading_error;

// IMU / Encoder-based control variables

static float initial_distance;
static float traveled_distance;
static float initial_heading;

void PControllers_Init(void)
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

    // Reset distances and heading

    initial_distance = 0.0f;
    traveled_distance = 0.0f;
    initial_heading = 0.0f;
}

void PControllers_SetWaypoint(uint16_t x, uint16_t y, Coordinates initial_pos)
{
    // Set the new waypoint

    controller_params.waypoint.x = x;
    controller_params.waypoint.y = y;

    // Calculate the initial theoretical distance to the waypoint

    int16_t dx = (int16_t)(controller_params.waypoint.x - initial_pos.x);
    int16_t dy = (int16_t)(controller_params.waypoint.y - initial_pos.y);
    initial_distance = sqrtf(dx * dx + dy * dy);

    // Calculate the initial desired heading (angle) to the waypoint

    initial_heading = atan2f(dy, dx) * 180.0f / M_PI;

    // Reset traveled distance and waypoint status

    traveled_distance = 0.0f;
    waypoint_reached = false;
}

bool OutPController_Update(Coordinates current_pos)
{
	// Store last position for debugging

	last_position = current_pos;

	// Calculate the vector to the waypoint

	int16_t dx = (int16_t)(controller_params.waypoint.x - current_pos.x);
	int16_t dy = (int16_t)(controller_params.waypoint.y - current_pos.y);

	// Calculate distance to waypoint

	float last_distance = sqrtf(dx * dx + dy * dy);

	// Check if waypoint is reached

	if (last_distance <= controller_params.waypoint_tolerance)
	{
		waypoint_reached = true;
		current_motor_speed = 0.0f;
		current_steering_angle = 0.0f;
		Turning_SetAngle(current_steering_angle);
		SetMotorSpeed(current_motor_speed);
		return true; // Waypoint reached
	}

	// Calculate desired heading (target angle) using GPS

	float desired_heading = atan2f(dy, dx) * 180.0f / M_PI;

	// Get the current heading (angle) from the GPS

	float current_heading = (int16_t)current_pos.angle;

	// ** P controller for steering **

	last_heading_error = desired_heading - current_heading;

	if (last_heading_error > 180.0f) last_heading_error -= 360.0f;
	if (last_heading_error < -180.0f) last_heading_error += 360.0f;

	// Limit steering angle between -90 and 90

	current_steering_angle = controller_params.Kp_steering * last_heading_error;

	if (current_steering_angle > 90.0f) current_steering_angle = 90.0f;
	if (current_steering_angle < -90.0f) current_steering_angle = -90.0f;

    // ** P controller for speed: map motor speed between 0.9 and 1.0 **

    current_motor_speed = 0.9f + (controller_params.Kp_speed * (last_distance / 100.0f)) * 0.1f;

    // Ensure speed is between 0.9 and 1.0

    if (current_motor_speed > 1.0f) current_motor_speed = 1.0f;
    if (current_motor_speed < 0.9f) current_motor_speed = 0.9f;

	// Apply control signals

	Turning_SetAngle(current_steering_angle);

	for(float i = 0.0f; i < current_motor_speed; i += 0.1f)
	{
		SetMotorSpeed(i);
		HAL_Delay(5);
	}

	HAL_Delay(500);
	SetMotorSpeed(0.0f);
	HAL_Delay(100);

	return false; // Waypoint not yet reached
}

/*
bool InPController_Update(Coordinates current_pos)
{
    // Store last position for debugging

    last_position = current_pos;

    // Track the distance traveled using the encoder (you'll need to implement GetEncoderDistance)
    traveled_distance += GetEncoderDistance();  // Get distance from encoder (in meters or units)

    // Calculate the remaining distance to the waypoint
    float remaining_distance = initial_distance - traveled_distance;

    // ** P-controller for Speed **
    current_motor_speed = controller_params.Kp_speed * (remaining_distance / initial_distance);
    if (current_motor_speed > 1.0f) current_motor_speed = 1.0f;
    if (current_motor_speed < 0.0f) current_motor_speed = 0.0f;

    // ** Angle Control (via IMU) **
    float current_heading = current_pos.angle;  // Get the current heading of the robot from IMU

    // ** P-controller for Steering (Angle) **
    float heading_error = initial_heading - current_heading;

    // Normalize heading error to [-180, 180]
    if (heading_error > 180.0f) heading_error -= 360.0f;
    if (heading_error < -180.0f) heading_error += 360.0f;

    // Apply proportional control for steering
    current_steering_angle = controller_params.Kp_steering * heading_error;

    // Limit steering angle between -90 and 90 degrees
    if (current_steering_angle > 90.0f) current_steering_angle = 90.0f;
    if (current_steering_angle < -90.0f) current_steering_angle = -90.0f;

    // Apply control signals to the motor and steering
    SetMotorSpeed(current_motor_speed);
    Turning_SetAngle(current_steering_angle);

    // Check if Waypoint is reached
    if (remaining_distance <= controller_params.waypoint_tolerance)
    {
        waypoint_reached = true;
        SetMotorSpeed(0.0f);  // Stop the motor when the waypoint is reached
        Turning_SetAngle(0.0f);  // Stop turning the servo
        return true;  // Waypoint reached
    }

    return false; // Waypoint not yet reached
}
*/

void OutPController_Debug(DebugLevel level)
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
            printf("Distance to Waypoint: %f\r\n", last_distance);
            printf("Applied Signals:\r\n");
            printf("  Steering Angle: %f degrees\r\n", current_steering_angle);
            printf("  Motor Speed: %f\r\n", current_motor_speed);
            break;
    }
}
