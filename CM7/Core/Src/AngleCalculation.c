/*
 * AngleCalculation.c
 */

#include "main.h"
#include "imu.h"
#include "AngleCalculation.h"
#include "printf.h"

#include <math.h>

float currentAngle = 0.0f;
float previousGyroZ = 0.0f;
uint32_t previousTimestamp = 0;

float updateRotationAngle(SensorData sensorData, uint32_t currentTimestamp)
{
    // Calculate time delta

    float deltaTime = (currentTimestamp - previousTimestamp) / 1000.0f;

    // Use gyroscope Z-axis for rotation (yaw rate)

    float gyroZRate = sensorData.gz;

    // Integrate gyroscope readings to get angle
    // Use trapezoidal integration for more accuracy

    float angleChange = 0.5f * (previousGyroZ + gyroZRate) * deltaTime;

    // Update cumulative angle

    currentAngle += angleChange;

    // Normalize angle to 0-360 range

    if (currentAngle < 0)
    {
        currentAngle += 360.0f;
    }
    else if (currentAngle >= 360.0f)
    {
        currentAngle -= 360.0f;
    }

    // Store current values for next iteration

    previousGyroZ = gyroZRate;
    previousTimestamp = currentTimestamp;

    // printf("Angle: %f\n\r", currentAngle);

    return currentAngle;
}
