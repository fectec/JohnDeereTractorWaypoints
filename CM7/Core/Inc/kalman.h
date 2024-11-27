/*
 * kalman.h
 */

#ifndef INC_KALMAN_H_
#define INC_KALMAN_H_

#include "main.h"
#include "imu.h"

// Kalman filter structure

typedef struct {
    float q;      // Process noise covariance
    float r;      // Measurement noise covariance
    float x;      // Value
    float p;      // Estimation error covariance
    float k;      // Kalman gain
} KalmanFilter;

// Initialize Kalman filter

void Kalman_Init(KalmanFilter *kf, float processNoise, float measurementNoise, float initialValue);

// Apply Kalman filter to a single value

float Kalman_Update(KalmanFilter *kf, float measurement);

#endif /* INC_KALMAN_H_ */
