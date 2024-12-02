/*
 * kalman_imu.c
 *
 *  This file contains functions to implement a simple Kalman Filter.
 *  The Kalman Filter is used to estimate the state of a system from noisy measurements.
 *  It is commonly used in control systems, robotics, and sensor fusion applications.
 *
 *  Functions:
 *  - Kalman_Init: Initializes the Kalman Filter with process noise, measurement noise,
 *                and the initial estimate value.
 *  - Kalman_Update: Updates the Kalman Filter with a new measurement and returns the
 *                   filtered estimate.
 */

#include "main.h"
#include "imu.h"
#include "kalman_imu.h"

/**
 * @brief Initializes the Kalman Filter.
 *
 * This function sets the initial values for the Kalman Filter's process noise (q),
 * measurement noise (r), initial state estimate (x), and initial estimation error (p).
 * The Kalman gain (k) is also initialized to zero.
 *
 * @param kf Pointer to the KalmanFilter structure.
 * @param processNoise The process noise covariance (q) - defines the uncertainty in the process model.
 * @param measurementNoise The measurement noise covariance (r) - defines the uncertainty in the measurements.
 * @param initialValue The initial estimate of the state.
 */

void Kalman_Init(KalmanFilter *kf, float processNoise, float measurementNoise, float initialValue)
{
    kf->q = processNoise;
    kf->r = measurementNoise;
    kf->x = initialValue;
    kf->p = 1.0f; 				// Initial estimation error
    kf->k = 0.0f; 				// Initial Kalman gain
}

/**
 * @brief Updates the Kalman Filter with a new measurement.
 *
 * This function performs the prediction and measurement update steps of the Kalman Filter.
 * It calculates the new estimate of the state based on the previous estimate and the new measurement.
 * The filter also updates the Kalman gain and estimation error covariance.
 *
 * @param kf Pointer to the KalmanFilter structure.
 * @param measurement The new measurement (z) used to update the state estimate.
 *
 * @return The updated state estimate (x).
 */

float Kalman_Update(KalmanFilter *kf, float measurement)
{
    // Prediction update

    kf->p = kf->p + kf->q;

    // Measurement update

    kf->k = kf->p / (kf->p + kf->r);           		// Calculate Kalman gain
    kf->x = kf->x + kf->k * (measurement - kf->x); 	// Update value with measurement
    kf->p = (1.0f - kf->k) * kf->p;           		// Update estimation error covariance

    return kf->x;
}
