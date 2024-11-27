/*
 * kalman.c
 *
 *  Created on: Nov 26, 2024
 *      Author: fecte
 */


#include "kalman.h"

void Kalman_Init(KalmanFilter *kf, float processNoise, float measurementNoise, float initialValue)
{
    kf->q = processNoise;
    kf->r = measurementNoise;
    kf->x = initialValue;
    kf->p = 1.0f; 				// Initial estimation error
    kf->k = 0.0f; 				// Initial Kalman gain
}

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
