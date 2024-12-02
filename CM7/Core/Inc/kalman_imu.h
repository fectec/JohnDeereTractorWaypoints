/*
 * kalman_imu.h
 */

#ifndef INC_KALMAN_IMU_H_
#define INC_KALMAN_IMU_H_

// Kalman filter structure

typedef struct {
    float q;      // Process noise covariance
    float r;      // Measurement noise covariance
    float x;      // Value
    float p;      // Estimation error covariance
    float k;      // Kalman gain
} KalmanFilter;

// Function prototypes

void Kalman_Init(KalmanFilter *kf, float processNoise, float measurementNoise, float initialValue);
float Kalman_Update(KalmanFilter *kf, float measurement);

#endif /* INC_KALMAN_IMU_H_ */
