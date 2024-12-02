#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

typedef struct {
    float x;
    float y;
    float angle;
} State;

typedef struct {
    float P[3][3]; // Covariance matrix
    State state;  // State vector
    float Q[3][3]; // Process noise covariance
    float R[3][3]; // Measurement noise covariance
} KalmanFilter;

void KalmanFilter_Init(KalmanFilter *kf, float x, float y, float angle);
void KalmanFilter_Predict(KalmanFilter *kf, float encoder_distance, float imu_angle);
void KalmanFilter_Update(KalmanFilter *kf, Coordinates camera_coords);

#endif // KALMAN_FILTER_H
