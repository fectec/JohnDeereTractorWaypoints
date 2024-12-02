#ifndef FUSION_H_
#define FUSION_H_

#include "movement_types.h"
#include <stdbool.h>
#include <math.h>
#include "main.h"

// PID Correction Structure
typedef struct {
    float integral_x;
    float integral_y;
    float integral_angle;
    float prev_error_x;
    float prev_error_y;
    float prev_error_angle;
    
    // New smoothing variables
    float smoothed_x;
    float smoothed_y;
    float smoothed_angle;
    
    // Error tracking
    float total_error_x;
    float total_error_y;
    float total_error_angle;
    uint32_t error_accumulation_time;
} PIDCorrection;

// PID Gains (these will need tuning)
typedef struct {
    float Kp; // Proportional gain
    float Ki; // Integral gain
    float Kd; // Derivative gain
    float smooth_factor; // Smoothing factor
    uint32_t error_reset_interval; // Time to reset accumulated error
} PIDGains;

// Global PID correction and gains
static PIDCorrection pid_correction = {0};
static PIDGains pid_gains = {
    .Kp = 0.2f,     // Reduced proportional gain
    .Ki = 0.1f,     // Reduced integral gain
    .Kd = 0.05f,    // Reduced derivative gain
    .smooth_factor = 0.2f, // Smoothing factor
    .error_reset_interval = 500 // Reset error every 5 seconds
}; 

FusedCoordinate applyPIDCorrection(FusedCoordinate current_coord, 
                                   CorrectionParameters drift_correction, 
                                   float dt,
                                   uint32_t current_time);

CorrectionParameters calculateDriftCorrection(FusedCoordinate predicted_coord, 
                                              Coordinates camera_coord);

FusedCoordinate fuseSensorData(Coordinates camera_coords, float encoder_distance, float imu_angle, bool reset_reference);

FusedCoordinate applyDriftCorrection(FusedCoordinate current_coord, 
                                     CorrectionParameters correction, 
                                     float correction_factor);

FusedCoordinate predictCoordinate(Coordinates previous_coord, 
                                  float encoder_distance, 
                                  float imu_angle);

#endif