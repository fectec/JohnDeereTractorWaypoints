#include "movement_types.h"
#include "fusion.h"
#include <stdbool.h>
#include <math.h>
#include "main.h"

FusedCoordinate applyPIDCorrection(FusedCoordinate current_coord, 
                                   CorrectionParameters drift_correction, 
                                   float dt,
                                   uint32_t current_time) {
    FusedCoordinate corrected_coord;
    
    // Calculate errors
    float error_x = drift_correction.x;
    float error_y = drift_correction.y;
    float error_angle = drift_correction.angle;
    
    // Accumulate total error and check for reset
    pid_correction.total_error_x += fabsf(error_x);
    pid_correction.total_error_y += fabsf(error_y);
    pid_correction.total_error_angle += fabsf(error_angle);
    
    // Reset accumulated error periodically
    if (current_time - pid_correction.error_accumulation_time > pid_gains.error_reset_interval) {
        // Implement error reset strategy
        if (pid_correction.total_error_x > 10.0f || 
            pid_correction.total_error_y > 10.0f || 
            pid_correction.total_error_angle > 10.0f) {
            // Aggressive reset if error is too high
            pid_correction.integral_x = 0;
            pid_correction.integral_y = 0;
            pid_correction.integral_angle = 0;
            
            pid_correction.total_error_x = 0;
            pid_correction.total_error_y = 0;
            pid_correction.total_error_angle = 0;
        }
        
        pid_correction.error_accumulation_time = current_time;
    }
    
    // Proportional term (reduced impact)
    float p_x = pid_gains.Kp * error_x;
    float p_y = pid_gains.Kp * error_y;
    float p_angle = pid_gains.Kp * error_angle;
    
    // Integral term (with tighter anti-windup)
    pid_correction.integral_x += error_x * dt;
    pid_correction.integral_y += error_y * dt;
    pid_correction.integral_angle += error_angle * dt;
    
    // Limit integral terms more strictly
    const float integral_max = 2.0f;
    pid_correction.integral_x = fmaxf(-integral_max, fminf(integral_max, pid_correction.integral_x));
    pid_correction.integral_y = fmaxf(-integral_max, fminf(integral_max, pid_correction.integral_y));
    pid_correction.integral_angle = fmaxf(-integral_max, fminf(integral_max, pid_correction.integral_angle));
    
    float i_x = pid_gains.Ki * pid_correction.integral_x;
    float i_y = pid_gains.Ki * pid_correction.integral_y;
    float i_angle = pid_gains.Ki * pid_correction.integral_angle;
    
    // Derivative term with error rate limiting
    float d_x = pid_gains.Kd * (error_x - pid_correction.prev_error_x) / dt;
    float d_y = pid_gains.Kd * (error_y - pid_correction.prev_error_y) / dt;
    float d_angle = pid_gains.Kd * (error_angle - pid_correction.prev_error_angle) / dt;
    
    // Limit derivative terms
    const float derivative_max = 1.0f;
    d_x = fmaxf(-derivative_max, fminf(derivative_max, d_x));
    d_y = fmaxf(-derivative_max, fminf(derivative_max, d_y));
    d_angle = fmaxf(-derivative_max, fminf(derivative_max, d_angle));
    
    // Apply PID correction with exponential smoothing
    float new_x = current_coord.x + p_x + i_x + d_x;
    float new_y = current_coord.y + p_y + i_y + d_y;
    float new_angle = current_coord.angle + p_angle + i_angle + d_angle;
    
    // Exponential smoothing
    pid_correction.smoothed_x = 
        pid_gains.smooth_factor * new_x + 
        (1.0f - pid_gains.smooth_factor) * pid_correction.smoothed_x;
    
    pid_correction.smoothed_y = 
        pid_gains.smooth_factor * new_y + 
        (1.0f - pid_gains.smooth_factor) * pid_correction.smoothed_y;
    
    pid_correction.smoothed_angle = 
        pid_gains.smooth_factor * new_angle + 
        (1.0f - pid_gains.smooth_factor) * pid_correction.smoothed_angle;
    
    // Use smoothed values
    corrected_coord.x = pid_correction.smoothed_x;
    corrected_coord.y = pid_correction.smoothed_y;
    corrected_coord.angle = pid_correction.smoothed_angle;
    
    // Update previous errors
    pid_correction.prev_error_x = error_x;
    pid_correction.prev_error_y = error_y;
    pid_correction.prev_error_angle = error_angle;
    
    return corrected_coord;
}

CorrectionParameters calculateDriftCorrection(FusedCoordinate predicted_coord, 
                                              Coordinates camera_coord) {
    CorrectionParameters correction;

    // Calculate the difference between predicted and actual coordinates
    correction.x = camera_coord.x - predicted_coord.x;
    correction.y = camera_coord.y - predicted_coord.y;
    correction.angle = camera_coord.angle - predicted_coord.angle;

    return correction;
}

FusedCoordinate fuseSensorData(Coordinates camera_coords, float encoder_distance, float imu_angle, bool reset_reference) {
    static FusedCoordinate previous_coord = {0}; 
    static uint32_t last_time = 0;
    FusedCoordinate fused_coord;

    // Convert encoder distance to centimeters
    encoder_distance *= 100;

    // Calculate time delta
    uint32_t current_time = HAL_GetTick();
    float dt = (current_time - last_time) / 1000.0f; // Convert to seconds
    last_time = current_time;

    if (reset_reference) {
        // Reset the reference point when new camera data is available
        previous_coord.x = camera_coords.x;
        previous_coord.y = camera_coords.y;
        previous_coord.angle = camera_coords.angle;
        
        // Reset integral terms when reference is reset
        pid_correction.integral_x = 0;
        pid_correction.integral_y = 0;
        pid_correction.integral_angle = 0;
    }

    // Calculate displacement from encoder data
    float dx_encoder = encoder_distance * cos(imu_angle * M_PI / 180.0);
    float dy_encoder = encoder_distance * sin(imu_angle * M_PI / 180.0);

    // Combine camera and encoder measurements
    float camera_weight = 0.50; 
    float encoder_weight = 0.50; 

    // Fuse coordinates using weighted average
    fused_coord.x = camera_coords.x * camera_weight + 
                    (previous_coord.x + dx_encoder) * encoder_weight;
    fused_coord.y = camera_coords.y * camera_weight + 
                    (previous_coord.y + dy_encoder) * encoder_weight;
    fused_coord.angle = camera_coords.angle * camera_weight + 
                        imu_angle * encoder_weight;

    // Calculate drift correction
    CorrectionParameters drift_correction = calculateDriftCorrection(fused_coord, camera_coords);

    // Apply PID-style correction with more conservative thresholds
    const float correction_threshold = 0.5f;  // Increased threshold
    const float max_correction = 2.0f;  // Limit maximum correction
    
    // Clamp drift correction to prevent extreme adjustments
    drift_correction.x = fmaxf(-max_correction, fminf(max_correction, drift_correction.x));
    drift_correction.y = fmaxf(-max_correction, fminf(max_correction, drift_correction.y));
    drift_correction.angle = fmaxf(-max_correction, fminf(max_correction, drift_correction.angle));

    // Only correct if error is significant
    if (fabsf(drift_correction.x) > correction_threshold || 
        fabsf(drift_correction.y) > correction_threshold || 
        fabsf(drift_correction.angle) > correction_threshold) {
        fused_coord = applyPIDCorrection(fused_coord, drift_correction, dt, current_time);
    }

    // Update the reference point for the next iteration
    previous_coord = fused_coord;

    return fused_coord;
}

FusedCoordinate applyDriftCorrection(FusedCoordinate current_coord, 
                                     CorrectionParameters correction, 
                                     float correction_factor) {
    FusedCoordinate corrected_coord;

    // Use an exponential moving average for smoother correction
    corrected_coord.x = current_coord.x + 
        (correction.x * correction_factor * 0.05 + current_coord.x * 0.95);
    
    corrected_coord.y = current_coord.y + 
        (correction.y * correction_factor * 0.05 + current_coord.y * 0.95);
    
    corrected_coord.angle = current_coord.angle + 
        (correction.angle * correction_factor * 0.05 + current_coord.angle * 0.95);

    return corrected_coord;
}

FusedCoordinate predictCoordinate(Coordinates previous_coord, 
                                  float encoder_distance, 
                                  float imu_angle) {
    FusedCoordinate predicted;

    float angle_rad = imu_angle * M_PI / 180.0;

    // Predict new position based on encoder and IMU
    predicted.x = previous_coord.x + encoder_distance * cos(angle_rad);
    predicted.y = previous_coord.y + encoder_distance * sin(angle_rad);
    predicted.angle = imu_angle;

    return predicted;
}