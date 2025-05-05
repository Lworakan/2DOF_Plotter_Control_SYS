/*
 * kalman_wrapper.h
 *
 *  Created on: May 3, 2025
 *      Author: lworakan
 */

#ifndef INC_KALMAN_WRAPPER_H_
#define INC_KALMAN_WRAPPER_H_



#include "kalman.h"

typedef struct {
    KalmanFilter filter;
    float A_matrix[16];  // State transition matrix
    float B_matrix[4];   // Control input matrix
    float Q;             // Process noise
    float R;             // Measurement noise

    float current_velocity;  // Kalman filtered velocity
    float current_position;  // Integrated position

    float last_velocity_raw; // Last raw velocity measurement
    uint32_t last_update_time;
} KalmanVelocityEstimator;

// Function prototypes
void kalman_velocity_estimator_init(KalmanVelocityEstimator* estimator, float Q, float R);
void kalman_velocity_estimator_update(KalmanVelocityEstimator* estimator,
                                     float raw_velocity, float control_input,
                                     float dt);
float kalman_velocity_estimator_get_velocity(KalmanVelocityEstimator* estimator);
float kalman_velocity_estimator_get_position(KalmanVelocityEstimator* estimator);

#endif // KALMAN_WRAPPER_H
