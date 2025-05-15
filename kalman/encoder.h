/*
 * encoder.h
 *
 *  Created on: May 3, 2025
 *      Author: lworakan
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_



#include "main.h"
#include "kalman_wrapper.h"

// Encoder configuration constants
#define REVOLUTE_ENCODER_CPR 4096
#define REVOLUTE_GEAR_RATIO 1.0f
#define PRISMATIC_ENCODER_CPR 4096
#define PRISMATIC_MM_PER_REV 10.0f
#define PRISMATIC_GEAR_RATIO 1.0f

typedef struct {
    TIM_HandleTypeDef* htim;
    int32_t count;                     // Raw encoder count
    int32_t last_count;                // Previous raw count for difference calculation
    int16_t speed;                     // Speed in encoder ticks
    float position;                    // Position in user units (degrees, mm, etc.)
    float last_position;               // Previous position for direct speed calculation
    float speed_real;                  // Speed in user units per second
    float cpr;                         // Counts per revolution
    float gear_ratio;                  // Gear ratio
    float conversion_factor;           // Conversion factor to user units
    uint32_t last_speed_calc_time;     // Last time speed was calculated
    int32_t last_count_for_speed;      // Last count used for speed calculation
    KalmanVelocityEstimator kalman_vel; // Kalman filter for velocity estimation
    float kalman_velocity;             // Kalman-filtered velocity
    float kalman_position;             // Kalman-filtered position
} EncoderData;


// Function prototypes
void encoder_init(EncoderData* enc, TIM_HandleTypeDef* htim, float cpr,
                 float gear_ratio, float conversion_factor);
void encoder_read(EncoderData* enc);
void encoder_calculate_speed(EncoderData* enc);
void encoder_reset(EncoderData* enc);

#endif // ENCODER_H
