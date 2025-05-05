/*
 * trapezoidal.h
 *
 *  Created on: May 3, 2025
 *      Author: lworakan
 */

#ifndef INC_TRAPEZOIDAL_H_
#define INC_TRAPEZOIDAL_H_



#include "main.h"

// Trapezoidal profile parameters
typedef struct {
    float x0;           // Initial position
    float xf;           // Final position
    float v_max;        // Maximum velocity
    float a_max;        // Maximum acceleration
    float t_accel;      // Acceleration time
    float t_const;      // Constant velocity time
    float t_decel;      // Deceleration time
    float t_total;      // Total trajectory time
    uint8_t profile_type; // 0: triangular, 1: trapezoidal
    float start_time;   // Trajectory start time
    uint8_t is_active;  // Profile active flag
    float direction;    // Movement direction (+1 or -1)
} TrapezoidalProfile;

// Trajectory point data
typedef struct {
    float position;
    float velocity;
    float acceleration;
} TrajectoryPoint;

// Function prototypes
void trapezoidal_init(TrapezoidalProfile* profile);
void trapezoidal_set_limits(TrapezoidalProfile* profile, float v_max, float a_max);
void trapezoidal_plan(TrapezoidalProfile* profile, float x_start, float x_end);
void trapezoidal_start(TrapezoidalProfile* profile);
TrajectoryPoint trapezoidal_get_point(TrapezoidalProfile* profile, float current_time);
uint8_t trapezoidal_is_complete(TrapezoidalProfile* profile, float current_time);
void trapezoidal_scale_to_time(TrapezoidalProfile* profile, float new_time);

#endif // TRAPEZOIDAL_H
