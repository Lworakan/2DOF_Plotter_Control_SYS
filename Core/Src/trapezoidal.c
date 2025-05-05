/*
 * trapezoidal.c
 *
 *  Created on: May 3, 2025
 *      Author: lworakan
 */


#include "trapezoidal.h"
#include <math.h>

void trapezoidal_init(TrapezoidalProfile* profile) {
    profile->x0 = 0.0f;
    profile->xf = 0.0f;
    profile->v_max = 1.0f;
    profile->a_max = 1.0f;
    profile->t_accel = 0.0f;
    profile->t_const = 0.0f;
    profile->t_decel = 0.0f;
    profile->t_total = 0.0f;
    profile->profile_type = 0;
    profile->start_time = 0.0f;
    profile->is_active = 0;
    profile->direction = 1.0f;
}

void trapezoidal_set_limits(TrapezoidalProfile* profile, float v_max, float a_max) {
    profile->v_max = v_max;
    profile->a_max = a_max;
}

void trapezoidal_plan(TrapezoidalProfile* profile, float x_start, float x_end) {
    profile->x0 = x_start;
    profile->xf = x_end;

    // Calculate distance and direction
    float distance = x_end - x_start;
    profile->direction = (distance >= 0) ? 1.0f : -1.0f;
    distance = fabsf(distance);

    // Calculate time to reach max velocity
    float t_to_vmax = profile->v_max / profile->a_max;

    // Calculate distance covered during acceleration/deceleration
    float d_accel = 0.5f * profile->a_max * t_to_vmax * t_to_vmax;

    // Check if we can reach max velocity
    if (2.0f * d_accel <= distance) {
        // Trapezoidal profile
        profile->profile_type = 1;
        profile->t_accel = t_to_vmax;
        profile->t_decel = t_to_vmax;
        profile->t_const = (distance - 2.0f * d_accel) / profile->v_max;
    } else {
        // Triangular profile
        profile->profile_type = 0;
        float v_peak = sqrtf(distance * profile->a_max);
        profile->t_accel = v_peak / profile->a_max;
        profile->t_decel = v_peak / profile->a_max;
        profile->t_const = 0.0f;
        profile->v_max = v_peak; // Temporary override for this trajectory
    }

    profile->t_total = profile->t_accel + profile->t_const + profile->t_decel;
}

void trapezoidal_start(TrapezoidalProfile* profile) {
    profile->start_time = HAL_GetTick() / 1000.0f;  // Convert to seconds
    profile->is_active = 1;
}

TrajectoryPoint trapezoidal_get_point(TrapezoidalProfile* profile, float current_time) {
    TrajectoryPoint point;

    if (!profile->is_active) {
        point.position = profile->x0;
        point.velocity = 0.0f;
        point.acceleration = 0.0f;
        return point;
    }

    float t = current_time - profile->start_time;

    if (t < 0) {
        // Before trajectory starts
        point.position = profile->x0;
        point.velocity = 0.0f;
        point.acceleration = 0.0f;
    }
    else if (t <= profile->t_accel) {
        // Acceleration phase
        point.acceleration = profile->a_max * profile->direction;
        point.velocity = profile->a_max * t * profile->direction;
        point.position = profile->x0 + 0.5f * profile->a_max * t * t * profile->direction;
    }
    else if (t <= profile->t_accel + profile->t_const) {
        // Constant velocity phase
        float t_phase = t - profile->t_accel;
        point.acceleration = 0.0f;
        point.velocity = profile->v_max * profile->direction;
        float pos_accel = profile->x0 + 0.5f * profile->a_max * profile->t_accel * profile->t_accel * profile->direction;
        point.position = pos_accel + profile->v_max * t_phase * profile->direction;
    }
    else if (t <= profile->t_total) {
        // Deceleration phase
        float t_phase = t - profile->t_accel - profile->t_const;
        point.acceleration = -profile->a_max * profile->direction;
        point.velocity = (profile->v_max - profile->a_max * t_phase) * profile->direction;
        float pos_accel = profile->x0 + 0.5f * profile->a_max * profile->t_accel * profile->t_accel * profile->direction;
        float pos_const = pos_accel + profile->v_max * profile->t_const * profile->direction;
        point.position = pos_const + (profile->v_max * t_phase - 0.5f * profile->a_max * t_phase * t_phase) * profile->direction;
    }
    else {
        // Trajectory completed
        point.position = profile->xf;
        point.velocity = 0.0f;
        point.acceleration = 0.0f;
        profile->is_active = 0;
    }

    return point;
}

uint8_t trapezoidal_is_complete(TrapezoidalProfile* profile, float current_time) {
    if (!profile->is_active) return 1;

    float t = current_time - profile->start_time;
    return (t >= profile->t_total);
}

void trapezoidal_scale_to_time(TrapezoidalProfile* profile, float new_time) {
    float scale_factor = profile->t_total / new_time;
    profile->v_max *= scale_factor;
    profile->a_max *= scale_factor * scale_factor;
    trapezoidal_plan(profile, profile->x0, profile->xf);
}
