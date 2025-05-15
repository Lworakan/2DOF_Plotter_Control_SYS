/*
 * robot_control.h
 *
 *  Created on: May 3, 2025
 *      Author: lworakan
 */

#ifndef INC_ROBOT_CONTROL_H_
#define INC_ROBOT_CONTROL_H_

#include "main.h"
#include "encoder.h"
#include <math.h>
#include "trapezoidal.h"

// PID Controller structure
typedef struct {
    float kp;
    float ki;
    float kd;
    float error_integral;
    float error_prev;
    float last_derivative;  // For derivative filtering
    float output_min;
    float output_max;
    float dt;               // Sample time
    uint32_t last_time;     // Last update time
} PIDController;

typedef struct {
    PIDController position_controller;
    PIDController velocity_controller;
    float velocity_setpoint;
    float current_setpoint;
    float max_velocity;       // Maximum allowed velocity
    float acceleration_limit; // Maximum allowed acceleration
    uint32_t last_velocity_update; // For calculating dt in acceleration limiting
} CascadeController;

// Motor parameters structure
typedef struct {
    float R;          // Resistance (Ohms)
    float L;          // Inductance (H)
    float Ke;         // Back-EMF constant (V/(rad/s))
    float Kt;         // Torque constant (Nm/A)
    float Jm;         // Motor inertia (kg*m^2)
    float Bm;         // Motor damping (Nms/rad)
    float n_gear;     // Gear ratio
    float r_pulley;   // Pulley radius (m)
    float voltage;
} MotorParams;

typedef struct {
    float m_plotter;       // Mass of plotter (kg)
    float m_counterweight; // Mass of counterweight (kg)
    float m_total;         // Total mass (kg)
    float m_diff;          // Mass difference (kg)
    float g;               // Gravity (m/s^2)
    float J_arm;           // Arm inertia (kg*m^2)
    float c_prismatic;     // Prismatic joint damping (Ns/m)
    float c_revolute;      // Revolute joint damping (Nms/rad)
} SystemParams;

// Feedforward parameters structure
typedef struct {
    float FTp;             // Velocity filter time constant
    float FT;              // Acceleration filter time constant
    float Ts;              // Sample time
    float prev_d_dot_ref;  // Previous prismatic velocity reference
    float prev_q_dot_ref;  // Previous revolute velocity reference
} FeedforwardParams;

// Robot controller structure
typedef struct {
    CascadeController revolute_cascade;
    CascadeController prismatic_cascade;
    MotorParams revolute_motor;
    MotorParams prismatic_motor;
    SystemParams system;
    FeedforwardParams ff_params;

    // Encoder data
    EncoderData revolute_encoder;
    EncoderData prismatic_encoder;

    // Target values
    float revolute_target;  // Target position for revolute joint (rad)
    float prismatic_target; // Target position for prismatic joint (m)

    // Trajectory profiles
    TrapezoidalProfile revolute_trajectory;
    TrapezoidalProfile prismatic_trajectory;

    // Trajectory active flag
    uint8_t trajectory_active;

    // Control status
    uint8_t control_enabled;

    // PWM period for TIM1
    uint32_t pwm_period;

    // Debug variables
    float debug_revolute_velocity;

    // Safety flag
    uint8_t stop_flag;
} RobotController;

// Constants for encoder configuration
#define REVOLUTE_ENCODER_CPR     400     // Counts per revolution for revolute encoder
#define REVOLUTE_GEAR_RATIO      2.0f    // Gear ratio for revolute joint
#define PRISMATIC_ENCODER_CPR    400     // Counts per revolution for prismatic encoder
#define PRISMATIC_GEAR_RATIO     4.0f    // Gear ratio for prismatic joint
#define PRISMATIC_MM_PER_REV     37.6f   // mm travel per revolution for prismatic joint

// Function prototypes
void robot_init(RobotController* robot, TIM_HandleTypeDef* htim_revolute,
                TIM_HandleTypeDef* htim_prismatic);
void robot_update(RobotController* robot);
void robot_set_targets(RobotController* robot, float revolute_deg, float prismatic_mm);
void robot_start_trajectory(RobotController* robot, float revolute_target_deg, float prismatic_target_mm);
uint8_t robot_trajectory_complete(RobotController* robot);
void robot_enable(RobotController* robot);
void robot_disable(RobotController* robot);
void robot_get_state(RobotController* robot, float* revolute_pos, float* revolute_vel,
                     float* prismatic_pos, float* prismatic_vel);

#endif // ROBOT_CONTROL_H
