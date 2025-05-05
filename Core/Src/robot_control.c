/*
 * robot_control.c
 *
 *  Created on: May 3, 2025
 *      Author: lworakan
 */


#include "robot_control.h"

// External HAL references
extern TIM_HandleTypeDef htim1;

// PID functions
static void pid_init(PIDController* pid, float kp, float ki, float kd,
             float out_min, float out_max, float sample_time) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->error_integral = 0;
    pid->error_prev = 0;
    pid->output_min = out_min;
    pid->output_max = out_max;
    pid->dt = sample_time;
    pid->last_time = HAL_GetTick();
    pid->last_derivative = 0;
}

static float pid_update(PIDController* pid, float setpoint, float actual) {
    float error = setpoint - actual;

    uint32_t current_time = HAL_GetTick();
    float dt = (current_time - pid->last_time) / 1000.0f;
    if (dt <= 0.0f || dt > 0.5f) dt = pid->dt; // Use default if time is invalid
    pid->last_time = current_time;

    float p_term = pid->kp * error;

    pid->error_integral += error * dt;

    // Anti-windup - limit integral term
    float max_integral = pid->output_max / pid->ki;
    float min_integral = pid->output_min / pid->ki;
    if (pid->error_integral > max_integral) pid->error_integral = max_integral;
    if (pid->error_integral < min_integral) pid->error_integral = min_integral;

    float i_term = pid->ki * pid->error_integral;

    // Calculate derivative term with filter
    float error_derivative;
    if (dt > 0) {
        error_derivative = (error - pid->error_prev) / dt;
        // Low-pass filter for derivative term
        error_derivative = 0.2f * error_derivative + 0.8f * pid->last_derivative;
    } else {
        error_derivative = 0;
    }
    pid->last_derivative = error_derivative;
    float d_term = pid->kd * error_derivative;

    pid->error_prev = error;

    float output = p_term + i_term + d_term;

    // Output limiting
    if (output > pid->output_max) output = pid->output_max;
    if (output < pid->output_min) output = pid->output_min;

    return output;
}

// Cascade control functions
static void cascade_init(CascadeController* cascade,
                 float pos_kp, float pos_ki, float pos_kd,
                 float vel_kp, float vel_ki, float vel_kd,
                 float dt) {
    pid_init(&cascade->position_controller, pos_kp, pos_ki, pos_kd,
             -1.0, 1.0, dt);
    pid_init(&cascade->velocity_controller, vel_kp, vel_ki, vel_kd,
             -24.0, 24.0, dt);
    cascade->velocity_setpoint = 0.0f;
    cascade->current_setpoint = 0.0f;
    cascade->acceleration_limit = 0.4f; // Set to the required acceleration limit of 0.4 rad/s²
    cascade->max_velocity = 1.0f;       // Set to the required max velocity of 1.0 rad/s
    cascade->last_velocity_update = HAL_GetTick();
}

static float cascade_update(CascadeController* cascade, float pos_ref,
                           float pos_actual, float vel_actual) {
    float raw_velocity_setpoint = pid_update(&cascade->position_controller,
                                         pos_ref, pos_actual);

    uint32_t current_time = HAL_GetTick();
    float dt = (current_time - cascade->last_velocity_update) / 1000.0f;
    if (dt <= 0.0f || dt > 0.5f) dt = 0.01f; // Default to 10ms if timing is invalid
    cascade->last_velocity_update = current_time;

    float max_velocity_change = cascade->acceleration_limit * dt;

    if (raw_velocity_setpoint - cascade->velocity_setpoint > max_velocity_change)
        cascade->velocity_setpoint += max_velocity_change;
    else if (cascade->velocity_setpoint - raw_velocity_setpoint > max_velocity_change)
        cascade->velocity_setpoint -= max_velocity_change;
    else
        cascade->velocity_setpoint = raw_velocity_setpoint;

    if (cascade->velocity_setpoint > cascade->max_velocity)
        cascade->velocity_setpoint = cascade->max_velocity;
    if (cascade->velocity_setpoint < -cascade->max_velocity)
        cascade->velocity_setpoint = -cascade->max_velocity;

    cascade->current_setpoint = pid_update(&cascade->velocity_controller,
                                         cascade->velocity_setpoint, vel_actual);

    return cascade->current_setpoint;
}

// Feedforward functions
static float prismatic_feedforward(float d_ref, float v_ref, float a_ref,
                           float theta, float omega,
                           MotorParams* motor, SystemParams* system) {
    float F_inertia = system->m_total * a_ref;
    float F_coriolis = 2.0f * system->m_total * omega * v_ref;
    float F_centrifugal = system->m_total * omega * omega * d_ref;
    float F_gravity = system->m_diff * system->g * sinf(theta);
    float F_friction = system->c_prismatic * v_ref;

    float F_total = F_inertia + F_coriolis - F_centrifugal + F_gravity + F_friction;

    float T_required = F_total * motor->r_pulley;
    float current_required = T_required / motor->Kt;
    float v_feedforward = motor->R * current_required +
                         motor->Ke * v_ref * motor->n_gear / motor->r_pulley;

    return v_feedforward;
}

static float revolute_feedforward(float theta_ref, float omega_ref, float alpha_ref,
                          float d, float v_prismatic,
                          MotorParams* motor, SystemParams* system) {
    float J_total = system->J_arm + system->m_total * d * d;
    float T_inertia = J_total * alpha_ref;
    float T_coriolis = 2.0f * system->m_total * d * v_prismatic * omega_ref;
    float T_gravity = system->m_diff * d * system->g * cosf(theta_ref);
    float T_friction = system->c_revolute * omega_ref;

    float T_motor_inertia = motor->Jm * alpha_ref * motor->n_gear * motor->n_gear;
    float T_motor_friction = motor->Bm * omega_ref * motor->n_gear;

    float T_total = T_inertia + T_coriolis + T_gravity + T_friction + T_motor_inertia + T_motor_friction;

    float current_required = T_total / (motor->Kt * motor->n_gear);
    float v_feedforward = motor->R * current_required + motor->Ke * omega_ref * motor->n_gear;

    return v_feedforward;
}

// Motor control function
void set_motor_pwm(TIM_HandleTypeDef* htim, uint32_t channel,
                  float voltage, float max_voltage, uint32_t period) {
    if (voltage > max_voltage) voltage = max_voltage;
    if (voltage < -max_voltage) voltage = -max_voltage;

    float duty_cycle = fabsf(voltage) / max_voltage;

    uint32_t pwm_value = (uint32_t)(duty_cycle * period);

    __HAL_TIM_SET_COMPARE(htim, channel, pwm_value);

    if (channel == TIM_CHANNEL_1) {  // Revolute motor
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, voltage >= 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    } else if (channel == TIM_CHANNEL_2) {  // Prismatic motor
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, voltage >= 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

// Public functions
void robot_init(RobotController* robot, TIM_HandleTypeDef* htim_revolute,
                TIM_HandleTypeDef* htim_prismatic) {
    // Initialize motor parameters - Revolute (ZGX45RGG)
    robot->revolute_motor.R = 5.237f;
    robot->revolute_motor.L = 0.176f;
    robot->revolute_motor.Ke = 1.574f;
    robot->revolute_motor.Kt = 1.551f;
    robot->revolute_motor.Jm = 0.009f;
    robot->revolute_motor.Bm = 0.199f;
    robot->revolute_motor.n_gear = 2.0f;
    robot->revolute_motor.r_pulley = 0.03273f;

    // Initialize motor parameters - Prismatic (ZYTD-50SRZ-R1)
    robot->prismatic_motor.R = 2.253f;
    robot->prismatic_motor.L = 0.076f;
    robot->prismatic_motor.Ke = 0.056f;
    robot->prismatic_motor.Kt = 0.055f;
    robot->prismatic_motor.Jm = 9.256e-5f;
    robot->prismatic_motor.Bm = 1.001e-9f;
    robot->prismatic_motor.n_gear = 4.0f;
    robot->prismatic_motor.r_pulley = 0.01273f;

    // Initialize system parameters
    robot->system.m_plotter = 0.2f;
    robot->system.m_counterweight = 0.2f;
    robot->system.m_total = 0.4f;
    robot->system.m_diff = 0.0f;
    robot->system.g = 9.81f;
    robot->system.J_arm = 0.1f;
    robot->system.c_prismatic = 0.05f;
    robot->system.c_revolute = 0.05f;

    // Initialize feedforward parameters
    robot->ff_params.FTp = 0.1f;
    robot->ff_params.FT = 0.1f;
    robot->ff_params.Ts = 0.01f;
    robot->ff_params.prev_d_dot_ref = 0;
    robot->ff_params.prev_q_dot_ref = 0;

    // Initialize cascade controllers with proper PID values
    // Revolute position controller
    pid_init(&robot->revolute_cascade.position_controller,
             5.0f,    // Kp: (rad/s)/rad - controls how fast position errors are corrected
             0.1f,    // Ki: (rad/s)/rad·s - eliminates steady-state position errors
             0.05f,   // Kd: (rad/s)/(rad/s) - dampens position oscillations
             -1.0f,   // Min output: -1 rad/s - max velocity setpoint
             1.0f,
             0.01f);

    // Revolute velocity controller
    pid_init(&robot->revolute_cascade.velocity_controller,
             20.0f,   // Kp: V/(rad/s) - controls how fast velocity errors are corrected
             5.0f,    // Ki: V/(rad) - eliminates steady-state velocity errors
             0.1f,    // Kd: V/(rad/s²) - dampens velocity oscillations
             -24.0f,  // Min output: -24V - max motor voltage
             24.0f,   // Max output: 24V - max motor voltage
             0.01f);  // Sample time: 10 ms

    // Initialize cascade controllers
    cascade_init(&robot->revolute_cascade,
                5.0f, 0.1f, 0.05f,   // Position PID
                20.0f, 5.0f, 0.1f,   // Velocity PID
                0.01f);              // Sample time

    // Set acceleration and velocity limits explicitly
    robot->revolute_cascade.acceleration_limit = 0.4f; // rad/s²
    robot->revolute_cascade.max_velocity = 1.0f;      // rad/s

    cascade_init(&robot->prismatic_cascade,
                8.0f, 0.2f, 0.1f,    // Position PID
                3.0f, 10.0f, 0.02f,  // Velocity PID
                0.01f);              // Sample time

    encoder_init(&robot->revolute_encoder, htim_revolute,
                 REVOLUTE_ENCODER_CPR, REVOLUTE_GEAR_RATIO, 360.0f);
    encoder_init(&robot->prismatic_encoder, htim_prismatic,
                 PRISMATIC_ENCODER_CPR, PRISMATIC_GEAR_RATIO, PRISMATIC_MM_PER_REV);

    // Initialize control state
    robot->revolute_target = 0.0f;
    robot->prismatic_target = 0.1f;  // 100mm default
    robot->control_enabled = 0;
    robot->stop_flag = 0;

    // Set PWM period
    robot->pwm_period = 19999;

    // Initialize trajectory profiles
    trapezoidal_init(&robot->revolute_trajectory);
    trapezoidal_init(&robot->prismatic_trajectory);

    trapezoidal_set_limits(&robot->revolute_trajectory, 1.0f, 0.4f);  // rad/s, rad/s^2
    trapezoidal_set_limits(&robot->prismatic_trajectory, 0.5f, 0.25f); // m/s, m/s^2

    robot->trajectory_active = 0;
}

void robot_update(RobotController* robot) {
    if (!robot->control_enabled || robot->stop_flag) return;

    encoder_read(&robot->revolute_encoder);
    encoder_read(&robot->prismatic_encoder);
    encoder_calculate_speed(&robot->revolute_encoder);
    encoder_calculate_speed(&robot->prismatic_encoder);

    if (fabsf(robot->revolute_encoder.speed_real) > 100.0f) {
            robot->revolute_encoder.speed_real = (robot->revolute_encoder.speed_real > 0) ? 100.0f : -100.0f;
        }
    float revolute_angle = robot->revolute_encoder.position * M_PI / 180.0f; // deg to rad
    float revolute_velocity = robot->revolute_encoder.speed_real * M_PI / 180.0f; // deg/s to rad/s

    float prismatic_position = robot->prismatic_encoder.position / 1000.0f; // mm to m
    float prismatic_velocity = robot->prismatic_encoder.speed_real / 1000.0f; // mm/s to m/s

    uint32_t current_time = HAL_GetTick();
    float revolute_target, revolute_vel_ref, revolute_accel_ref;
    float prismatic_target, prismatic_vel_ref, prismatic_accel_ref;

    if (robot->trajectory_active) {
        TrajectoryPoint rev_point = trapezoidal_get_point(&robot->revolute_trajectory, current_time);
        TrajectoryPoint pri_point = trapezoidal_get_point(&robot->prismatic_trajectory, current_time);

        revolute_target = rev_point.position;
        revolute_vel_ref = rev_point.velocity;
        revolute_accel_ref = rev_point.acceleration;

        prismatic_target = pri_point.position;
        prismatic_vel_ref = pri_point.velocity;
        prismatic_accel_ref = pri_point.acceleration;

        // Check if trajectory is complete
        if (trapezoidal_is_complete(&robot->revolute_trajectory, current_time) &&
            trapezoidal_is_complete(&robot->prismatic_trajectory, current_time)) {
            robot->trajectory_active = 0;
        }
    } else {
        // Use fixed setpoints
        revolute_target = robot->revolute_target;
        prismatic_target = robot->prismatic_target;
        revolute_vel_ref = 0.0f;
        prismatic_vel_ref = 0.0f;
        revolute_accel_ref = 0.0f;
        prismatic_accel_ref = 0.0f;
    }

    // Apply cascade control for each joint
    float v_revolute_fb = cascade_update(&robot->revolute_cascade,
                                       revolute_target, // in radians
                                       revolute_angle, // in radians
                                       revolute_velocity); // in rad/s

    float v_prismatic_fb = cascade_update(&robot->prismatic_cascade,
                                        prismatic_target, // in meters
                                        prismatic_position, // in meters
                                        prismatic_velocity); // in m/s

    // Calculate feedforward compensation
    float v_revolute_ff = revolute_feedforward(
        revolute_target, revolute_vel_ref, revolute_accel_ref,
        prismatic_position, prismatic_velocity,
        &robot->revolute_motor, &robot->system
    );

    float v_prismatic_ff = prismatic_feedforward(
        prismatic_target, prismatic_vel_ref, prismatic_accel_ref,
        revolute_angle, revolute_velocity,
        &robot->prismatic_motor, &robot->system
    );

    float ff_weight = 0.4f;
    float v_revolute_total = v_revolute_fb + ff_weight * v_revolute_ff;
    float v_prismatic_total = v_prismatic_fb + ff_weight * v_prismatic_ff;

    // Apply control signals
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == GPIO_PIN_RESET) {
        set_motor_pwm(&htim1, TIM_CHANNEL_1, v_revolute_fb, 24.0f, robot->pwm_period);
        set_motor_pwm(&htim1, TIM_CHANNEL_2, v_prismatic_fb, 24.0f, robot->pwm_period);
    } else {
        set_motor_pwm(&htim1, TIM_CHANNEL_1, 0.0f, 24.0f, robot->pwm_period);
        set_motor_pwm(&htim1, TIM_CHANNEL_2, 0.0f, 24.0f, robot->pwm_period);
    }
}

void robot_start_trajectory(RobotController* robot, float revolute_target_deg, float prismatic_target_mm) {
    float current_revolute = robot->revolute_encoder.position * M_PI / 180.0f;  // Convert to radians
    float current_prismatic = robot->prismatic_encoder.position / 1000.0f;      // Convert to meters

    float target_revolute = revolute_target_deg * M_PI / 180.0f;
    float target_prismatic = prismatic_target_mm / 1000.0f;

    // Plan trajectories
    trapezoidal_plan(&robot->revolute_trajectory, current_revolute, target_revolute);
    trapezoidal_plan(&robot->prismatic_trajectory, current_prismatic, target_prismatic);

    // Synchronize trajectories to finish at the same time
    float t_revolute = robot->revolute_trajectory.t_total;
    float t_prismatic = robot->prismatic_trajectory.t_total;
    float t_max = fmaxf(t_revolute, t_prismatic);

    if (t_revolute < t_max) {
        trapezoidal_scale_to_time(&robot->revolute_trajectory, t_max);
    }
    if (t_prismatic < t_max) {
        trapezoidal_scale_to_time(&robot->prismatic_trajectory, t_max);
    }

    // Start trajectories
    trapezoidal_start(&robot->revolute_trajectory);
    trapezoidal_start(&robot->prismatic_trajectory);

    robot->trajectory_active = 1;
}

uint8_t robot_trajectory_complete(RobotController* robot) {
    return !robot->trajectory_active;
}

void robot_set_targets(RobotController* robot, float revolute_deg, float prismatic_mm) {
    robot->revolute_target = revolute_deg * M_PI / 180.0f;
    robot->prismatic_target = prismatic_mm / 1000.0f;
}

void robot_enable(RobotController* robot) {
    encoder_reset(&robot->revolute_encoder);
    encoder_reset(&robot->prismatic_encoder);

    robot->revolute_target = 0.0f;
    robot->prismatic_target = 0.0f;

    robot->revolute_cascade.velocity_setpoint = 0.0f;
    robot->revolute_cascade.current_setpoint = 0.0f;
    robot->prismatic_cascade.velocity_setpoint = 0.0f;
    robot->prismatic_cascade.current_setpoint = 0.0f;

    robot->control_enabled = 1;
}

void robot_disable(RobotController* robot) {
    robot->control_enabled = 0;
    set_motor_pwm(&htim1, TIM_CHANNEL_1, 0.0f, 24.0f, robot->pwm_period);
    set_motor_pwm(&htim1, TIM_CHANNEL_2, 0.0f, 24.0f, robot->pwm_period);
}

void robot_get_state(RobotController* robot, float* revolute_pos, float* revolute_vel,
                     float* prismatic_pos, float* prismatic_vel) {
    if (revolute_pos) *revolute_pos = robot->revolute_encoder.position;
    if (revolute_vel) *revolute_vel = robot->revolute_encoder.speed_real;
    if (prismatic_pos) *prismatic_pos = robot->prismatic_encoder.position;
    if (prismatic_vel) *prismatic_vel = robot->prismatic_encoder.speed_real;
}
