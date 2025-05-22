/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "main.h"
#include "arm_math.h"
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
//#include "kalman_wrapper.c"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc3;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
// Prismatic control variables
float position_setpoint_deg = 0.0f;
static uint32_t motion_start_time = 0;
static float current_velocity_setpoint = 0.0f;
static uint8_t motion_complete = 0;
static uint8_t motion_direction = 1;
float control_signal_output;
bool is_prismatic_compensating = false;
float prismatic_max_velocity_normal = 30.0f;  // rad/s for normal operation
float prismatic_max_velocity_compensating = 1.0f;  // rad/s during compensation

float prismatic_target_positions[2] = {1.0f, 0.0f};  // Target positions in radians
int current_target_index = 0;                         // Current target index
float position_tolerance = 0.05f;                     // Position tolerance (radians)
bool prismatic_motion_active = false;                 // Motion activation flag
uint32_t last_target_change_time = 0;                 // Time of last target change
uint32_t min_dwell_time_ms = 2000;

// Trajectory variables
float move_in_progress = 0;
//static float target_velocity = 1.0f;
//static uint8_t use_motor_model = 0;
//static uint32_t filter_switch_time = 0;
//static uint8_t profile_state = 0; // 0: accel, 1: constant, 2: decel, 3: rest
//static float test_velocity = 1.0f;

// Kalman filter parameters
float process_noise = 0.15f;
float meas_noise = 0.4f;
float process_noise_pos = 0.0005f;
float process_noise_vel = 0.05f;
//static uint8_t circle_started = 0;
/* PID Controller structure */
typedef struct {
	float kp;               // Proportional gain
	float ki;               // Integral gain
	float kd;               // Derivative gain
	float error_integral;   // Integral accumulator
	float error_prev;       // Previous error for D term
	float last_derivative;  // Filtered derivative
	float output_min;       // Output lower limit
	float output_max;       // Output upper limit
	float dt;               // Sample time in seconds
	uint32_t last_time;     // Last update timestamp
} PIDController;

typedef struct {
	float Kp;        // Proportional gain
	float Ki;        // Integral gain (Ti = Kp/Ki)
	float Kd;        // Derivative gain (Td = Kd/Kp)

	float prev_error;       // Previous error
	float prev_prev_error;  // Error from two steps ago
	float prev_output;      // Previous control output

	float output_min;       // Minimum output
	float output_max;       // Maximum output
	float delta_max;        // Maximum change in output per cycle

	float T;                // Sampling period in seconds

	bool enabled;           // Enable/disable controller
} VelocityPID;

typedef struct {
	TIM_HandleTypeDef *htim;        // Timer handle
	int32_t count;                  // Encoder count
	int32_t last_count;             // Previous encoder count
	int32_t last_count_for_speed;   // Count used for speed calculation
	float position;                 // Current position in radians
	float speed;                    // Current speed in rad/s
	float speed_filtered;           // Filtered speed (original alpha filter)
	uint32_t last_speed_calc_time;
} EncoderData;

typedef struct {
	VelocityPID position_pid;     // Position loop PID
	VelocityPID velocity_pid;     // Velocity loop PID
	VelocityPID current_pid;      // Optional current loop PID

	float position_setpoint;  // Position setpoint - important to maintain state
	float velocity_setpoint;      // Velocity setpoint
	float current_setpoint;       // Current setpoint

	float velocity_output;        // Output from position controller
	float current_output;
	float max_velocity;

	uint8_t control_enabled; // Enable/disable control (using uint8_t instead of bool)
} CascadeController;

typedef struct {
	float position_kp;
	float position_ki;
	float position_kd;

	float velocity_kp;
	float velocity_ki;
	float velocity_kd;

	float max_velocity;        // Maximum velocity setpoint (rad/s)
	float max_voltage;         // Maximum motor voltage

	float position_setpoint;   // Target position in radians

	uint8_t control_enabled;   // Control enable flag

	uint32_t pwm_period;       // PWM timer period

	uint8_t test_running;
	uint32_t test_start_time;
	float test_amplitude;
	float test_frequency;
} TuningParameters;

typedef struct {
	float R;        // Motor resistance
	float L;        // Motor inductance
	float Kt;       // Torque constant
	float Ke;       // Back-EMF constant
	float J;        // Inertia (combined motor and load)
	float B;        // Damping/friction coefficient
	float n;        // Gear ratio
	float r;        // Pulley radius (for prismatic)
} MotorParams;

typedef struct {
	float mp;       // Plotter mass
	float mc;       // Counter weight mass
	float g;        // Gravity acceleration
	float T;        // Low-pass filter time constant
	bool enabled;   // Enable/disable compensation
} DisturbanceComp;

typedef struct {
	// Reference feedforward
	float T;        // Low-pass filter time constant
	bool enabled;   // Enable/disable feedforward
} FeedforwardComp;

typedef struct {
	float target_velocity;
	float target_accel;
	uint32_t start_time;
	uint8_t test_active;
	uint8_t test_phase;
} TestParameters;

TestParameters test_params;

typedef struct {
	float max_velocity;
	float max_acceleration;
	uint32_t start_time;
	uint8_t test_active;
} TargetParameters;

TargetParameters target_params;

typedef struct {
	float m_total;
	float m_diff;
	float g;
	float J_arm;
	float c_revolute;
	float lambda;
} SystemParams;

MotorParams revolute_motor_params = { .R = 5.237f, // Ohms (from your measurements)
		.L = 0.176f,      // Henry
		.Kt = 1.551f,     // Nm/A
		.Ke = 1.574f,     // V/(rad/s)
		.J = 0.009f,      // kg*m²
		.B = 0.199f,      // Ns/rad
		.n = 4.0f,        // 4:1 gear ratio
		.r = 0.0f         // Not used for revolute
		};

MotorParams prismatic_motor_params = { .R = 2.253f,      // Ohms
		.L = 0.076f,      // Henry
		.Kt = 0.055f,     // Nm/A
		.Ke = 0.056f,     // V/(rad/s)
		.J = 9.256E-5f,   // kg*m²
		.B = 1.001E-9f,   // Ns/rad
		.n = 4.0f,        // 4:1 gear ratio
		.r = 0.01273f     // Pulley radius in meters
		};

DisturbanceComp revolute_disturbance = { .mp = 0.5f,       // Plotter mass in kg
		.mc = 0.5f,       // Counter weight mass in kg
		.g = 9.81f,       // Gravity acceleration
		.T = 0.05f,       // 50ms filter time constant
		.enabled = true   // Enable by default
		};

DisturbanceComp prismatic_disturbance = { .mp = 0.5f,       // Same masses
		.mc = 0.5f, .g = 9.81f, .T = 0.05f, .enabled = true };

FeedforwardComp revolute_feedforward = { .T = 0.05f, // 50ms filter time constant
		.enabled = true   // Enable by default
		};

FeedforwardComp prismatic_feedforward = { .T = 0.05f, .enabled = true };

SystemParams system_params = { .lambda = 0.2f // Example value, adjust as needed
		};
// Define instances for revolute and prismatic joints
EncoderData revolute_encoder;
EncoderData prismatic_encoder;
CascadeController revolute_cascade;
CascadeController prismatic_cascade;
TuningParameters revolute_tuning;
TuningParameters prismatic_tuning;

MotorParams revolute_motor_params;
MotorParams prismatic_motor_params;
SystemParams system_params;
#define MAX_VELOCITY_rev 1.0f
#define MAX_VELOCIT_pris 50.0f

#define ENCODER_CPR         8192
#define GEAR_RATIO          1.0f
#define RAD_PER_DEGREE      0.0174533f
#define ENCODER_TIMER_PERIOD 65535
static float prev_revolute_position = 0.0f;
static int prismatic_pwm = 0;

float displacement_change;
#define SAMPLE_TIME_S 0.001f
float motor_model_velocity;

/* Motor parameters - Adjust based on your motor */
#define MOTOR_RESISTANCE 5.237f      /* Armature resistance (Ohms) */
#define MOTOR_INDUCTANCE 0.176f      /* Armature inductance (H) */
#define MOTOR_INERTIA 0.009f         /* Motor inertia (kg·m²) */
#define MOTOR_FRICTION 0.199f        /* Viscous friction coefficient (N·m·s/rad) */
#define MOTOR_TORQUE_CONST 1.551f    /* Torque constant (N·m/A) */
#define MOTOR_EMF_CONST 1.574f       /* Back-EMF constant (V·s/rad) */

/* Kalman filter configuration */
#define MOTOR_MODEL_PROCESS_NOISE 0.01f  /* Process noise for DC motor model */
#define MOTOR_MODEL_MEAS_NOISE 0.5f      /* Position measurement noise for DC motor model */

#define KINEMATIC_MODEL_PROCESS_NOISE_POS 0.001f  /* Position process noise for kinematic model */
#define KINEMATIC_MODEL_PROCESS_NOISE_VEL 0.1f    /* Velocity process noise for kinematic model */
#define KINEMATIC_MODEL_MEAS_NOISE 0.5f           /* Position measurement noise for kinematic model */

/* Enable debug output for verification */
#define DEBUG_ENABLE 1

/* Buffer size for test data */
#define TEST_BUFFER_SIZE 1000

/***************************************************************
 *                   DC MOTOR MODEL FILTER                     *
 ***************************************************************/

/* Number of states in DC motor model */
#define MOTOR_MODEL_NUM_STATES 4
#define MOTOR_MODEL_NUM_INPUTS 1
#define MOTOR_MODEL_NUM_OUTPUTS 1

/* DC Motor Model Kalman Filter Structure */
typedef struct {
	/* State vector [position; velocity; load_torque; current] */
	float X[MOTOR_MODEL_NUM_STATES];

	/* State covariance matrix */
	float P[MOTOR_MODEL_NUM_STATES * MOTOR_MODEL_NUM_STATES];

	/* System matrices */
	float A[MOTOR_MODEL_NUM_STATES * MOTOR_MODEL_NUM_STATES]; /* Continuous state transition */
	float A_d[MOTOR_MODEL_NUM_STATES * MOTOR_MODEL_NUM_STATES]; /* Discrete state transition */
	float B[MOTOR_MODEL_NUM_STATES * MOTOR_MODEL_NUM_INPUTS]; /* Continuous input matrix */
	float B_d[MOTOR_MODEL_NUM_STATES * MOTOR_MODEL_NUM_INPUTS]; /* Discrete input matrix */
	float C[MOTOR_MODEL_NUM_OUTPUTS * MOTOR_MODEL_NUM_STATES]; /* Output matrix */

	/* Noise matrices */
	float Q[MOTOR_MODEL_NUM_STATES * MOTOR_MODEL_NUM_STATES]; /* Process noise - continuous */
	float Q_d[MOTOR_MODEL_NUM_STATES * MOTOR_MODEL_NUM_STATES]; /* Process noise - discrete */
	float R[MOTOR_MODEL_NUM_OUTPUTS]; /* Measurement noise */

	/* Kalman gain */
	float K[MOTOR_MODEL_NUM_STATES];

	/* Motor parameters */
	float dt; /* Sample time (seconds) */
	float J; /* Motor inertia (kg*m^2) */
	float b; /* Viscous friction coefficient (N*m*s) */
	float Kt; /* Torque constant (N*m/A) */
	float Ke; /* Back-EMF constant (V*s/rad) */
	float Ra; /* Armature resistance (Ohms) */
	float La; /* Armature inductance (H) */

	/* Estimated states */
	float position;
	float velocity;
	float load_torque;
	float current;
} MotorModelKalman;

// Kalman filter instances for both joints
MotorModelKalman revolute_motor_filter;
MotorModelKalman prismatic_motor_filter;

// Trajectory generator structure
typedef struct {
	float max_velocity;        // Maximum velocity (rad/s)
	float max_acceleration;    // Maximum acceleration (rad/s²)

	float start_position;      // Start position (rad)
	float target_position;     // Target position (rad)

	uint32_t start_time;       // Start time (ms)
	float total_time;          // Total trajectory time (s)
	float accel_time;          // Acceleration time (s)
	float cruise_time;         // Constant velocity time (s)
	float decel_time;          // Deceleration time (s)

	uint8_t state; // Current state (0=idle, 1=accel, 2=cruise, 3=decel, 4=done)

	// Output values
	float position;            // Current position (rad)
	float velocity;            // Current velocity (rad/s)
	float acceleration;        // Current acceleration (rad/s²)
} SimpleTrapezoid;

typedef struct {
    float center_revolute;    // Circle center (revolute axis position in radians)
    float center_prismatic;   // Circle center (prismatic axis position in radians)
    float radius;             // Circle radius in radians
    float max_velocity;       // Maximum velocity during drawing (rad/s)
    float max_acceleration;   // Maximum acceleration (rad/s²)
    float segment_angle;      // Segment size in radians
    float current_angle;      // Current angle in the circle traversal
    float target_angle;       // Target ending angle (usually 2π for full circle)
    uint32_t last_update;     // Timestamp of last update
    uint8_t active;           // Is drawing active

    // Trajectory values for monitoring and debugging
    float revolute_trajectory_position;
    float revolute_trajectory_velocity;
    float revolute_trajectory_acceleration;
    float prismatic_trajectory_position;
    float prismatic_trajectory_velocity;
    float prismatic_trajectory_acceleration;

    // Error tracking
    float revolute_position_error;
    float revolute_velocity_error;
    float prismatic_position_error;
    float prismatic_velocity_error;

    // Profile generation parameters
    float angular_velocity;
    float current_vel;
    float accel_distance;
    float decel_distance;
    float constant_vel_distance;

    uint8_t circle_phase;  // 0: accel, 1: constant, 2: decel
    float phase_start_angle;
    float phase_end_angle;
} SmoothCircleDrawer;

typedef struct {
	float revolute_pos;    // Position for revolute axis (radians)
	float prismatic_pos;   // Position for prismatic axis (radians)
} WorkspacePoint;

typedef struct {
	WorkspacePoint *points;        // Array of points to traverse
	int num_points;                // Total number of points
	int current_point;             // Current position in the sequence
	float max_velocity;            // Maximum velocity (rad/s)
	float max_acceleration;        // Maximum acceleration (rad/s²)
	uint8_t active;                // Is test active
	uint8_t point_reached;         // Has current point been reached

	SimpleTrapezoid revolute_trajectory;
	SimpleTrapezoid prismatic_trajectory;
} PointTraversal;

SmoothCircleDrawer circle;
PointTraversal custom_test;
SimpleTrapezoid revolute_trap;
SimpleTrapezoid prismatic_trap;
uint8_t test = 0;
volatile float latest_revolute_position = 0.0f;
volatile float latest_revolute_velocity = 0.0f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC3_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM17_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM15_Init(void);
/* USER CODE BEGIN PFP */
// Common function prototypes
void PID_Init(PIDController *pid, float kp, float ki, float kd, float out_min,
		float out_max, float sample_time);
float PID_Update(PIDController *pid, float setpoint, float actual);
void Cascade_Init(CascadeController *cascade, float pos_kp, float pos_ki,
		float pos_kd, float vel_kp, float vel_ki, float vel_kd, float dt);
float Cascade_Update(CascadeController *cascade, float pos_ref,
		float pos_actual, float vel_actual);
void Encoder_Init(EncoderData *enc, TIM_HandleTypeDef *htim);
void Encoder_Read(EncoderData *enc);
void Encoder_CalculateSpeed(EncoderData *enc);
float ApplyPhaseLead(float input, float alpha, float beta);

// Revolute joint function prototypes
void InitializeRevoluteTuning(void);
void RevoluteControlLoop(void);
void SetRevoluteMotorPWM(float voltage, float max_voltage, uint32_t period);
void MoveRevoluteToPosition(float target_position_rad);
float RevoluteFeedforward(float theta_ref, float omega_ref, float alpha_ref,
		MotorParams *motor, SystemParams *system);
void TestRevolutePositionStep(void);

// Prismatic joint function prototypes
void InitializePrismaticTuning(void);
void PrismaticControlLoop(void);
void SetPrismaticMotorPWM(float voltage, float max_voltage, uint32_t period);
void MovePrismaticToPosition(float target_position);
void SetPWMToPrismatic(float revolute_position, uint32_t prismatic_pwm_period,
		float prismatic_max_voltage);
float CalculateDisplacement(float angle_rad);

// Trajectory generation function prototypes
void Trapezoid_Init(SimpleTrapezoid *t, float max_vel, float max_accel);
void Trapezoid_SetTarget(SimpleTrapezoid *t, float current_pos,
		float target_pos);
void Trapezoid_Update(SimpleTrapezoid *t);
uint8_t Trapezoid_IsComplete(SimpleTrapezoid *t);
void TrapezoidalVelocityProfile(void);

// Common control functions
void EnableControl(void);
void DisableControl(void);
void UpdateRevoluteTuningParameters(float pos_kp, float pos_ki, float pos_kd,
		float vel_kp, float vel_ki, float vel_kd);
void UpdatePrismaticTuningParameters(float pos_kp, float pos_ki, float pos_kd,
		float vel_kp, float vel_ki, float vel_kd);

// Kalman filter functions
void MotorModelKalman_Init(MotorModelKalman *filter, float dt, float J, float b,
		float Kt, float Ke, float Ra, float La, float process_noise,
		float meas_noise);
void MotorModelKalman_DiscretizeSystem(MotorModelKalman *filter);
void MotorModelKalman_Reset(MotorModelKalman *filter);
void MotorModelKalman_Predict(MotorModelKalman *filter, float voltage_input);
void MotorModelKalman_Update(MotorModelKalman *filter,
		float position_measurement);
void MotorModelKalman_Estimate(MotorModelKalman *filter, float voltage_input,
		float position_measurement);

// Matrix utility functions
void MatrixMultiply(float *A, float *B, float *C, int rows_a, int cols_a,
		int cols_b);
void MatrixAdd(float *A, float *B, float *C, int rows, int cols);
void MatrixSubtract(float *A, float *B, float *C, int rows, int cols);
void MatrixTranspose(float *A, float *AT, int rows, int cols);
void MatrixScale(float *A, float *B, float scale, int rows, int cols);
void MatrixCopy(float *src, float *dst, int size);
void MatrixIdentity(float *A, int size);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void PID_Init(PIDController *pid, float kp, float ki, float kd, float out_min,
		float out_max, float sample_time) {
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->error_integral = 0.0f;
	pid->error_prev = 0.0f;
	pid->last_derivative = 0.0f;
	pid->output_min = out_min;
	pid->output_max = out_max;
	pid->dt = sample_time;
	pid->last_time = HAL_GetTick();
}

float PID_Update(PIDController *pid, float setpoint, float actual) {
	float error = setpoint - actual;

	uint32_t current_time = HAL_GetTick();
	float dt = (current_time - pid->last_time) / 1000.0f;

	if (dt <= 0.0f || dt > 0.5f) {
		dt = pid->dt;
	}
	pid->last_time = current_time;

	float p_term = pid->kp * error;

	pid->error_integral += error * dt;

	float max_integral = pid->output_max / pid->ki;
	float min_integral = pid->output_min / pid->ki;

	if (pid->error_integral > max_integral) {
		pid->error_integral = max_integral;
	}
	if (pid->error_integral < min_integral) {
		pid->error_integral = min_integral;
	}

	float i_term = pid->ki * pid->error_integral;

	float error_derivative;
	if (dt > 0) {
		error_derivative = (error - pid->error_prev) / dt;
		error_derivative = 0.2f * error_derivative
				+ 0.8f * pid->last_derivative;
	} else {
		error_derivative = 0.0f;
	}
	pid->last_derivative = error_derivative;
	float d_term = pid->kd * error_derivative;

	pid->error_prev = error;

	float output = p_term + i_term + d_term;

	if (output > pid->output_max) {
		output = pid->output_max;
	}
	if (output < pid->output_min) {
		output = pid->output_min;
	}

	return output;
}
void VelocityPID_Init(VelocityPID *pid, float Kp, float Ki, float Kd, float T) {
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
	pid->T = T;

	pid->prev_error = 0.0f;
	pid->prev_prev_error = 0.0f;
	pid->prev_output = 0.0f;

	pid->output_min = -FLT_MAX;
	pid->output_max = FLT_MAX;
	pid->delta_max = FLT_MAX;

	pid->enabled = true;
}

float VelocityPID_Update(VelocityPID *pid, float setpoint, float measurement) {
	if (!pid->enabled) {
		return pid->prev_output;
	}

	float error = setpoint - measurement;

	float proportional_term = pid->Kp * (error - pid->prev_error);
	float integral_term = (pid->Kp * pid->T / pid->Ki) * error;
	float derivative_term = (pid->Kp * pid->Kd / pid->T)
			* (error - 2.0f * pid->prev_error + pid->prev_prev_error);

	float delta_u = proportional_term + integral_term + derivative_term;

	if (delta_u > pid->delta_max) {
		delta_u = pid->delta_max;
	} else if (delta_u < -pid->delta_max) {
		delta_u = -pid->delta_max;
	}

	float output = pid->prev_output + delta_u;

	if (output > pid->output_max) {
		output = pid->output_max;
	} else if (output < pid->output_min) {
		output = pid->output_min;
	}

	pid->prev_prev_error = pid->prev_error;
	pid->prev_error = error;
	pid->prev_output = output;

	return output;
}
//void Cascade_Init(CascadeController* cascade, float pos_kp, float pos_ki, float pos_kd,
//                 float vel_kp, float vel_ki, float vel_kd, float dt) {
//    PID_Init(&cascade->position_controller, pos_kp, pos_ki, pos_kd, -50.0f, 50.0f, dt);
//    PID_Init(&cascade->velocity_controller, vel_kp, vel_ki, vel_kd, -24.0f, 24.0f, dt);
//
//    cascade->velocity_setpoint = 0.0f;
//    cascade->current_setpoint = 0.0f;
//    cascade->max_velocity = 50.0f;
//}
//
//float Cascade_Update(CascadeController* cascade, float pos_ref, float pos_actual, float vel_actual) {
//    float raw_velocity_setpoint = PID_Update(&cascade->position_controller, pos_ref, pos_actual);
//
//    float max_velocity_change = 0.2f;
//
//    if (raw_velocity_setpoint - cascade->velocity_setpoint > max_velocity_change) {
//        cascade->velocity_setpoint += max_velocity_change;
//    } else if (cascade->velocity_setpoint - raw_velocity_setpoint > max_velocity_change) {
//        cascade->velocity_setpoint -= max_velocity_change;
//    } else {
//        cascade->velocity_setpoint = raw_velocity_setpoint;
//    }
//
//    if (cascade->velocity_setpoint > cascade->max_velocity) {
//        cascade->velocity_setpoint = cascade->max_velocity;
//    }
//    if (cascade->velocity_setpoint < -cascade->max_velocity) {
//        cascade->velocity_setpoint = -cascade->max_velocity;
//    }
//
//    cascade->current_setpoint = PID_Update(&cascade->velocity_controller,
//                                         cascade->velocity_setpoint, vel_actual);
//
//    return cascade->current_setpoint;
//}

void CascadeController_Init(CascadeController *controller, float sample_time) {
	VelocityPID_Init(&controller->position_pid, 5.0f, 0.5f, 0.0f, sample_time);

	VelocityPID_Init(&controller->velocity_pid, 2.0f, 0.2f, 0.05f, sample_time);

	VelocityPID_Init(&controller->current_pid, 10.0f, 1.0f, 0.0f, sample_time);

	controller->position_setpoint = 0.0f;
	controller->velocity_setpoint = 0.0f;
	controller->current_setpoint = 0.0f;

	controller->control_enabled = true;
}

float CascadeController_Update(CascadeController *controller,
		float position_setpoint, float position_feedback,
		float velocity_feedback, float max_velocity_limit) {
	if (!controller->control_enabled) {
		return 0.0f;
	}

	controller->velocity_setpoint = PID_Update(&controller->position_pid,
			position_setpoint, position_feedback);

	// Apply the appropriate velocity limit
	if (controller->velocity_setpoint > max_velocity_limit) {
		controller->velocity_setpoint = max_velocity_limit;
	} else if (controller->velocity_setpoint < -max_velocity_limit) {
		controller->velocity_setpoint = -max_velocity_limit;
	}

	controller->current_setpoint = VelocityPID_Update(&controller->velocity_pid,
			controller->velocity_setpoint, velocity_feedback);

	if (controller->current_setpoint > 24.0f) {
		controller->current_setpoint = 24.0f;
	}
	if (controller->current_setpoint < -24.0f) {
		controller->current_setpoint = -24.0f;
	}

	return controller->current_setpoint;
}

void Encoder_Init(EncoderData *enc, TIM_HandleTypeDef *htim) {
	HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);
//    HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);

	enc->htim = htim;
	enc->count = 0;
	enc->last_count = 0;
	enc->last_count_for_speed = 0;
	enc->position = 0.0f;
	enc->speed = 0.0f;
	enc->speed_filtered = 0.0f;
	enc->last_speed_calc_time = HAL_GetTick();
}

void Encoder_Read(EncoderData *enc) {
	uint16_t current_count = __HAL_TIM_GET_COUNTER(enc->htim);

	int16_t count_diff;

	if (current_count > enc->last_count) {
		if (current_count - enc->last_count > 32768) {
			count_diff = (int16_t) ((int32_t) current_count - 65536
					- (int32_t) enc->last_count);
		} else {
			count_diff = (int16_t) (current_count - enc->last_count);
		}
	} else {
		if (enc->last_count - current_count > 32768) {
			count_diff = (int16_t) ((int32_t) current_count + 65536
					- (int32_t) enc->last_count);
		} else {
			count_diff = (int16_t) (current_count - enc->last_count);
		}
	}

	enc->count += count_diff;
	enc->last_count = current_count;

	enc->position = (float) enc->count / ENCODER_CPR * 2.0f * M_PI / GEAR_RATIO;
}

void Encoder_CalculateSpeed(EncoderData *enc) {
	uint32_t current_time = HAL_GetTick();
	uint32_t time_diff = current_time - enc->last_speed_calc_time;

	if (time_diff >= 10) {  // Calculate speed every 10ms (matches control loop)
		float dt = time_diff / 1000.0f;  // Convert to seconds
		int32_t count_diff = enc->count - enc->last_count_for_speed;
		float raw_speed = (float) count_diff / ENCODER_CPR * 2.0f * M_PI
				/ GEAR_RATIO / dt;

		float alpha = 0.0f; // No filtering initially
		enc->speed_filtered = alpha * raw_speed
				+ (1.0f - alpha) * enc->speed_filtered;

		enc->speed = raw_speed;
		enc->last_count_for_speed = enc->count;
		enc->last_speed_calc_time = current_time;
	}
}

float ApplyPhaseLead(float input, float alpha, float beta) {
	static float prev_input = 0.0f;
	static float output = 0.0f;

	// Simple phase lead: output = input + alpha * (input - prev_input)
	float new_output = input + alpha * (input - prev_input);

	// Apply smoothing if needed
	output = beta * new_output + (1.0f - beta) * output;

	prev_input = input;
	return output;
}

void SetRevoluteMotorPWM(float voltage, float max_voltage, uint32_t period) {
	if (voltage > max_voltage) {
		voltage = max_voltage;
	}
	if (voltage < -max_voltage) {
		voltage = -max_voltage;
	}

	float duty_cycle = fabsf(voltage) / max_voltage;
	uint32_t pwm_value = (uint32_t) (duty_cycle * period);

//    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_value);
//
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1,
//                      voltage >= 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
//

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm_value);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12,
			voltage >= 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void SetPrismaticMotorPWM(float voltage, float max_voltage, uint32_t period) {
	if (voltage > max_voltage) {
		voltage = max_voltage;
	}
	if (voltage < -max_voltage) {
		voltage = -max_voltage;
	}

	float duty_cycle = fabsf(voltage) / max_voltage;
	uint32_t pwm_value = (uint32_t) (duty_cycle * period);

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_value);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1,
			voltage >= 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);

}

void InitializeRevoluteTuning(void) {
	revolute_tuning.position_kp = 0.0f;
	revolute_tuning.position_ki = 0.0f;
	revolute_tuning.position_kd = 0.0f;
	revolute_tuning.velocity_kp = 0.0f;
	revolute_tuning.velocity_ki = 0.0f;
	revolute_tuning.velocity_kd = 0.0f;
	revolute_tuning.max_velocity = 50.0f;
	revolute_tuning.max_voltage = 24.0f;
	revolute_tuning.pwm_period = 999;
	revolute_tuning.position_setpoint = 0.0f;
	revolute_tuning.control_enabled = 0;
	revolute_tuning.test_running = 0;
	revolute_tuning.test_start_time = 0;
	revolute_tuning.test_amplitude = M_PI / 4.0f;  // 45 degrees
	revolute_tuning.test_frequency = 0.2f;

	// Revolute motor parameters
	revolute_motor_params.R = 5.236883729f;
	revolute_motor_params.L = 0.1755341958f;
	revolute_motor_params.Ke = 1.573781418f;
	revolute_motor_params.Kt = 1.551f;
	revolute_motor_params.J = 0.009335780353f;
	revolute_motor_params.B = 0.1999320178f;
	revolute_motor_params.n = 4.0f;
	revolute_motor_params.r = 0.01273f;

	system_params.m_total = 0.6f;
	system_params.m_diff = 0.0f;
	system_params.g = 9.81f;
	system_params.J_arm = 0.02f;
	system_params.c_revolute = 0.02f;

	CascadeController_Init(&revolute_cascade, 0.001f);
//    Cascade_Init(&revolute_cascade,
//                revolute_tuning.position_kp, revolute_tuning.position_ki, revolute_tuning.position_kd,
//                revolute_tuning.velocity_kp, revolute_tuning.velocity_ki, revolute_tuning.velocity_kd,
//                0.001f);  // 1ms sample time

	revolute_cascade.max_velocity = revolute_tuning.max_velocity;

	Encoder_Init(&revolute_encoder, &htim2);

	float Q_increased = 0.3f;
	float R_decreased = 0.01f;

	MotorModelKalman_Init(&revolute_motor_filter, 0.001f,
			revolute_motor_params.J, revolute_motor_params.B,
			revolute_motor_params.Kt, revolute_motor_params.Ke,
			revolute_motor_params.R, revolute_motor_params.L, Q_increased,
			R_decreased);

	Trapezoid_Init(&revolute_trap, 1.0f, 4.0f);
}

void InitializePrismaticTuning(void) {
	prismatic_tuning.position_kp = 0.0f;
	prismatic_tuning.position_ki = 0.0f;
	prismatic_tuning.position_kd = 0.0f;
	prismatic_tuning.velocity_kp = 0.0f;
	prismatic_tuning.velocity_ki = 0.0f;
	prismatic_tuning.velocity_kd = 0.0f;
	prismatic_tuning.max_velocity = 50.0f;
	prismatic_tuning.max_voltage = 24.0f;
	prismatic_tuning.pwm_period = 999;
	prismatic_tuning.position_setpoint = 0.0f;
	prismatic_tuning.control_enabled = 0;
	prismatic_tuning.test_running = 0;
	prismatic_tuning.test_start_time = 0;
	prismatic_tuning.test_amplitude = 0.1f;  // Smaller amplitude for prismatic
	prismatic_tuning.test_frequency = 0.2f;

	// Prismatic motor parameters
	prismatic_motor_params.R = 2.252824525f;
	prismatic_motor_params.L = 0.07603942161f;
	prismatic_motor_params.Ke = 0.05618266023f;
	prismatic_motor_params.Kt = 0.055f;
	prismatic_motor_params.J = 0.0000925625527f;
	prismatic_motor_params.B = 0.000000001001456027f;
	prismatic_motor_params.n = 1.0f;
	prismatic_motor_params.r = 0.017f;

	CascadeController_Init(&prismatic_cascade, 0.001f);
//    Cascade_Init(&prismatic_cascade,
//                prismatic_tuning.position_kp, prismatic_tuning.position_ki, prismatic_tuning.position_kd,
//                prismatic_tuning.velocity_kp, prismatic_tuning.velocity_ki, prismatic_tuning.velocity_kd,
//                0.001f);  // 1ms sample time

	prismatic_cascade.max_velocity = prismatic_tuning.max_velocity;

	Encoder_Init(&prismatic_encoder, &htim3);

	float Q_increased = 0.3f;
	float R_decreased = 0.01f;

	MotorModelKalman_Init(&prismatic_motor_filter, 0.001f,
			prismatic_motor_params.J, prismatic_motor_params.B,
			prismatic_motor_params.Kt, prismatic_motor_params.Ke,
			prismatic_motor_params.R, prismatic_motor_params.L, Q_increased,
			R_decreased);

	Trapezoid_Init(&prismatic_trap, 0.5f, 2.0f);
}

float RevoluteFeedforward(float theta_ref, float omega_ref, float alpha_ref,
		MotorParams *motor, SystemParams *system) {
	float d = 0.3f;  // Distance from joint to center of mass
	float J_total = system->J_arm + system->m_total * d * d;
	float T_inertia = J_total * alpha_ref;
	float T_gravity = system->m_diff * d * system->g * cosf(theta_ref);
	float T_friction = system->c_revolute * omega_ref;

	float T_motor_inertia = motor->J * alpha_ref * motor->n * motor->n;
	float T_motor_friction = motor->B * omega_ref * motor->n;

	float T_total = T_inertia + T_gravity + T_friction + T_motor_inertia
			+ T_motor_friction;

	float current_required = T_total / (motor->Kt * motor->n);
	float v_feedforward = motor->R * current_required
			+ motor->Ke * omega_ref * motor->n;

	return v_feedforward;
}

float CalculateDisplacement(float angle_rad) {
	// Polynomial approximation of the displacement function
	return 0.0492f + 1.33E-03f * angle_rad - 2.32E-05f * angle_rad * angle_rad
			+ 1.4E-07f * powf(angle_rad, 3) - 3.01E-10f * powf(angle_rad, 4)
			- 1.05E-13f * powf(angle_rad, 5);
}

void TestRevolutePositionStep(void) {
	static uint32_t last_step_time = 0;
	static uint8_t step_index = 0;

//    static float position_setpoints[] = {
//        0.0f,                      // Starting position (0 degrees)
//        -0.3/8/RAD_PER_DEGREE,     // Step 1
//        -0.3*2/8/RAD_PER_DEGREE,   // Step 2
//        -0.3*3/8/RAD_PER_DEGREE,   // Step 3
//        -0.3*4/8/RAD_PER_DEGREE,   // Step 4
//        -0.3*5/8/RAD_PER_DEGREE,   // Step 5
//        -0.3*6/8/RAD_PER_DEGREE,   // Step 6
//        -0.3*7/8/RAD_PER_DEGREE,   // Step 7
//        -0.3*8/8/RAD_PER_DEGREE,   // Step 8
//        -0.3*7/8/RAD_PER_DEGREE,   // Back to Step 7
//        -0.3*6/8/RAD_PER_DEGREE,   // Back to Step 6
//        -0.3*5/8/RAD_PER_DEGREE,   // Back to Step 5
//        -0.3*4/8/RAD_PER_DEGREE,   // Back to Step 4
//        -0.3*3/8/RAD_PER_DEGREE,   // Back to Step 3
//        -0.3*2/8/RAD_PER_DEGREE,   // Back to Step 2
//        -0.3/8/RAD_PER_DEGREE,     // Back to Step 1
//        0.0f                       // Back to start
//    };
	static float position_setpoints[] = { 0.0f, // Starting position (0 degrees)
			//        M_PI/2.0f,       // 30 degrees
			//        0.0f,                // Back to 0
			//        M_PI/4.0f,           // 45 degrees
			//        0.0f,                // Back to 0
			//        0.0f,                // Back to 0
			M_PI / 2.0f          // 90 degrees

	};

	static const uint8_t num_setpoints = sizeof(position_setpoints)
			/ sizeof(position_setpoints[0]);

	if (HAL_GetTick() - last_step_time > 3000) {
		step_index = (step_index + 1) % num_setpoints;
		revolute_tuning.position_setpoint = position_setpoints[step_index];
		last_step_time = HAL_GetTick();
	}
}

void TestVelocityBidirectional(void) {
	static uint32_t last_step_time = 0;
	static uint8_t step_index = 0;

	static float velocity_setpoint[] = { 1.0f,  // Starting position (0 degrees)
			1.0f,       // 30 degrees
			//        0.0f,                // Back to 0
			//        M_PI/4.0f,           // 45 degrees
			//        0.0f,                // Back to 0
			//        0.0f,                // Back to 0
			-1.0f, -1.0f,        		// 90 degrees

			};

	static const uint8_t num_setpoints = sizeof(velocity_setpoint)
			/ sizeof(velocity_setpoint[0]);

	if (HAL_GetTick() - last_step_time > 3000) {
		step_index = (step_index + 1) % num_setpoints;
		revolute_cascade.velocity_setpoint = velocity_setpoint[step_index];
		last_step_time = HAL_GetTick();
	}
}

void Trapezoid_Init(SimpleTrapezoid *t, float max_vel, float max_accel) {
	t->max_velocity = max_vel;
	t->max_acceleration = max_accel;
	t->state = 0; // IDLE
	t->position = 0.0f;
	t->velocity = 0.0f;
	t->acceleration = 0.0f;
}

void Trapezoid_SetTarget(SimpleTrapezoid *t, float current_pos,
		float target_pos) {
	t->start_position = current_pos;
	t->target_position = target_pos;
	t->start_time = HAL_GetTick();

	float distance = fabsf(target_pos - current_pos);
	float direction = (target_pos > current_pos) ? 1.0f : -1.0f;

	t->accel_time = t->max_velocity / t->max_acceleration;

	float accel_distance = 0.5f * t->max_acceleration * t->accel_time
			* t->accel_time;

	if (distance > 2.0f * accel_distance) {
		t->decel_time = t->accel_time;
		float cruise_distance = distance - 2.0f * accel_distance;
		t->cruise_time = cruise_distance / t->max_velocity;
	} else {
		t->accel_time = sqrtf(distance / t->max_acceleration);
		t->cruise_time = 0.0f;
		t->decel_time = t->accel_time;
	}

	t->total_time = t->accel_time + t->cruise_time + t->decel_time;
	t->state = 1;
}

void Trapezoid_Update(SimpleTrapezoid *t) {
	if (t->state == 0 || t->state == 4) {
		return;
	}

	float direction = (t->target_position > t->start_position) ? 1.0f : -1.0f;
	uint32_t elapsed_ms = HAL_GetTick() - t->start_time;
	float elapsed_time = elapsed_ms / 1000.0f;

	if (elapsed_time >= t->total_time) {
		t->position = t->target_position;
		t->velocity = 0.0f;
		t->acceleration = 0.0f;
		t->state = 4; // DONE
		return;
	}

	if (elapsed_time < t->accel_time) {

		t->state = 1; // ACCELERATING
		t->acceleration = t->max_acceleration * direction;
		t->velocity = t->acceleration * elapsed_time;
		t->position = t->start_position
				+ 0.5f * t->acceleration * elapsed_time * elapsed_time;
	} else if (elapsed_time < (t->accel_time + t->cruise_time)) {
		// Constant velocity phase
		t->state = 2; // CRUISE
		t->acceleration = 0.0f;
		t->velocity = t->max_acceleration * t->accel_time * direction;

		float time_in_cruise = elapsed_time - t->accel_time;
		float distance_in_accel = 0.5f * t->max_acceleration * t->accel_time
				* t->accel_time;
		float distance_in_cruise = t->velocity * time_in_cruise;

		t->position = t->start_position
				+ (distance_in_accel + distance_in_cruise) * direction;
	} else {
		t->state = 3;
		float time_in_decel = elapsed_time - t->accel_time - t->cruise_time;

		t->acceleration = -t->max_acceleration * direction;
		t->velocity = (t->max_acceleration * t->accel_time
				- t->max_acceleration * time_in_decel) * direction;

		float distance_in_accel = 0.5f * t->max_acceleration * t->accel_time
				* t->accel_time;
		float distance_in_cruise = t->max_acceleration * t->accel_time
				* t->cruise_time;
		float distance_in_decel = (t->max_acceleration * t->accel_time
				* time_in_decel
				- 0.5f * t->max_acceleration * time_in_decel * time_in_decel);

		t->position = t->start_position
				+ (distance_in_accel + distance_in_cruise + distance_in_decel)
						* direction;
	}

	if ((direction > 0 && t->position > t->target_position)
			|| (direction < 0 && t->position < t->target_position)) {
		t->position = t->target_position;
		t->velocity = 0.0f;
		t->acceleration = 0.0f;
		t->state = 4; // DONE
	}
}

uint8_t Trapezoid_IsComplete(SimpleTrapezoid *t) {
	return (t->state == 4); // DONE
}

void MoveRevoluteToPosition(float target_position_rad) {
	Trapezoid_SetTarget(&revolute_trap, revolute_encoder.position,
			target_position_rad);
	move_in_progress = 1;
}

void TrapezoidalVelocityProfile(void) {
	const float max_velocity = 1.0f;        // rad/s
	const float max_acceleration = 0.4f;    // rad/s²

	const float accel_time = max_velocity / max_acceleration; // Time to reach max velocity

	uint32_t current_time = HAL_GetTick();
	if (motion_start_time == 0) {
		motion_start_time = current_time;
		current_velocity_setpoint = 0.0f;
		motion_complete = 0;
	}

	float elapsed_time = (current_time - motion_start_time) / 1000.0f;

	if (elapsed_time < accel_time) {
		current_velocity_setpoint = max_acceleration * elapsed_time;
	} else if (elapsed_time < (accel_time + 3.0f)) { // 3 seconds at max velocity
		current_velocity_setpoint = max_velocity;
	} else if (elapsed_time < (2 * accel_time + 3.0f)) {
		float decel_time = elapsed_time - (accel_time + 3.0f);
		current_velocity_setpoint = max_velocity
				- (max_acceleration * decel_time);
	} else {
		current_velocity_setpoint = 0.0f;

		if (!motion_complete) {
			motion_complete = 1;
			motion_direction = !motion_direction; // Toggle direction

			if (elapsed_time > (2 * accel_time + 5.0f)) {
				motion_start_time = 0; // Reset to start a new cycle
			}
		}
	}

	if (!motion_direction) {
		current_velocity_setpoint = -current_velocity_setpoint;
	}

	revolute_cascade.velocity_setpoint = current_velocity_setpoint;
}
float RevoluteDisturbanceComp(float theta, DisturbanceComp *comp,
		MotorParams *params) {
	if (!comp->enabled)
		return 0.0f;

	// Vin = (nr*R/kt) * (mp-mc)*g*r*cos(θ)
	float mp_minus_mc = comp->mp - comp->mc;
	float cos_theta = cosf(theta);
	float disturbance_torque = mp_minus_mc * comp->g * system_params.lambda
			* cos_theta;

	static float filtered_disturbance = 0.0f;
	static float prev_time = 0.0f;
	float current_time = HAL_GetTick() / 1000.0f;
	float dt = current_time - prev_time;
	prev_time = current_time;

	if (dt > 0.0f && dt < 0.1f) {
		filtered_disturbance += (dt / (comp->T + dt))
				* (disturbance_torque - filtered_disturbance);
	}
	return (params->n * params->R / params->Kt) * filtered_disturbance;
}

float PrismaticDisturbanceComp(float theta, DisturbanceComp *comp,
		MotorParams *params) {
	if (!comp->enabled)
		return 0.0f;

	float mp_minus_mc = comp->mp - comp->mc;
	float sin_theta = sinf(theta);
	float disturbance_force = mp_minus_mc * comp->g * sin_theta;

	static float filtered_disturbance = 0.0f;
	static float prev_time = 0.0f;
	float current_time = HAL_GetTick() / 1000.0f;
	float dt = current_time - prev_time;
	prev_time = current_time;

	if (dt > 0.0f && dt < 0.1f) {
		filtered_disturbance += (dt / (comp->T + dt))
				* (disturbance_force - filtered_disturbance);
	}

	return (params->r * params->R / params->Kt) * filtered_disturbance;
}

//float RevoluteFeedforward(float pos_setpoint, float vel_setpoint, float accel_setpoint,
//                         MotorParams* params, SystemParams* sys_params) {
//    if (!revolute_feedforward.enabled) return 0.0f;
//
//    float J_term = params->J * accel_setpoint;
//    float B_term = params->B * vel_setpoint;
//    float K_term = params->Ke * params->n * vel_setpoint;
//
//    float feedforward = (params->n / params->Kt) *
//                        (params->R * J_term +
//                         params->R * B_term +
//                         K_term);
//
//    static float filtered_ff = 0.0f;
//    static float prev_time = 0.0f;
//    float current_time = HAL_GetTick() / 1000.0f;
//    float dt = current_time - prev_time;
//    prev_time = current_time;
//
//    if (dt > 0.0f && dt < 0.1f) {
//        filtered_ff += (dt / (revolute_feedforward.T + dt)) *
//                      (feedforward - filtered_ff);
//    }
//
//    return filtered_ff;
//}

float PrismaticFeedforward(float pos_setpoint, float vel_setpoint,
		float accel_setpoint, MotorParams *params, SystemParams *sys_params) {
	if (!prismatic_feedforward.enabled)
		return 0.0f;

	float J_term = (params->J
			+ (params->n * params->n) / (params->r * params->r))
			* accel_setpoint;
	float B_term = params->B * vel_setpoint;
	float K_term = params->Ke * params->n * vel_setpoint / params->r;

	float feedforward = (params->r / params->Kt)
			* (params->R * J_term + params->R * B_term + K_term);

	static float filtered_ff = 0.0f;
	static float prev_time = 0.0f;
	float current_time = HAL_GetTick() / 1000.0f;
	float dt = current_time - prev_time;
	prev_time = current_time;

	if (dt > 0.0f && dt < 0.1f) {
		filtered_ff += (dt / (prismatic_feedforward.T + dt))
				* (feedforward - filtered_ff);
	}

	return filtered_ff;
}
//void RevoluteControlLoop(void) {
//	Encoder_Read(&revolute_encoder);
//	Encoder_CalculateSpeed(&revolute_encoder);
//
//	if (move_in_progress) {
//		Trapezoid_Update(&revolute_trap);
//
//		if (Trapezoid_IsComplete(&revolute_trap)) {
//			move_in_progress = 0;
//		}
//	}
//
//	float position_setpoint =
//			move_in_progress ?
//					revolute_trap.position : revolute_tuning.position_setpoint;
//	float velocity_setpoint =
//			move_in_progress ?
//					revolute_trap.velocity : revolute_cascade.velocity_setpoint;
//	float acceleration_setpoint =
//			move_in_progress ? revolute_trap.acceleration : 0.0f;
//
//	float control_signal = revolute_cascade.current_setpoint;
//	MotorModelKalman_Estimate(&revolute_motor_filter, control_signal,
//			revolute_encoder.position);
//
//	float filtered_velocity = revolute_motor_filter.velocity;
//
//	if (!revolute_tuning.control_enabled) {
//		SetRevoluteMotorPWM(0.0f, revolute_tuning.max_voltage,
//				revolute_tuning.pwm_period);
//		return;
//	}
//
//	float compensated_velocity = ApplyPhaseLead(filtered_velocity, 0.5f, 0.6f);
//
//	float v_feedback = CascadeController_Update(&revolute_cascade,
//			position_setpoint, revolute_encoder.position, revolute_motor_filter.velocity,
//			1.0f);
//
//	float v_feedforward = RevoluteFeedforward(position_setpoint,
//			velocity_setpoint, acceleration_setpoint, &revolute_motor_params,
//			&system_params);
//
//	float v_disturbance = RevoluteDisturbanceComp(revolute_encoder.position,
//			&revolute_disturbance, &revolute_motor_params);
//
//	float control_signal_output = v_feedback + v_feedforward + v_disturbance;
//
//	SetRevoluteMotorPWM(control_signal_output, revolute_tuning.max_voltage,
//			revolute_tuning.pwm_period);
//
//	latest_revolute_position = revolute_encoder.position;
//	latest_revolute_velocity = revolute_encoder.speed;
//}
//
//void PrismaticControlLoop(void) {
//    Encoder_Read(&prismatic_encoder);
//    Encoder_CalculateSpeed(&prismatic_encoder);
//
//    float revolute_position = latest_revolute_position;
//    float revolute_velocity = revolute_encoder.speed;
//
//    float revolute_change = revolute_position - prev_revolute_position;
//    prev_revolute_position = revolute_position;
//
//
//    float prismatic_change = -revolute_change;
//
//    if (!move_in_progress) {
//
//        prismatic_tuning.position_setpoint += prismatic_change;
//
//
//        prismatic_cascade.velocity_setpoint = -revolute_velocity;
//    }
//
//    if (!prismatic_tuning.control_enabled) {
//        SetPrismaticMotorPWM(0.0f, prismatic_tuning.max_voltage, prismatic_tuning.pwm_period);
//        return;
//    }
//
//    float v_feedforward = PrismaticFeedforward(
//            prismatic_tuning.position_setpoint,
//            prismatic_cascade.velocity_setpoint,
//            0.0f, // No acceleration feedforward
//            &prismatic_motor_params,
//            &system_params);
//
//    float v_disturbance = PrismaticDisturbanceComp(revolute_position,
//            &prismatic_disturbance,
//            &prismatic_motor_params);
//
//    float v_feedback = CascadeController_Update(&prismatic_cascade,
//            prismatic_tuning.position_setpoint,
//            prismatic_encoder.position,
//            prismatic_encoder.speed,
//            30.0f); // Max velocity limit
//
//    float control_signal_output = v_feedback + v_feedforward + v_disturbance;
//
//    SetPrismaticMotorPWM(control_signal_output, prismatic_tuning.max_voltage,
//            prismatic_tuning.pwm_period);
//}
void EnableControl(void) {
	// Reset integral terms
//    revolute_cascade.position_controller.error_integral = 0.0f;
//    revolute_cascade.velocity_controller.error_integral = 0.0f;
//    prismatic_cascade.position_controller.error_integral = 0.0f;
//    prismatic_cascade.velocity_controller.error_integral = 0.0f;

	// Set initial setpoints to current positions
	revolute_tuning.position_setpoint = revolute_encoder.position;
	prismatic_tuning.position_setpoint = prismatic_encoder.position;

	// Enable control for both joints
	revolute_tuning.control_enabled = 1;
	prismatic_tuning.control_enabled = 1;
}

void DisableControl(void) {
	revolute_tuning.control_enabled = 0;
	prismatic_tuning.control_enabled = 0;

	SetRevoluteMotorPWM(0.0f, revolute_tuning.max_voltage,
			revolute_tuning.pwm_period);
	SetPrismaticMotorPWM(0.0f, prismatic_tuning.max_voltage,
			prismatic_tuning.pwm_period);
}

//typedef struct {
//    VelocityPID position_pid;     // Position loop PID
//    VelocityPID velocity_pid;     // Velocity loop PID
//    VelocityPID current_pid;      // Optional current loop PID
//
//    float position_setpoint;      // Position setpoint - important to maintain state
//    float velocity_setpoint;      // Velocity setpoint
//    float current_setpoint;       // Current setpoint
//
//    float velocity_output;        // Output from position controller
//    float current_output;
//    float max_velocity;
//
//    uint8_t control_enabled;      // Enable/disable control (using uint8_t instead of bool)
//} CascadeController;
//

void UpdateRevoluteTuningParameters(float pos_kp, float pos_ki, float pos_kd,
		float vel_kp, float vel_ki, float vel_kd) {
	revolute_tuning.position_kp = pos_kp;
	revolute_tuning.position_ki = pos_ki;
	revolute_tuning.position_kd = pos_kd;
	revolute_tuning.velocity_kp = vel_kp;
	revolute_tuning.velocity_ki = vel_ki;
	revolute_tuning.velocity_kd = vel_kd;

	revolute_cascade.position_pid.Kp = pos_kp;
	revolute_cascade.position_pid.Ki = pos_ki;
	revolute_cascade.position_pid.Kd = pos_kd;

	revolute_cascade.velocity_pid.Kp = vel_kp;
	revolute_cascade.velocity_pid.Ki = vel_ki;
	revolute_cascade.velocity_pid.Kd = vel_kd;

//    revolute_cascade.position_pid.error_integral = 0.0f;
//    revolute_cascade.velocity_pid.error_integral = 0.0f;
}

void UpdatePrismaticTuningParameters(float pos_kp, float pos_ki, float pos_kd,
		float vel_kp, float vel_ki, float vel_kd) {
	prismatic_tuning.position_kp = pos_kp;
	prismatic_tuning.position_ki = pos_ki;
	prismatic_tuning.position_kd = pos_kd;
	prismatic_tuning.velocity_kp = vel_kp;
	prismatic_tuning.velocity_ki = vel_ki;
	prismatic_tuning.velocity_kd = vel_kd;

	prismatic_cascade.position_pid.Kp = pos_kp;
	prismatic_cascade.position_pid.Ki = pos_ki;
	prismatic_cascade.position_pid.Kd = pos_kd;

	prismatic_cascade.velocity_pid.Kp = vel_kp;
	prismatic_cascade.velocity_pid.Ki = vel_ki;
	prismatic_cascade.velocity_pid.Kd = vel_kd;

//    prismatic_cascade.position_controller.error_integral = 0.0f;
//    prismatic_cascade.velocity_controller.error_integral = 0.0f;
}

/***************************************************************
 *              DC MOTOR MODEL IMPLEMENTATION                  *
 ***************************************************************/

void MotorModelKalman_Init(MotorModelKalman *filter, float dt, float J, float b,
		float Kt, float Ke, float Ra, float La, float process_noise,
		float meas_noise) {
	/* Store motor parameters */
	filter->dt = dt;
	filter->J = J;
	filter->b = b;
	filter->Kt = Kt;
	filter->Ke = Ke;
	filter->Ra = Ra;
	filter->La = La;

	/* Initialize state vector to zeros */
	memset(filter->X, 0, sizeof(filter->X));

	/* Initialize covariance matrix with uncertainty on diagonal */
	memset(filter->P, 0, sizeof(filter->P));
	for (int i = 0; i < MOTOR_MODEL_NUM_STATES; i++) {
		filter->P[i * MOTOR_MODEL_NUM_STATES + i] = 100.0f;
	}

	memset(filter->C, 0, sizeof(filter->C));
	filter->C[0] = 1.0f; /* We measure position only */

	filter->R[0] = meas_noise * meas_noise * 0.5f; // Reduce by 50% to trust measurements more

	memset(filter->A, 0, sizeof(filter->A));

	filter->A[0 * MOTOR_MODEL_NUM_STATES + 1] = 1.0f; /* dθ/dt = ω */
	filter->A[1 * MOTOR_MODEL_NUM_STATES + 1] = (-filter->b / filter->J) * 1.5f; /* Friction */
	filter->A[1 * MOTOR_MODEL_NUM_STATES + 2] = -1.0f / filter->J; /* Load torque */
	filter->A[1 * MOTOR_MODEL_NUM_STATES + 3] = filter->Kt / filter->J; /* Motor torque */
	filter->A[3 * MOTOR_MODEL_NUM_STATES + 1] = -filter->Ke / filter->La; /* Back-EMF */
	filter->A[3 * MOTOR_MODEL_NUM_STATES + 3] = -filter->Ra / filter->La; /* Resistance */

	memset(filter->B, 0, sizeof(filter->B));
	filter->B[3] = 1.0f / filter->La; /* Voltage input affects current */

	memset(filter->Q, 0, sizeof(filter->Q));
	filter->Q[0 * MOTOR_MODEL_NUM_STATES + 0] = (process_noise * 0.5f)
			* (process_noise * 0.5f); // Position
	filter->Q[1 * MOTOR_MODEL_NUM_STATES + 1] = (process_noise * 0.2f)
			* (process_noise * 0.2f); // Velocity - reduce from original
	filter->Q[2 * MOTOR_MODEL_NUM_STATES + 2] = (process_noise * 0.5f)
			* (process_noise * 0.5f); // Load torque
	filter->Q[3 * MOTOR_MODEL_NUM_STATES + 3] = (process_noise * 0.1f)
			* (process_noise * 0.1f); // Current

	filter->R[0] = meas_noise * meas_noise * 1.5f; // Increase by 50% to rely more on model

	MotorModelKalman_DiscretizeSystem(filter);
}

void MotorModelKalman_DiscretizeSystem(MotorModelKalman *filter) {
	/*
	 * Discretize continuous time system using Euler method:
	 * A_d = I + A*dt
	 * B_d = B*dt
	 * Q_d = Q*dt
	 */

	/* Initialize discrete A matrix with identity */
	MatrixIdentity(filter->A_d, MOTOR_MODEL_NUM_STATES);

	/* A_d = I + A*dt */
	for (int i = 0; i < MOTOR_MODEL_NUM_STATES; i++) {
		for (int j = 0; j < MOTOR_MODEL_NUM_STATES; j++) {
			filter->A_d[i * MOTOR_MODEL_NUM_STATES + j] += filter->A[i
					* MOTOR_MODEL_NUM_STATES + j] * filter->dt;
		}
	}

	/* B_d = B*dt */
	for (int i = 0; i < MOTOR_MODEL_NUM_STATES; i++) {
		filter->B_d[i] = filter->B[i] * filter->dt;
	}

	/* Q_d = Q*dt */
	for (int i = 0; i < MOTOR_MODEL_NUM_STATES; i++) {
		for (int j = 0; j < MOTOR_MODEL_NUM_STATES; j++) {
			filter->Q_d[i * MOTOR_MODEL_NUM_STATES + j] = filter->Q[i
					* MOTOR_MODEL_NUM_STATES + j] * filter->dt;
		}
	}
}

void MotorModelKalman_Reset(MotorModelKalman *filter) {
	/* Reset state vector */
	memset(filter->X, 0, sizeof(filter->X));

	/* Reset covariance matrix with high uncertainty */
	memset(filter->P, 0, sizeof(filter->P));
	for (int i = 0; i < MOTOR_MODEL_NUM_STATES; i++) {
		filter->P[i * MOTOR_MODEL_NUM_STATES + i] = 100.0f;
	}

	/* Reset state estimates */
	filter->position = 0.0f;
	filter->velocity = 0.0f;
	filter->load_torque = 0.0f;
	filter->current = 0.0f;
}

void MotorModelKalman_Predict(MotorModelKalman *filter, float voltage_input) {
	float temp_state[MOTOR_MODEL_NUM_STATES] = { 0 };
	float temp_P[MOTOR_MODEL_NUM_STATES * MOTOR_MODEL_NUM_STATES] = { 0 };

	/* 1. State prediction: X = A_d*X + B_d*u */

	/* Calculate A_d*X */
	for (int i = 0; i < MOTOR_MODEL_NUM_STATES; i++) {
		temp_state[i] = 0;
		for (int j = 0; j < MOTOR_MODEL_NUM_STATES; j++) {
			temp_state[i] += filter->A_d[i * MOTOR_MODEL_NUM_STATES + j]
					* filter->X[j];
		}
	}

	/* Add B_d*u */
	for (int i = 0; i < MOTOR_MODEL_NUM_STATES; i++) {
		filter->X[i] = temp_state[i] + filter->B_d[i] * voltage_input;
	}

	/* 2. Proper covariance prediction: P = A_d*P*A_d' + Q_d */

	/* First calculate A_d*P -> temp_P */
	for (int i = 0; i < MOTOR_MODEL_NUM_STATES; i++) {
		for (int j = 0; j < MOTOR_MODEL_NUM_STATES; j++) {
			temp_P[i * MOTOR_MODEL_NUM_STATES + j] = 0;
			for (int k = 0; k < MOTOR_MODEL_NUM_STATES; k++) {
				temp_P[i * MOTOR_MODEL_NUM_STATES + j] += filter->A_d[i
						* MOTOR_MODEL_NUM_STATES + k]
						* filter->P[k * MOTOR_MODEL_NUM_STATES + j];
			}
		}
	}

	/* Now calculate (A_d*P)*A_d' -> filter->P */
	memset(filter->P, 0, sizeof(filter->P));
	for (int i = 0; i < MOTOR_MODEL_NUM_STATES; i++) {
		for (int j = 0; j < MOTOR_MODEL_NUM_STATES; j++) {
			for (int k = 0; k < MOTOR_MODEL_NUM_STATES; k++) {
				filter->P[i * MOTOR_MODEL_NUM_STATES + j] += temp_P[i
						* MOTOR_MODEL_NUM_STATES + k]
						* filter->A_d[j * MOTOR_MODEL_NUM_STATES + k]; /* Note: A_d transpose */
			}
		}
	}

	/* Add Q_d */
	for (int i = 0; i < MOTOR_MODEL_NUM_STATES; i++) {
		for (int j = 0; j < MOTOR_MODEL_NUM_STATES; j++) {
			filter->P[i * MOTOR_MODEL_NUM_STATES + j] += filter->Q_d[i
					* MOTOR_MODEL_NUM_STATES + j];
		}
	}

	/* Update state estimates */
	filter->position = filter->X[0];
	filter->velocity = filter->X[1];
	filter->load_torque = filter->X[2];
	filter->current = filter->X[3];
}

void MotorModelKalman_Update(MotorModelKalman *filter,
		float position_measurement) {
	/* Measurement innovation (error): y = z - H*x */
	float innovation = position_measurement - filter->X[0];

	/* Innovation covariance: S = H*P*H' + R = P(0,0) + R */
	float S = filter->P[0] + filter->R[0];

	/* Kalman gain: K = P*H'/S */
	for (int i = 0; i < MOTOR_MODEL_NUM_STATES; i++) {
		filter->K[i] = filter->P[i * MOTOR_MODEL_NUM_STATES + 0] / S;
	}

	/* State update: X = X + K*innovation */
	for (int i = 0; i < MOTOR_MODEL_NUM_STATES; i++) {
		filter->X[i] += filter->K[i] * innovation;
	}

	/* Covariance update: P = (I - K*H)*P */
	/* Since H = [1 0 0 0], this simplifies to: */
	for (int i = 0; i < MOTOR_MODEL_NUM_STATES; i++) {
		for (int j = 0; j < MOTOR_MODEL_NUM_STATES; j++) {
			filter->P[i * MOTOR_MODEL_NUM_STATES + j] -= filter->K[i]
					* filter->P[0 * MOTOR_MODEL_NUM_STATES + j];
		}
	}

	/* Update state estimates */
	filter->position = filter->X[0];
	filter->velocity = filter->X[1];
	filter->load_torque = filter->X[2];
	filter->current = filter->X[3];
}

void MotorModelKalman_Estimate(MotorModelKalman *filter, float voltage_input,
		float position_measurement) {
	MotorModelKalman_Predict(filter, voltage_input);
	MotorModelKalman_Update(filter, position_measurement);
}

/***************************************************************
 *                   UTILITY FUNCTIONS                         *
 ***************************************************************/

void MatrixMultiply(float *A, float *B, float *C, int rows_a, int cols_a,
		int cols_b) {
	for (int i = 0; i < rows_a; i++) {
		for (int j = 0; j < cols_b; j++) {
			C[i * cols_b + j] = 0.0f;
			for (int k = 0; k < cols_a; k++) {
				C[i * cols_b + j] += A[i * cols_a + k] * B[k * cols_b + j];
			}
		}
	}
}

void MatrixAdd(float *A, float *B, float *C, int rows, int cols) {
	for (int i = 0; i < rows * cols; i++) {
		C[i] = A[i] + B[i];
	}
}

void MatrixSubtract(float *A, float *B, float *C, int rows, int cols) {
	for (int i = 0; i < rows * cols; i++) {
		C[i] = A[i] - B[i];
	}
}

void MatrixTranspose(float *A, float *AT, int rows, int cols) {
	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < cols; j++) {
			AT[j * rows + i] = A[i * cols + j];
		}
	}
}

void MatrixScale(float *A, float *B, float scale, int rows, int cols) {
	for (int i = 0; i < rows * cols; i++) {
		B[i] = A[i] * scale;
	}
}

void MatrixCopy(float *src, float *dst, int size) {
	for (int i = 0; i < size; i++) {
		dst[i] = src[i];
	}
}

void MatrixIdentity(float *A, int size) {
	memset(A, 0, size * size * sizeof(float));
	for (int i = 0; i < size; i++) {
		A[i * size + i] = 1.0f;
	}
}

void MotorControlUpdate(void) {
	RevoluteControlLoop();
	PrismaticControlLoop();
}

void IntegratedControlLoop() {
	Encoder_Read(&revolute_encoder);
	Encoder_CalculateSpeed(&revolute_encoder);

	Encoder_Read(&prismatic_encoder);
	Encoder_CalculateSpeed(&prismatic_encoder);

	if (move_in_progress) {
		Trapezoid_Update(&revolute_trap);
		Trapezoid_Update(&prismatic_trap);

		if (Trapezoid_IsComplete(&revolute_trap)
				&& Trapezoid_IsComplete(&prismatic_trap)) {
			move_in_progress = 0;
		}
	}

	float revolute_setpoint =
			move_in_progress ?
					revolute_trap.position : revolute_tuning.position_setpoint;
	float revolute_velocity =
			move_in_progress ?
					revolute_trap.velocity : revolute_cascade.velocity_setpoint;
	float revolute_acceleration =
			move_in_progress ? revolute_trap.acceleration : 0.0f;

	float prismatic_setpoint =
			move_in_progress ?
					prismatic_trap.position :
					prismatic_tuning.position_setpoint;
	float prismatic_velocity =
			move_in_progress ?
					prismatic_trap.velocity :
					prismatic_cascade.velocity_setpoint;
	float prismatic_acceleration =
			move_in_progress ? prismatic_trap.acceleration : 0.0f;

	float revolute_output = CascadeController_Update(&revolute_cascade,
			revolute_setpoint, revolute_encoder.position,
			revolute_encoder.speed, 1.0f);

	float revolute_ff = RevoluteFeedforward(revolute_setpoint,
			revolute_velocity, revolute_acceleration, &revolute_motor_params,
			&system_params);

	float revolute_dist = RevoluteDisturbanceComp(revolute_encoder.position,
			&revolute_disturbance, &revolute_motor_params);

	float revolute_control = revolute_output + revolute_ff + revolute_dist;

	float prismatic_output = CascadeController_Update(&prismatic_cascade,
			prismatic_setpoint, prismatic_encoder.position,
			prismatic_encoder.speed, 30.0f);

	float prismatic_ff = PrismaticFeedforward(prismatic_setpoint,
			prismatic_velocity, prismatic_acceleration, &prismatic_motor_params,
			&system_params);

	float prismatic_dist = PrismaticDisturbanceComp(revolute_encoder.position,
			&prismatic_disturbance, &prismatic_motor_params);

	float prismatic_control = prismatic_output + prismatic_ff + prismatic_dist;

	SetRevoluteMotorPWM(revolute_control, revolute_tuning.max_voltage,
			revolute_tuning.pwm_period);
	SetPrismaticMotorPWM(prismatic_control, prismatic_tuning.max_voltage,
			prismatic_tuning.pwm_period);
}


// Initialize the circle drawer
void SmoothCircleDrawer_Init(SmoothCircleDrawer* drawer,
                             float center_revolute, float center_prismatic,
                             float radius, float max_velocity, float max_acceleration) {
    drawer->center_revolute = center_revolute;
    drawer->center_prismatic = center_prismatic;
    drawer->radius = radius;
    drawer->max_velocity = max_velocity;
    drawer->max_acceleration = max_acceleration;

    // Small segment angle for smooth motion
    drawer->segment_angle = 0.02f;  // ~1.1 degrees

    drawer->current_angle = 0.0f;
    drawer->target_angle = 2.0f * M_PI;  // Full circle by default
    drawer->last_update = 0;
    drawer->active = 0;

    // Initialize trajectory and error values
    drawer->revolute_trajectory_position = 0.0f;
    drawer->revolute_trajectory_velocity = 0.0f;
    drawer->revolute_trajectory_acceleration = 0.0f;
    drawer->prismatic_trajectory_position = 0.0f;
    drawer->prismatic_trajectory_velocity = 0.0f;
    drawer->prismatic_trajectory_acceleration = 0.0f;

    drawer->revolute_position_error = 0.0f;
    drawer->revolute_velocity_error = 0.0f;
    drawer->prismatic_position_error = 0.0f;
    drawer->prismatic_velocity_error = 0.0f;

    // Calculate profile parameters
    drawer->angular_velocity = max_velocity / radius;
    drawer->current_vel = 0.0f;

    // Calculate acceleration and deceleration angles
    float accel_angle = max_velocity / (radius * max_acceleration);

    // Allow 20% for acceleration, 60% for constant velocity, 20% for deceleration
    drawer->accel_distance = accel_angle;
    drawer->decel_distance = accel_angle;
    drawer->constant_vel_distance = 2.0f * M_PI - 2.0f * accel_angle;

    // If acceleration distance is too large, adjust
    if (drawer->accel_distance > M_PI / 3.0f) {
        drawer->accel_distance = M_PI / 3.0f;
        drawer->decel_distance = M_PI / 3.0f;
        drawer->constant_vel_distance = 2.0f * M_PI - 2.0f * (M_PI / 3.0f);
    }

    drawer->circle_phase = 0;  // Start in acceleration phase
    drawer->phase_start_angle = 0.0f;
    drawer->phase_end_angle = drawer->accel_distance;
}

// Start drawing the circle
void SmoothCircleDrawer_Start(SmoothCircleDrawer* drawer) {
    drawer->current_angle = 0.0f;
    drawer->active = 1;
    drawer->last_update = HAL_GetTick();

    // Initialize phase parameters
    drawer->circle_phase = 0;  // Start in acceleration phase
    drawer->phase_start_angle = 0.0f;
    drawer->phase_end_angle = drawer->accel_distance;
    drawer->current_vel = 0.0f;

    // Initial positions
    drawer->revolute_trajectory_position = drawer->center_revolute + drawer->radius;
    drawer->prismatic_trajectory_position = drawer->center_prismatic;

    // Set initial setpoints
    revolute_tuning.position_setpoint = drawer->revolute_trajectory_position;
    prismatic_tuning.position_setpoint = drawer->prismatic_trajectory_position;
}

// Update the circle drawing progress
void SmoothCircleDrawer_Update(SmoothCircleDrawer* drawer) {
    if (!drawer->active) return;

    uint32_t current_time = HAL_GetTick();
    float dt = (current_time - drawer->last_update) / 1000.0f;
    drawer->last_update = current_time;

    // Limit time step to prevent large jumps
    if (dt > 0.05f) dt = 0.05f;

    // Calculate velocity based on current phase
    float target_velocity;
    float acceleration;

    switch (drawer->circle_phase) {
        case 0:  // Acceleration phase
            // Linear acceleration to max velocity
            acceleration = drawer->max_acceleration;
            drawer->current_vel += acceleration * dt;

            // Limit to max velocity
            if (drawer->current_vel > drawer->max_velocity) {
                drawer->current_vel = drawer->max_velocity;
            }

            // Calculate angle increment
            float angle_increment = drawer->current_vel * dt / drawer->radius;
            drawer->current_angle += angle_increment;

            // Check if we've reached the end of acceleration phase
            if (drawer->current_angle >= drawer->phase_end_angle) {
                drawer->circle_phase = 1;  // Move to constant velocity phase
                drawer->phase_start_angle = drawer->current_angle;
                drawer->phase_end_angle = 2.0f * M_PI - drawer->decel_distance;
            }
            break;

        case 1:  // Constant velocity phase
            // Maintain max velocity
            drawer->current_vel = drawer->max_velocity;

            // Calculate angle increment
            angle_increment = drawer->current_vel * dt / drawer->radius;
            drawer->current_angle += angle_increment;

            // Check if we've reached the end of constant velocity phase
            if (drawer->current_angle >= drawer->phase_end_angle) {
                drawer->circle_phase = 2;  // Move to deceleration phase
                drawer->phase_start_angle = drawer->current_angle;
                drawer->phase_end_angle = 2.0f * M_PI;
            }
            break;

        case 2:  // Deceleration phase
            // Linear deceleration to zero
            acceleration = -drawer->max_acceleration;
            drawer->current_vel += acceleration * dt;

            // Ensure velocity doesn't go negative
            if (drawer->current_vel < 0.0f) {
                drawer->current_vel = 0.0f;
            }

            // Calculate angle increment
            angle_increment = drawer->current_vel * dt / drawer->radius;
            drawer->current_angle += angle_increment;

            // Check if we've reached the end of the circle
            if (drawer->current_angle >= drawer->phase_end_angle ||
                drawer->current_vel <= 0.01f) {
                drawer->current_angle = 2.0f * M_PI;
                drawer->active = 0;  // Circle drawing complete
                drawer->current_vel = 0.0f;
            }
            break;
    }

    // Calculate tangential velocity
    float tangential_velocity = drawer->current_vel;

    // Calculate angular velocity
    float angular_velocity = tangential_velocity / drawer->radius;

    // Calculate positions for both axes
    drawer->revolute_trajectory_position = drawer->center_revolute +
                                          drawer->radius * cosf(drawer->current_angle);
    drawer->prismatic_trajectory_position = drawer->center_prismatic +
                                           drawer->radius * sinf(drawer->current_angle);

    // Calculate velocities (tangential to the circle)
    drawer->revolute_trajectory_velocity = -drawer->radius * angular_velocity *
                                          sinf(drawer->current_angle);
    drawer->prismatic_trajectory_velocity = drawer->radius * angular_velocity *
                                           cosf(drawer->current_angle);

    // Calculate accelerations
    // For revolute axis: a = -r * (α * sin(θ) + ω² * cos(θ))
    // For prismatic axis: a = r * (α * cos(θ) - ω² * sin(θ))
    float angular_acceleration = 0.0f;
    if (drawer->circle_phase == 0) angular_acceleration = drawer->max_acceleration / drawer->radius;
    else if (drawer->circle_phase == 2) angular_acceleration = -drawer->max_acceleration / drawer->radius;

    drawer->revolute_trajectory_acceleration = -drawer->radius *
        (angular_acceleration * sinf(drawer->current_angle) +
         angular_velocity * angular_velocity * cosf(drawer->current_angle));

    drawer->prismatic_trajectory_acceleration = drawer->radius *
        (angular_acceleration * cosf(drawer->current_angle) -
         angular_velocity * angular_velocity * sinf(drawer->current_angle));

    // Set position and velocity setpoints
    revolute_tuning.position_setpoint = drawer->revolute_trajectory_position;
//    prismatic_tuning.position_setpoint = drawer->prismatic_trajectory_position;

    revolute_cascade.velocity_setpoint = drawer->revolute_trajectory_velocity;
//    prismatic_cascade.velocity_setpoint = drawer->prismatic_trajectory_velocity;


    drawer->revolute_position_error = drawer->revolute_trajectory_position - revolute_encoder.position;
    drawer->revolute_velocity_error = drawer->revolute_trajectory_velocity - revolute_encoder.speed;
//    drawer->prismatic_position_error = drawer->prismatic_trajectory_position - prismatic_encoder.position;
//    drawer->prismatic_velocity_error = drawer->prismatic_trajectory_velocity - prismatic_encoder.speed;
}

// Function to get the current trajectory values for sending to Node-RED
void SmoothCircleDrawer_GetTrajectoryValues(SmoothCircleDrawer* drawer,
                                          float* rev_pos, float* rev_vel, float* rev_acc,
                                          float* pris_pos, float* pris_vel, float* pris_acc) {
    *rev_pos = drawer->revolute_trajectory_position;
    *rev_vel = drawer->revolute_trajectory_velocity;
    *rev_acc = drawer->revolute_trajectory_acceleration;
    *pris_pos = drawer->prismatic_trajectory_position;
    *pris_vel = drawer->prismatic_trajectory_velocity;
    *pris_acc = drawer->prismatic_trajectory_acceleration;
}

// Function to get error values for monitoring
void SmoothCircleDrawer_GetErrors(SmoothCircleDrawer* drawer,
                                float* rev_pos_err, float* rev_vel_err,
                                float* pris_pos_err, float* pris_vel_err) {
    *rev_pos_err = drawer->revolute_position_error;
    *rev_vel_err = drawer->revolute_velocity_error;
    *pris_pos_err = drawer->prismatic_position_error;
    *pris_vel_err = drawer->prismatic_velocity_error;
}

void StartPrismaticOscillation(void) {
    prismatic_motion_active = true;
    current_target_index = 0;
    prismatic_tuning.position_setpoint = prismatic_target_positions[current_target_index];
    prismatic_cascade.velocity_setpoint = prismatic_max_velocity_normal;  // Set to normal max velocity
    last_target_change_time = HAL_GetTick();
}
void TestPrismaticOscillation(void) {
    prismatic_tuning.control_enabled = true;

    StartPrismaticOscillation();
}

void RotateRevoluteJoint(void) {
    static uint32_t start_time = 0;
    static int last_angle = -1;
    static uint8_t move_in_progress = 0;
    static uint32_t last_move_time = 0;

    if (start_time == 0) {
        start_time = HAL_GetTick();
        last_move_time = start_time;
    }

    uint32_t elapsed_ms = HAL_GetTick() - start_time;
    int current_angle = (elapsed_ms / 10) * 0.25;
    current_angle = current_angle % 360;

    uint32_t current_time = HAL_GetTick();

    if ((current_angle != last_angle) && !move_in_progress) {
        last_angle = current_angle;

        float angle_rad = current_angle * (M_PI / 180.0f);
        float angle_rad_pris =0;

        float next_revolute_position = -angle_rad;
        float next_prismatic_position = angle_rad_pris;

        Trapezoid_SetTarget(&revolute_trap, revolute_encoder.position, next_revolute_position);
        Trapezoid_SetTarget(&prismatic_trap, prismatic_encoder.position, next_prismatic_position);

        move_in_progress = 1;
        last_move_time = current_time;


    }

    if (move_in_progress) {
        Trapezoid_Update(&revolute_trap);
        Trapezoid_Update(&prismatic_trap);

        revolute_tuning.position_setpoint = revolute_trap.position;
//        prismatic_tuning.position_setpoint = prismatic_trap.position - revolute_trap.position;

        revolute_cascade.velocity_setpoint = revolute_trap.velocity;
        prismatic_cascade.velocity_setpoint = prismatic_trap.velocity - revolute_trap.velocity;

        if (Trapezoid_IsComplete(&revolute_trap) && Trapezoid_IsComplete(&prismatic_trap)) {
            move_in_progress = 0;

        }

        if (current_time - last_move_time > 5000) {  // 5 second timeout
            move_in_progress = 0;

        }
    }
}

void UpdatePrismaticTargets(void) {
    if (!prismatic_motion_active) {
        return;
    }

    float position_error = fabsf(prismatic_encoder.position - prismatic_target_positions[current_target_index]);
    uint32_t current_time = HAL_GetTick();

    if (position_error < position_tolerance &&
        (current_time - last_target_change_time) > min_dwell_time_ms) {

        current_target_index = (current_target_index + 1) % 2;
        prismatic_tuning.position_setpoint = prismatic_target_positions[current_target_index];
        last_target_change_time = current_time;
    }
}

void SetPrismaticTargetToZero(void) {
    prismatic_tuning.control_enabled = true;


    prismatic_tuning.position_setpoint = 0.0f;

    prismatic_cascade.velocity_setpoint = prismatic_max_velocity_normal;
}


void PrismaticControlLoop(void) {
    Encoder_Read(&prismatic_encoder);
    Encoder_CalculateSpeed(&prismatic_encoder);

    // Update oscillation targets if active
    UpdatePrismaticTargets();

    if (!prismatic_tuning.control_enabled) {
        SetPrismaticMotorPWM(0.0f, prismatic_tuning.max_voltage,
                prismatic_tuning.pwm_period);
        return;
    }

    // Select max velocity based on whether we're compensating
    float max_velocity = is_prismatic_compensating ?
                         prismatic_max_velocity_compensating :
                         prismatic_max_velocity_normal;

    float v_feedforward = PrismaticFeedforward(
            prismatic_tuning.position_setpoint,
            prismatic_cascade.velocity_setpoint, 0.0f,
            &prismatic_motor_params, &system_params);

    float v_disturbance = PrismaticDisturbanceComp(prismatic_encoder.position,
            &prismatic_disturbance, &prismatic_motor_params);

    // Use the selected max velocity limit here
    float v_feedback = CascadeController_Update(&prismatic_cascade,
            prismatic_tuning.position_setpoint, prismatic_encoder.position,
            prismatic_encoder.speed, max_velocity);  // Use selected max velocity

    float control_signal_output = v_feedback + v_feedforward + v_disturbance;
    SetPrismaticMotorPWM(control_signal_output, prismatic_tuning.max_velocity,
            prismatic_tuning.pwm_period);
}


void RevoluteControlLoop(void) {
    Encoder_Read(&revolute_encoder);
    Encoder_CalculateSpeed(&revolute_encoder);

    // Calculate revolute changes for compensation
    float revolute_position = revolute_encoder.position;
    float revolute_velocity = revolute_encoder.speed;
    float revolute_change = revolute_position - prev_revolute_position;
    prev_revolute_position = revolute_position;

    // If revolute moved significantly, adjust prismatic target
    if (fabsf(revolute_change) > 0.0001f) {
        float prismatic_change = -revolute_change;
        prismatic_tuning.position_setpoint += prismatic_change;
        prismatic_cascade.velocity_setpoint = -revolute_velocity;
        is_prismatic_compensating = true; // Flag that we're compensating
    } else {
        is_prismatic_compensating = false; // Not compensating
    }

    // Handle trajectory generation if a move is in progress
    if (move_in_progress) {
        Trapezoid_Update(&revolute_trap);
        if (Trapezoid_IsComplete(&revolute_trap)) {
            move_in_progress = 0;
        }
    }

    // Determine setpoints based on whether a move is in progress
    float position_setpoint;
    float velocity_setpoint;
    float acceleration_setpoint;

    if (move_in_progress) {
        // Using trapezoid profile
        position_setpoint = revolute_trap.position;
        velocity_setpoint = revolute_trap.velocity;
        acceleration_setpoint = revolute_trap.acceleration;
    } else {
        // Using direct setpoints
        position_setpoint = revolute_tuning.position_setpoint;
        velocity_setpoint = revolute_cascade.velocity_setpoint;
        acceleration_setpoint = 0.0f;
    }

    // Get current control signal and estimate state using Kalman filter
    float control_signal = revolute_cascade.current_setpoint;
    MotorModelKalman_Estimate(&revolute_motor_filter, control_signal,
            revolute_encoder.position);

    // Skip control if disabled
    if (!revolute_tuning.control_enabled) {
        SetRevoluteMotorPWM(0.0f, revolute_tuning.max_voltage,
                revolute_tuning.pwm_period);
        return;
    }

    // Apply filtering and phase lead compensation to velocity
    float filtered_velocity = revolute_motor_filter.velocity;
    float compensated_velocity = ApplyPhaseLead(filtered_velocity, 0.5f, 0.6f);

    // Calculate control components
    float v_feedback = CascadeController_Update(&revolute_cascade,
            position_setpoint, revolute_encoder.position,
            revolute_motor_filter.velocity, 1.0f);

    float v_feedforward = RevoluteFeedforward(position_setpoint,
            velocity_setpoint, acceleration_setpoint, &revolute_motor_params,
            &system_params);

    float v_disturbance = RevoluteDisturbanceComp(revolute_encoder.position,
            &revolute_disturbance, &revolute_motor_params);

    // Sum control components and apply to motor
    float control_signal_output = v_feedback + v_feedforward + v_disturbance;
    SetRevoluteMotorPWM(control_signal_output, revolute_tuning.max_voltage,
            revolute_tuning.pwm_period);

    // Store current position and velocity for next iteration
    latest_revolute_position = revolute_encoder.position;
    latest_revolute_velocity = revolute_encoder.speed;
}




void PointTraversal_Init(PointTraversal *traversal, WorkspacePoint *points,
		int num_points, float max_velocity, float max_acceleration) {
	traversal->points = points;
	traversal->num_points = num_points;
	traversal->current_point = 0;
	traversal->max_velocity = max_velocity;
	traversal->max_acceleration = max_acceleration;
	traversal->active = 0;
	traversal->point_reached = 1;  // Start ready to move to first point

	// Initialize trajectory generators
	Trapezoid_Init(&traversal->revolute_trajectory, max_velocity,
			max_acceleration);
	Trapezoid_Init(&traversal->prismatic_trajectory, max_velocity,
			max_acceleration);
}

// Start the point traversal test
void PointTraversal_Start(PointTraversal *traversal) {
	traversal->current_point = 0;
	traversal->active = 1;
	traversal->point_reached = 1;  // Ready to move to first point
}

void PointTraversal_Update(PointTraversal *traversal) {
    if (!traversal->active)
        return;

    static uint32_t point_start_time = 0;

    if (point_start_time == 0) {
        point_start_time = HAL_GetTick();
        revolute_tuning.position_setpoint = traversal->points[traversal->current_point].revolute_pos;
        prismatic_tuning.position_setpoint = traversal->points[traversal->current_point].prismatic_pos;
    }

    if (HAL_GetTick() - point_start_time > 2000) {
        traversal->current_point++;
        point_start_time = HAL_GetTick();

        if (traversal->current_point >= traversal->num_points) {
            traversal->active = 0;
            traversal->current_point = 0;
            return;
        }

        revolute_tuning.position_setpoint = traversal->points[traversal->current_point].revolute_pos;
        prismatic_tuning.position_setpoint = traversal->points[traversal->current_point].prismatic_pos;


    }
}

// Test function to create a 10-point test in the workspace
void CreateWorkspaceTestPoints(WorkspacePoint *points, int num_points) {
	// Ensure we have at least 10 points
	if (num_points < 10)
		return;

	// Create test points that cover the workspace
	// These are examples - adjust based on your actual workspace

	// Point 1: Center of workspace
	points[0].revolute_pos = 0.0f;
	points[0].prismatic_pos = 0.0f;

	// Point 2: Right side (3 o'clock)
	points[1].revolute_pos = 0.3f;
	points[1].prismatic_pos = 0.0f;

	// Point 3: Upper right (1:30)
	points[2].revolute_pos = 0.22f;
	points[2].prismatic_pos = 0.22f;

	// Point 4: Top (12 o'clock)
	points[3].revolute_pos = 0.0f;
	points[3].prismatic_pos = 0.3f;

	// Point 5: Upper left (10:30)
	points[4].revolute_pos = -0.22f;
	points[4].prismatic_pos = 0.22f;

	// Point 6: Left side (9 o'clock)
	points[5].revolute_pos = -0.3f;
	points[5].prismatic_pos = 0.0f;

	// Point 7: Lower left (7:30)
	points[6].revolute_pos = -0.22f;
	points[6].prismatic_pos = -0.22f;

	// Point 8: Bottom (6 o'clock)
	points[7].revolute_pos = 0.0f;
	points[7].prismatic_pos = -0.3f;

	// Point 9: Lower right (4:30)
	points[8].revolute_pos = 0.22f;
	points[8].prismatic_pos = -0.22f;

	// Point 10: Back to center
	points[9].revolute_pos = 0.0f;
	points[9].prismatic_pos = 0.0f;
}

// Function to run the workspace test
void RunWorkspaceTest(void) {
	// Create an array of points
	WorkspacePoint test_points[10];

	// Initialize the test points
	CreateWorkspaceTestPoints(test_points, 10);

	// Create traversal tester
	PointTraversal tester;
	PointTraversal_Init(&tester, test_points, 10, 0.5f, 1.0f);

	// Start the test
	PointTraversal_Start(&tester);

	// Run the test in your main loop or timer
	while (tester.active) {
		PointTraversal_Update(&tester);

		// Run control loops
		RevoluteControlLoop();
		PrismaticControlLoop();

		// Small delay
		HAL_Delay(10);
	}

	// Test complete
	// You could add code here to log results or display success
}
//
//void MotorControlUpdate(void) {
//    RevoluteControlLoop();
//    PrismaticControlLoop();
//}


// Add this function to your code
void GenerateSimpleSinusoidalTrajectory(void) {
    static uint32_t start_time = 0;

    if (start_time == 0) {
        start_time = HAL_GetTick();
    }

    uint32_t elapsed_ms = HAL_GetTick() - start_time;
    float elapsed_s = elapsed_ms / 1000.0f;

    float angular_freq = 0.1f * 2.0f * M_PI;
    float amplitude = 0.3f;  // 0.3 radians amplitude

    revolute_tuning.position_setpoint = amplitude * sinf(angular_freq * elapsed_s);
    prismatic_tuning.position_setpoint = amplitude * cosf(angular_freq * elapsed_s);

    revolute_cascade.velocity_setpoint = amplitude * angular_freq *
                                        cosf(angular_freq * elapsed_s);
    prismatic_cascade.velocity_setpoint = -amplitude * angular_freq *
                                         sinf(angular_freq * elapsed_s);
}


//void RotateRevoluteJoint(void) {
//    static uint32_t start_time = 0;
//    static int last_angle = -1;
//
//    if (start_time == 0) start_time = HAL_GetTick();
//
//    uint32_t elapsed_ms = HAL_GetTick() - start_time;
//    int current_angle = (elapsed_ms / 10) * 0.5;
//    current_angle = current_angle % 360;
//
//    if (current_angle != last_angle) {
//        last_angle = current_angle;
//
//        float angle_rad = current_angle * (M_PI / 180.0f);
//
//        revolute_tuning.position_setpoint = -angle_rad;
//        prismatic_tuning.position_setpoint = angle_rad;
//
//
//    }
//}

void SmoothCircularMotion(void) {
    static uint32_t start_time = 0;
    static float prev_rev_pos = 0.0f;
    static float prev_pris_pos = 0.0f;

    // Initialize on first call
    if (start_time == 0) {
        start_time = HAL_GetTick();
        prev_rev_pos = revolute_encoder.position;
        prev_pris_pos = prismatic_encoder.position;
    }

    float current_time_sec = (HAL_GetTick() - start_time) / 1000.0f;

    float circle_radius = 0.3f;      // Size of circle in radians
    float circle_period = 20.0f;     // Time for one complete revolution (seconds)
    float angular_velocity = 2.0f * M_PI / circle_period;  // rad/s
    float current_angle = angular_velocity * current_time_sec;

    float target_revolute_pos = circle_radius * cosf(current_angle);
    float target_prismatic_pos = circle_radius * sinf(current_angle);

    float target_revolute_vel = -circle_radius * angular_velocity * sinf(current_angle);
    float target_prismatic_vel = circle_radius * angular_velocity * cosf(current_angle);

    float target_revolute_acc = -circle_radius * angular_velocity * angular_velocity * cosf(current_angle);
    float target_prismatic_acc = -circle_radius * angular_velocity * angular_velocity * sinf(current_angle);


    float blend_factor = 0.03f;  // Lower = smoother but more delay (0.01-0.05 is good)

    revolute_tuning.position_setpoint = prev_rev_pos * (1.0f - blend_factor) +
                                        target_revolute_pos * blend_factor;
    prismatic_tuning.position_setpoint = prev_pris_pos * (1.0f - blend_factor) +
                                         target_prismatic_pos * blend_factor;

    revolute_cascade.velocity_setpoint = target_revolute_vel;
    prismatic_cascade.velocity_setpoint = target_prismatic_vel;

    prev_rev_pos = revolute_tuning.position_setpoint;
    prev_pris_pos = prismatic_tuning.position_setpoint;



}


/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */
	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */
	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_ADC3_Init();
	MX_I2C1_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_USART1_UART_Init();
	MX_TIM5_Init();
	MX_TIM17_Init();
	MX_TIM4_Init();
	MX_TIM15_Init();
	/* USER CODE BEGIN 2 */

	InitializeRevoluteTuning();
	InitializePrismaticTuning();

	EnableControl();

	//	UpdateTuningParameters(5.0f, 0.000000000000000000000000000f, 0.000000f, 0.8475f,0.00625f, 0.0f); // rever

	//	UpdateTuningParameters(5.575f, 0.0455f, 2.5f, 0.000001f, 1.1f, 0.0f);
	//	UpdateTuningParameters(0.0f, 0.0f, 0.0f, 0.3f, 20.0f, 0.0f);

//		UpdateTuningParameters(25.575f, 0.0755f, 2.5f, 0.75f, 0.015f, 0.0f);  //pris

	UpdateRevoluteTuningParameters(5.0f, 0.000000000000000000000000000f,
			0.000000f, 0.8475f, 0.00625f, 0.0f);
	UpdatePrismaticTuningParameters(25.575f, 0.0755f, 2.5f, 0.75f, 0.015f,
			0.0f);
//    revolute_tuning.position_setpoint = 3.14;
//	DrawCircle_Start(50.0f, 50.0f);

	WorkspacePoint custom_points[10] = {
	    {0.3f, 0.0f},         // 0°
	    {0.27f, 0.13f},       // 36°
	    {0.15f, 0.26f},       // 72°
	    {0.0f, 0.3f},         // 108°
	    {-0.15f, 0.26f},      // 144°
	    {-0.27f, 0.13f},      // 180°
	    {-0.3f, 0.0f},        // 216°
	    {-0.27f, -0.13f},     // 252°
	    {-0.15f, -0.26f},     // 288°
	    {0.0f, -0.3f}         // 324°
	};



	PointTraversal_Init(&custom_test, custom_points, 10, 1.5f, 2.0f);
	PointTraversal_Start(&custom_test);
	SetPrismaticTargetToZero();
//	SmoothCircleDrawer_Init(&circle, 0.0f, 0.0f, 0.3f, 1.5f, 1.0f);
//
//	// Start drawing when needed
//	SmoothCircleDrawer_Start(&circle);

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); // Revolute joint
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // Prismatic joint
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);

	HAL_TIM_Base_Start_IT(&htim17);
	HAL_TIM_Base_Start_IT(&htim4);

//  UpdateTuningParameters(15.0f, 0.0025f, 1.0f, 0.4f,70.125f, 0.0f);
//  StartStepTest(M_PI / 4.0f);

	//		arm_mat_init_f32(&IP_KCPI_KC_trans_KRKT, 4, 4, IP_KCPI_KC_trans_KRKT_f32);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
//	  static uint32_t current_Hz = 0;
//      if (HAL_GetTick() - current_Hz > 1) {  // Running at 500 Hz
//          current_Hz = HAL_GetTick();
//
//          // Toggle between filters every 5 seconds
//          if (HAL_GetTick() - filter_switch_time > 3000) {
//              use_motor_model = !use_motor_model;
//              filter_switch_time = HAL_GetTick();
//
//              // Flash LED or set some debug signal
//              HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
//          }
//
//          TestVelocityBidirectional();
//
//          // Run the main control loop
//          ControlLoop();
//
////          setPWMToPrismatic(encoder.position, 19999, 24.0f);
//      }
////rev
//	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 1);
//
//	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 500);
//  }
//  ControlLoop();
//  HAL_Delay(2); 500 HZ revo
//	  static uint32_t current_Hz = 0;
//	  if (HAL_GetTick() - current_Hz > 0.001) {
//		  current_Hz = HAL_GetTick();
//////		  TestPositionStep();
////		   TestPositionStep();
//
//		  revolute_tuning.position_setpoint = position_setpoint_deg * (M_PI / 180.0f);
////		  TestVelocityBidirectional();
////		  TrapezoidalVelocityProfile();
//		  RevoluteControlLoop();
//
//	  }
		//trajec
//	  static uint32_t last_control_time = 0;
//	     uint32_t current_time = HAL_GetTick();
//
//	     // Run control loop at 1kHz (every 1ms)
//	     if (current_time - last_control_time >= 1) {
//	         last_control_time = current_time;
//
//	         // Run the control loop
//	         ControlLoop();
//
//	         // Optional: If you want to run a back-and-forth test automatically
//	         static uint32_t last_move_time = 0;
//	         static uint8_t position_index = 0;
//
//	         // If currently not moving and it's been 3 seconds since last move
//	         if (!move_in_progress && current_time - last_move_time > 3000) {
//	             position_index = (position_index + 1) % 2;
//	             float next_position = (position_index == 0) ? 0 : M_PI/2.0f;  // Alternate between 0 and 90 degrees
//
//	             // Start the next movement
//	             Trapezoid_SetTarget(&trap, encoder.position, next_position);
//	             move_in_progress = 1;
//	             last_move_time = current_time;
//	         }
//	     }

		// In your main loop:
//		static uint32_t last_control_time = 0;
//		uint32_t current_time = HAL_GetTick();
//
//		if (current_time - last_control_time >= 1) {
//		    last_control_time = current_time;
//
//
////		        SmoothCircleDrawer_Update(&circle);
//
//
//		    RotateRevoluteJoint();
//
////		    GenerateSimpleSinusoidalTrajectory();
//
//		    RevoluteControlLoop();
//		    PrismaticControlLoop();
//
//
//		}

//	  static uint32_t last_control_time = 0;
//	      uint32_t current_time = HAL_GetTick();
//
//	      if (current_time - last_control_time >= 1) {
//	          last_control_time = current_time;
//
//	          TestRevolutePositionStep();
//
////	          RevoluteControlLoop();
//	          MotorControlUpdate();
////	          PrismaticControlLoop();
//
//	      }
////	  TestPositionStep();
////	  ControlLoop();
//
////	  TestVelocityBidirectional();
////	  		   ControlLoop();
////	  HAL_Delay(2);
//
////  TestPositionStep();
	}

	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
	RCC_OscInitStruct.PLL.PLLN = 85;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */
	/* USER CODE END ADC1_Init 0 */

	ADC_MultiModeTypeDef multimode = { 0 };
	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */
	/* USER CODE END ADC1_Init 1 */

	/** Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.GainCompensation = 0;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc1.Init.OversamplingMode = DISABLE;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure the ADC multi-mode
	 */
	multimode.Mode = ADC_MODE_INDEPENDENT;
	if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_9;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */
	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief ADC3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC3_Init(void) {

	/* USER CODE BEGIN ADC3_Init 0 */
	/* USER CODE END ADC3_Init 0 */

	ADC_MultiModeTypeDef multimode = { 0 };
	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC3_Init 1 */
	/* USER CODE END ADC3_Init 1 */

	/** Common config
	 */
	hadc3.Instance = ADC3;
	hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc3.Init.Resolution = ADC_RESOLUTION_12B;
	hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc3.Init.GainCompensation = 0;
	hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc3.Init.LowPowerAutoWait = DISABLE;
	hadc3.Init.ContinuousConvMode = DISABLE;
	hadc3.Init.NbrOfConversion = 1;
	hadc3.Init.DiscontinuousConvMode = DISABLE;
	hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc3.Init.DMAContinuousRequests = DISABLE;
	hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc3.Init.OversamplingMode = DISABLE;
	if (HAL_ADC_Init(&hadc3) != HAL_OK) {
		Error_Handler();
	}

	/** Configure the ADC multi-mode
	 */
	multimode.Mode = ADC_MODE_INDEPENDENT;
	if (HAL_ADCEx_MultiModeConfigChannel(&hadc3, &multimode) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_5;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC3_Init 2 */
	/* USER CODE END ADC3_Init 2 */

}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */
	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */
	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x40B285C2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE)
			!= HAL_OK) {
		Error_Handler();
	}

	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */
	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */
	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */
	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 169;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 999;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 2000;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.Pulse = 0;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.BreakFilter = 0;
	sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
	sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
	sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
	sBreakDeadTimeConfig.Break2Filter = 0;
	sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */
	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */
	/* USER CODE END TIM2_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */
	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 65535;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 5;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */
	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */
	/* USER CODE END TIM3_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */
	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 65535;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 5;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */
	/* USER CODE END TIM3_Init 2 */

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 169;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 999;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */

}

/**
 * @brief TIM5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM5_Init(void) {

	/* USER CODE BEGIN TIM5_Init 0 */
	/* USER CODE END TIM5_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM5_Init 1 */
	/* USER CODE END TIM5_Init 1 */
	htim5.Instance = TIM5;
	htim5.Init.Prescaler = 1699;
	htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim5.Init.Period = 1999;
	htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim5) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim5) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM5_Init 2 */
	/* USER CODE END TIM5_Init 2 */
	HAL_TIM_MspPostInit(&htim5);

}

/**
 * @brief TIM15 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM15_Init(void) {

	/* USER CODE BEGIN TIM15_Init 0 */

	/* USER CODE END TIM15_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM15_Init 1 */

	/* USER CODE END TIM15_Init 1 */
	htim15.Instance = TIM15;
	htim15.Init.Prescaler = 0;
	htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim15.Init.Period = 65535;
	htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim15.Init.RepetitionCounter = 0;
	htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim15) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM15_Init 2 */

	/* USER CODE END TIM15_Init 2 */

}

/**
 * @brief TIM17 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM17_Init(void) {

	/* USER CODE BEGIN TIM17_Init 0 */
	/* USER CODE END TIM17_Init 0 */

	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM17_Init 1 */
	/* USER CODE END TIM17_Init 1 */
	htim17.Instance = TIM17;
	htim17.Init.Prescaler = 169;
	htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim17.Init.Period = 999;
	htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim17.Init.RepetitionCounter = 0;
	htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim17) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_OC_Init(&htim17) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_OnePulse_Init(&htim17, TIM_OPMODE_SINGLE) != HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_ACTIVE;
	sConfigOC.Pulse = 1433;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_OC_ConfigChannel(&htim17, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.BreakFilter = 0;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim17, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM17_Init 2 */
	/* USER CODE END TIM17_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */
	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */
	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 19200;
	huart1.Init.WordLength = UART_WORDLENGTH_9B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_EVEN;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */
	/* USER CODE END USART1_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMAMUX1_CLK_ENABLE();
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);

	/*Configure GPIO pin : PC13 */
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : LPUART1_TX_Pin LPUART1_RX_Pin */
	GPIO_InitStruct.Pin = LPUART1_TX_Pin | LPUART1_RX_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF12_LPUART1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PA4 PA5 PA15 */
	GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PB1 */
	GPIO_InitStruct.Pin = GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : PB2 PB10 PB11 PB4
	 PB5 */
	GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_4
			| GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : PC7 */
	GPIO_InitStruct.Pin = GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PA9 PA10 */
	GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PA12 */
	GPIO_InitStruct.Pin = GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// Timer 1 interrupt callback
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
//	InitializeRevoluteTuning();
//	    InitializePrismaticTuning();
//	    EnableControl();
//	    UpdateRevoluteTuningParameters(5.0f, 0.00000000000001f, 0.1f, 0.5f,42.0f, 0.0f);
//	    UpdatePrismaticTuningParameters(5.575f, 0.0455f, 2.5f, 0.000001f, 1.1f, 0.0f);
//	revolute_tuning.position_setpoint = 3.14;
//	revolute_cascade.velocity_setpoint  = 1.0f;
	if (htim == &htim17) {
//    	TestRevolutePositionStep();
//    	test += 1;
//        RevoluteControlLoop();
//        PrismaticControlLoop();

	}
	if (htim == &htim4) {

//    	test += 1;
//		revolute_tuning.position_setpoint = position_setpoint_deg
//				* (M_PI / 180.0f);

		RotateRevoluteJoint();


//    	TestRevolutePositionStep();
//    	TrapezoidalVelocityProfile();

//		RotateRevoluteJoint();
		RevoluteControlLoop();
		PrismaticControlLoop();

	}

}
//
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
//	if (htim == &htim4) {
//
//		while (circle.active) {
//// Update the circle drawing
////			SmoothCircleDrawer_Update(&circle);
//
//// Run your control loops
//			RevoluteControlLoop();
//			PrismaticControlLoop();
//		}
//	}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
