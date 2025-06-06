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
#include <stdint.h>

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
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
// Prismatic control variables
uint32_t main_servo = 0;
float PWM_set = 0;
float position_setpoint_deg = 0.0f;
static uint32_t motion_start_time = 0;
static float current_velocity_setpoint = 0.0f;
static uint8_t motion_complete = 0;
static uint8_t motion_direction = 1;
float control_signal_output;
bool is_prismatic_compensating = false;
float prismatic_max_velocity_normal = 40.0f;  // rad/s for normal operation
float prismatic_max_velocity_compensating = 1.0f;  // rad/s during compensation

float prismatic_raw_setpoint = 0.0f; // The user-set target, without compensation
float prismatic_compensation_offset = 0.0f;  // Current compensation offset
float prismatic_target_positions[2] = { 0.1f, -0.1f }; // Target positions in radians
int current_target_index = 0;                         // Current target index
float position_tolerance = 0.05f;                // Position tolerance (radians)
bool prismatic_motion_active = false;                 // Motion activation flag
uint32_t last_target_change_time = 0;              // Time of last target change
uint32_t min_dwell_time_ms = 2000;             // Minimum time to wait at target

// Trajectory variables
float move_in_progress = 0;
uint8_t move_index = 0;

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
	float revolute_pos;
	float prismatic_pos;
} CoordPosition;
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
	PIDController position_pid;
//	VelocityPID position_pid;     // Position loop PID (outer loop)
	VelocityPID velocity_pid;     // Velocity loop PID (inner loop)
	VelocityPID current_pid;      // Optional current loop PID

	float position_setpoint;      // Position setpoint
	float velocity_setpoint;      // Velocity setpoint
	float current_setpoint;       // Current setpoint

	float velocity_output;        // Output from position controller
	float current_output;         // Output from velocity controller
	float max_velocity;

	uint8_t control_enabled;      // Enable/disable control

	// Timing variables for cascade control
	uint32_t last_position_update_time;  // Last time position loop ran (500Hz)
	uint32_t last_velocity_update_time;  // Last time velocity loop ran (1000Hz)

	// Loop frequencies
	uint32_t position_loop_period_ms;    // Position loop period (2ms for 500Hz)
	uint32_t velocity_loop_period_ms;   // Velocity loop period (1ms for 1000Hz)
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

typedef struct {
	float revolute_angle;    // Revolute joint angle (radians)
	float prismatic_pos; // Prismatic joint position (radians, matching your system)
} JointPosition;

typedef struct {
	float x;  // Cartesian X coordinate (meters)
	float y;  // Cartesian Y coordinate (meters)
} CartesianPosition;

typedef struct {
	float r;     // Polar radius (meters)
	float theta; // Polar angle (radians)
} PolarPosition;

// Coupling compensation coefficients (based on your experimental data)
typedef struct {
	float coupling_coeff;      // Coupling coefficient between axes
	float compensation_gain;   // Compensation gain
	float linearization_offset; // Linearization offset
} CouplingParams;

// Integration with your trajectory system
typedef struct {
	float revolute_target;     // Target revolute position
	float prismatic_target;    // Target prismatic position
	float sync_time;           // Synchronized movement time
	uint8_t move_active;       // Movement status
	uint32_t start_time;       // Movement start time
} CoordinatedMove;

// Initialize coupling parameters based on your experimental data
static CouplingParams coupling_params = { .coupling_coeff = 1.0f, // Direct coupling (revolute affects prismatic 1:1)
		.compensation_gain = 1.0f,   // Full compensation
		.linearization_offset = 0.0f // No offset by default
		};

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

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// System parameters matching your STM32 implementation
#define BELT_CIRCUMFERENCE 0.08f  // 80mm belt circumference
#define WORKSPACE_RADIUS 0.3f     // 600mm diameter -> 300mm radius
#define MAX_PRISMATIC_RANGE 0.3f  // Maximum prismatic extension
#define COUPLING_RATIO (2.0f * M_PI / BELT_CIRCUMFERENCE)  // rad/m conversion
#define RAD_PER_DEGREE 0.0174533f // From your STM32 code

#define BELT_CONVERSION_FACTOR (0.08f / (2.0f * M_PI))
#define REVOLUTE_TOLERANCE 0.002f   // ±0.2° in radians (bonus: ±0.1°)
#define PRISMATIC_TOLERANCE 0.0002f // ±0.2mm (bonus: ±0.1mm)

// Integration with your existing trajectory system
#define TRAJECTORY_MAX_VEL_REVOLUTE 1.0f    // From your code
#define TRAJECTORY_MAX_ACCEL_REVOLUTE 0.4f  // From your code
#define TRAJECTORY_MAX_VEL_PRISMATIC 40.0f  // From your code
#define TRAJECTORY_MAX_ACCEL_PRISMATIC 20.0f // From your code
/* Number of states in DC motor model */
#define MOTOR_MODEL_NUM_STATES 4
#define MOTOR_MODEL_NUM_INPUTS 1
#define MOTOR_MODEL_NUM_OUTPUTS 1

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
	float center_revolute;  // Circle center (revolute axis position in radians)
	float center_prismatic; // Circle center (prismatic axis position in radians)
	float radius;             // Circle radius in radians
	float max_velocity;       // Maximum velocity during drawing (rad/s)
	float max_acceleration;   // Maximum acceleration (rad/s²)
	float segment_angle;      // Segment size in radians
	float current_angle;      // Current angle in the circle traversal
	float target_angle;      // Target ending angle (usually 2π for full circle)
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
typedef enum {
	Previous = 0, Current = 1
} TimeIndex;

// Trajectory profile types
typedef enum {
	PROFILE_TRIANGULAR = 0, PROFILE_TRAPEZOIDAL = 1
} ProfileType;

// Trajectory phase enumeration
typedef enum {
	PHASE_IDLE = 0,
	PHASE_ACCELERATION = 1,
	PHASE_CONSTANT_VELOCITY = 2,
	PHASE_DECELERATION = 3,
	PHASE_COMPLETE = 4
} TrajectoryPhase;

// Enhanced trajectory structure
typedef struct {
	// Input parameters
	float Init_pos;          // Initial position (θ₀)
	float Final_pos;         // Final position (θf)
	float delta_pos;         // Total distance to travel |θf - θ₀|
	float dir;               // Direction (1 or -1)
	float desire_av;         // Desired maximum velocity (ωmax)
	float desire_ac;         // Desired acceleration (αmax)

	// Profile parameters
	ProfileType profile_type; // TRIANGULAR or TRAPEZOIDAL
	float t_acc;             // Acceleration time (tacc)
	float t_const;           // Constant velocity time (tconst)
	float t_dec;             // Deceleration time (tdec)
	float t_total;           // Total trajectory time (ttotal)

	// Distance calculations
	float s_acc;             // Distance during acceleration
	float s_const;           // Distance during constant velocity
	float s_dec;             // Distance during deceleration

	// Real-time outputs
	float ang_pos;           // Current position (θ)
	float ang_velo;          // Current velocity (ω)
	float ang_acc;           // Current acceleration (α)

	// Timing
	uint64_t t[2];           // Time array [Previous, Current] in microseconds
	float t_elapsed;         // Time since trajectory start (seconds)

	// Status
	uint8_t active;          // Trajectory active flag
	TrajectoryPhase phase;   // Current trajectory phase

	// Peak velocity achieved (for triangular profiles)
	float peak_velocity;     // Actual peak velocity reached
} Trajectory;

// Global trajectory instances
Trajectory revolute_trajectory;
Trajectory prismatic_trajectory;

SmoothCircleDrawer circle;
PointTraversal custom_test;
SimpleTrapezoid revolute_trap;
SimpleTrapezoid prismatic_trap;
uint8_t test = 0;
volatile float latest_revolute_position = 0.0f;
volatile float latest_revolute_velocity = 0.0f;

#define GCODE_MIN_X 0.0f
#define GCODE_MAX_X 24.94401f     // Maximum X coordinate in G-code
#define GCODE_MIN_Y 0.0f
#define GCODE_MAX_Y 5.0f          // Maximum Y coordinate in G-code

// Your robot workspace (from previous specifications)
#define ROBOT_WORKSPACE_RADIUS 0.3f    // 30cm radius
#define ROBOT_SAFE_AREA 0.25f          // Use 25cm for safety margin

// Scaling factors to fit G-code into robot workspace
#define SCALE_FACTOR 0.008f            // Scale down to fit workspace
#define OFFSET_X 0.05f                 // Center offset X
#define OFFSET_Y 0.15f                 // Center offset Y (15cm forward)

typedef enum {
	PEN_UP = 0, PEN_DOWN = 1
} PenState;

// Structure for robot writing commands
typedef struct {
	float x;                // Cartesian X coordinate (meters)
	float y;                // Cartesian Y coordinate (meters)
	PenState pen_state;     // Pen up/down state
	uint8_t is_move_only; // True for G00 (rapid move), False for G01 (linear move)
} RobotWriteCommand;

// Optimization structures
typedef struct {
	CartesianPosition pos;
	uint8_t is_corner;      // Mark sharp corners for slower movement
	float speed_factor;     // 0.1 to 1.0 speed multiplier
} OptimizedPoint;

typedef struct {
	CartesianPosition positions[5]; // Look ahead 5 points
	uint8_t count;
	uint8_t index;
} LookAheadBuffer;

typedef struct {
	uint8_t fast_mode_active;
	float position_tolerance;
	float max_step_size;
	uint32_t update_rate_ms;
} FastDrawingMode;

// Global variables for drawing optimization
FastDrawingMode drawing_mode = { .fast_mode_active = 0, .position_tolerance =
		0.0008f,  // 0.8mm tolerance
		.max_step_size = 0.003f,        // 3mm max step
		.update_rate_ms = 5             // 200Hz base update rate
		};

LookAheadBuffer lookahead_buffer = { 0 };
OptimizedPoint optimized_points[300];
uint8_t drawing_initialized = 0;
JointPosition joints;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
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
static void MX_TIM8_Init(void);
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

void MoveToCoordinatedPositionSynced(float revolute_target,
		float prismatic_target);
float CalculateTrajectoryTime(float distance, float max_vel, float max_accel);

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

//	float max_integral = pid->output_max / pid->ki;
//	float min_integral = pid->output_min / pid->ki;
//
//	if (pid->error_integral > max_integral) {
//		pid->error_integral = max_integral;
//	}
//	if (pid->error_integral < min_integral) {
//		pid->error_integral = min_integral;
//	}

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

//	if ( abs(joints.revolute_angle-actual)  < 0.05f) {
//		return output;
//	}

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
	PID_Init(&controller->position_pid, 5.0f, 0.5f, 0.0f, -50.0f, 50.0f,
			0.002f);  // 2ms sample time (500Hz)

	VelocityPID_Init(&controller->velocity_pid, 2.0f, 0.2f, 0.05f, 0.001f); // 1ms sample time (1000Hz)

	VelocityPID_Init(&controller->current_pid, 10.0f, 1.0f, 0.0f, 0.001f); // 1ms sample time (1000Hz)

	controller->position_setpoint = 0.0f;
	controller->velocity_setpoint = 0.0f;
	controller->current_setpoint = 0.0f;

	controller->control_enabled = 1;

	controller->last_position_update_time = HAL_GetTick();
	controller->last_velocity_update_time = HAL_GetTick();

	controller->position_loop_period_ms = 2;  // 500Hz = 2ms period
	controller->velocity_loop_period_ms = 1;  // 1000Hz = 1ms period
}

float CascadeController_Update(CascadeController *controller,
		float position_setpoint, float position_feedback,
		float velocity_feedback, float max_velocity_limit,
		float tra_velocity_setpoint) {

	if (!controller->control_enabled) {
		return 0.0f;
	}

	uint32_t current_time = HAL_GetTick();

	if ((current_time - controller->last_position_update_time)
			>= controller->velocity_loop_period_ms) {

		// Update position controller (outer loop) using PID_Update
		float new_velocity_setpoint = PID_Update(&controller->position_pid,
				position_setpoint, position_feedback);

		// Apply velocity limits
		if (new_velocity_setpoint > max_velocity_limit) {
			new_velocity_setpoint = max_velocity_limit;
		} else if (new_velocity_setpoint < -max_velocity_limit) {
			new_velocity_setpoint = -max_velocity_limit;
		}

		controller->velocity_setpoint = new_velocity_setpoint
				+ tra_velocity_setpoint;

		controller->last_position_update_time = current_time;
	}

	if ((current_time - controller->last_velocity_update_time)
			>= controller->velocity_loop_period_ms) {

		static float filtered_velocity = 0.0f;
//		filtered_velocity = 0.3f * velocity_feedback + 0.7f * filtered_velocity;
		controller->current_setpoint = VelocityPID_Update(
				&controller->velocity_pid, controller->velocity_setpoint,
				velocity_feedback);

		// Apply current limits
		if (controller->current_setpoint > 24.0f) {
			controller->current_setpoint = 24.0f;
		}
		if (controller->current_setpoint < -24.0f) {
			controller->current_setpoint = -24.0f;
		}

		controller->velocity_pid.prev_output = controller->current_setpoint;

		// Update timing
		controller->last_velocity_update_time = current_time;
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

	// Calculate position based on encoder type
	if (enc == &prismatic_encoder) {
		enc->position = (float) enc->count / ENCODER_CPR * 2.0f * M_PI
				/ GEAR_RATIO;
		enc->position -= revolute_encoder.position; // Coupling compensation
	} else {
		enc->position = (float) enc->count / ENCODER_CPR * 2.0f * M_PI
				/ GEAR_RATIO;
	}
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

	// Initialize cascade controller with mixed PID types
	CascadeController_Init(&revolute_cascade, 0.001f);
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

	// Initialize cascade controller with mixed PID types
	CascadeController_Init(&prismatic_cascade, 0.001f);
	prismatic_cascade.max_velocity = prismatic_tuning.max_velocity;

	Encoder_Init(&prismatic_encoder, &htim3);

	float Q_prismatic = 0.1f;  // ลดลง → เชื่อใจ model มากขึ้น
	float R_prismatic = 0.4675f;   // เพิ่มขึ้น → เชื่อใจ measurement น้อยลง

	MotorModelKalman_Init(&prismatic_motor_filter, 0.001f,
			prismatic_motor_params.J, prismatic_motor_params.B,
			prismatic_motor_params.Kt, prismatic_motor_params.Ke,
			prismatic_motor_params.R, prismatic_motor_params.L, Q_prismatic,
			R_prismatic);

	Trapezoid_Init(&prismatic_trap, 1.0f, 4.0f);
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
	if (filtered_ff >= 2.0) {
		filtered_ff = 2.0;
	} else if (filtered_ff < 2.0) {
		filtered_ff = filtered_ff;
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
	// Reset PIDController states (position loops)
	revolute_cascade.position_pid.error_integral = 0.0f;
	revolute_cascade.position_pid.error_prev = 0.0f;
	revolute_cascade.position_pid.last_derivative = 0.0f;

	prismatic_cascade.position_pid.error_integral = 0.0f;
	prismatic_cascade.position_pid.error_prev = 0.0f;
	prismatic_cascade.position_pid.last_derivative = 0.0f;

	// Reset VelocityPID states (velocity loops)
	revolute_cascade.velocity_pid.prev_error = 0.0f;
	revolute_cascade.velocity_pid.prev_prev_error = 0.0f;
	revolute_cascade.velocity_pid.prev_output = 0.0f;

	prismatic_cascade.velocity_pid.prev_error = 0.0f;
	prismatic_cascade.velocity_pid.prev_prev_error = 0.0f;
	prismatic_cascade.velocity_pid.prev_output = 0.0f;

	// Set initial setpoints to current positions
	revolute_tuning.position_setpoint = revolute_encoder.position;
	prismatic_tuning.position_setpoint = prismatic_encoder.position;

	// Enable control for both joints
	revolute_tuning.control_enabled = 1;
	prismatic_tuning.control_enabled = 1;

	// Enable cascade controllers
	revolute_cascade.control_enabled = 1;
	prismatic_cascade.control_enabled = 1;
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

	// Update PIDController parameters (position loop)
	revolute_cascade.position_pid.kp = pos_kp;
	revolute_cascade.position_pid.ki = pos_ki;
	revolute_cascade.position_pid.kd = pos_kd;

	// Update VelocityPID parameters (velocity loop)
	revolute_cascade.velocity_pid.Kp = vel_kp;
	revolute_cascade.velocity_pid.Ki = vel_ki;
	revolute_cascade.velocity_pid.Kd = vel_kd;

	// Reset PIDController states (position loop)
	revolute_cascade.position_pid.error_integral = 0.0f;
	revolute_cascade.position_pid.error_prev = 0.0f;
	revolute_cascade.position_pid.last_derivative = 0.0f;

	// Reset VelocityPID states (velocity loop)
	revolute_cascade.velocity_pid.prev_error = 0.0f;
	revolute_cascade.velocity_pid.prev_prev_error = 0.0f;
	revolute_cascade.velocity_pid.prev_output = 0.0f;
}

void UpdatePrismaticTuningParameters(float pos_kp, float pos_ki, float pos_kd,
		float vel_kp, float vel_ki, float vel_kd) {
	prismatic_tuning.position_kp = pos_kp;
	prismatic_tuning.position_ki = pos_ki;
	prismatic_tuning.position_kd = pos_kd;
	prismatic_tuning.velocity_kp = vel_kp;
	prismatic_tuning.velocity_ki = vel_ki;
	prismatic_tuning.velocity_kd = vel_kd;

	// Update PIDController parameters (position loop)
	prismatic_cascade.position_pid.kp = pos_kp;
	prismatic_cascade.position_pid.ki = pos_ki;
	prismatic_cascade.position_pid.kd = pos_kd;

	// Update VelocityPID parameters (velocity loop)
	prismatic_cascade.velocity_pid.Kp = vel_kp;
	prismatic_cascade.velocity_pid.Ki = vel_ki;
	prismatic_cascade.velocity_pid.Kd = vel_kd;

	// Reset PIDController states (position loop)
	prismatic_cascade.position_pid.error_integral = 0.0f;
	prismatic_cascade.position_pid.error_prev = 0.0f;
	prismatic_cascade.position_pid.last_derivative = 0.0f;

	// Reset VelocityPID states (velocity loop)
	prismatic_cascade.velocity_pid.prev_error = 0.0f;
	prismatic_cascade.velocity_pid.prev_prev_error = 0.0f;
	prismatic_cascade.velocity_pid.prev_output = 0.0f;
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

//void IntegratedControlLoop() {
//	Encoder_Read(&revolute_encoder);
//	Encoder_CalculateSpeed(&revolute_encoder);
//
//	Encoder_Read(&prismatic_encoder);
//	Encoder_CalculateSpeed(&prismatic_encoder);
//
//	if (move_in_progress) {
//		Trapezoid_Update(&revolute_trap);
//		Trapezoid_Update(&prismatic_trap);
//
//		if (Trapezoid_IsComplete(&revolute_trap)
//				&& Trapezoid_IsComplete(&prismatic_trap)) {
//			move_in_progress = 0;
//		}
//	}
//
//	float revolute_setpoint =
//			move_in_progress ?
//					revolute_trap.position : revolute_tuning.position_setpoint;
//	float revolute_velocity =
//			move_in_progress ?
//					revolute_trap.velocity : revolute_cascade.velocity_setpoint;
//	float revolute_acceleration =
//			move_in_progress ? revolute_trap.acceleration : 0.0f;
//
//	float prismatic_setpoint =
//			move_in_progress ?
//					prismatic_trap.position :
//					prismatic_tuning.position_setpoint;
//	float prismatic_velocity =
//			move_in_progress ?
//					prismatic_trap.velocity :
//					prismatic_cascade.velocity_setpoint;
//	float prismatic_acceleration =
//			move_in_progress ? prismatic_trap.acceleration : 0.0f;
//
////	float revolute_output = CascadeController_Update(&revolute_cascade,
////			revolute_setpoint, revolute_encoder.position,
////			revolute_encoder.speed, 1.0f);
//
//	float revolute_ff = RevoluteFeedforward(revolute_setpoint,
//			revolute_velocity, revolute_acceleration, &revolute_motor_params,
//			&system_params);
//
//	float revolute_dist = RevoluteDisturbanceComp(revolute_encoder.position,
//			&revolute_disturbance, &revolute_motor_params);
//
////	float revolute_control = revolute_output + revolute_ff + revolute_dist;
////
////	float prismatic_output = CascadeController_Update(&prismatic_cascade,
////			prismatic_setpoint, prismatic_encoder.position,
////			prismatic_encoder.speed, 30.0f);
//
//	float prismatic_ff = PrismaticFeedforward(prismatic_setpoint,
//			prismatic_velocity, prismatic_acceleration, &prismatic_motor_params,
//			&system_params);
//
//	float prismatic_dist = PrismaticDisturbanceComp(revolute_encoder.position,
//			&prismatic_disturbance, &prismatic_motor_params);
//
//	float prismatic_control = prismatic_output + prismatic_ff + prismatic_dist;
//
//	SetRevoluteMotorPWM(revolute_control, revolute_tuning.max_voltage,
//			revolute_tuning.pwm_period);
//	SetPrismaticMotorPWM(prismatic_control, prismatic_tuning.max_voltage,
//			prismatic_tuning.pwm_period);
//}

// Initialize the circle drawer
void SmoothCircleDrawer_Init(SmoothCircleDrawer *drawer, float center_revolute,
		float center_prismatic, float radius, float max_velocity,
		float max_acceleration) {
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

void SetPrismaticRawTarget(float target) {
	prismatic_raw_setpoint = target;
	prismatic_tuning.position_setpoint = prismatic_raw_setpoint
			+ prismatic_compensation_offset;
}
void TestPositionStep(void) {
	static uint32_t last_step_time = 0;
	static uint8_t step_index = 0;
//    tuning.test_running = 1;
	static float position_setpoints[] = { 0.0f,
//			-0.3/8/RAD_PER_DEGREE,
//			-0.3*2/8/RAD_PER_DEGREE,
//			-0.3*3/8/RAD_PER_DEGREE,
			-0.3 * 4 / 8 / RAD_PER_DEGREE,
//			-0.3*5/8/RAD_PER_DEGREE,
//			-0.3*6/8/RAD_PER_DEGREE,
//			-0.3*7/8/RAD_PER_DEGREE,
			-0.3 * 8 / 8 / RAD_PER_DEGREE,
//			-0.3*7/8/RAD_PER_DEGREE,
//			-0.3*6/8/RAD_PER_DEGREE,
//			-0.3*5/8/RAD_PER_DEGREE,
			-0.3 * 4 / 8 / RAD_PER_DEGREE,
//			-0.3*3/8/RAD_PER_DEGREE,
//			-0.3*2/8/RAD_PER_DEGREE,
//			-0.3/8/RAD_PER_DEGREE,
			0.0f };

	static const uint8_t num_setpoints = sizeof(position_setpoints)
			/ sizeof(position_setpoints[0]);

	if (HAL_GetTick() - last_step_time > 1500) {
		step_index = (step_index + 1) % num_setpoints;

		prismatic_tuning.position_setpoint = position_setpoints[step_index];

		last_step_time = HAL_GetTick();
	}
}
void StartPrismaticOscillation(void) {
	prismatic_motion_active = true;
	current_target_index = 0;
	SetPrismaticRawTarget(prismatic_target_positions[current_target_index]);
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
		float angle_rad_pris = 0;

		float next_revolute_position = -angle_rad;
		float next_prismatic_position = angle_rad_pris;

		Trapezoid_SetTarget(&revolute_trap, revolute_encoder.position,
				next_revolute_position);
		Trapezoid_SetTarget(&prismatic_trap, prismatic_encoder.position,
				next_prismatic_position);

		move_in_progress = 1;
		last_move_time = current_time;

	}

	if (move_in_progress) {
		Trapezoid_Update(&revolute_trap);
		Trapezoid_Update(&prismatic_trap);

		revolute_tuning.position_setpoint = revolute_trap.position;
//        prismatic_tuning.position_setpoint = prismatic_trap.position - revolute_trap.position;

		revolute_cascade.velocity_setpoint = revolute_trap.velocity;
		prismatic_cascade.velocity_setpoint = prismatic_trap.velocity
				- revolute_trap.velocity;

		if (Trapezoid_IsComplete(&revolute_trap)
				&& Trapezoid_IsComplete(&prismatic_trap)) {
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

	float raw_position = prismatic_encoder.position
			- prismatic_compensation_offset;
	float position_error = fabsf(
			raw_position - prismatic_target_positions[current_target_index]);
	uint32_t current_time = HAL_GetTick();

	if (position_error < position_tolerance
			&& (current_time - last_target_change_time) > min_dwell_time_ms) {

		current_target_index = (current_target_index + 1) % 2;
		SetPrismaticRawTarget(prismatic_target_positions[current_target_index]);
		last_target_change_time = current_time;
	}
}

//void RevoluteControlLoop(void) {
//	Encoder_Read(&revolute_encoder);
//	Encoder_CalculateSpeed(&revolute_encoder);
//
//	// Calculate revolute changes for compensation
//	float revolute_position = revolute_encoder.position;
//	float revolute_velocity = revolute_encoder.speed;
//	float revolute_change = revolute_position - prev_revolute_position;
//	prev_revolute_position = revolute_position;
//
//	// If revolute moved significantly, adjust compensation offset
//	if (fabsf(revolute_change) > 0.0001f) {
//		float prismatic_change = -revolute_change;
//		prismatic_compensation_offset += prismatic_change;
//
//		// Update the actual setpoint while preserving the raw target
//		prismatic_tuning.position_setpoint = prismatic_raw_setpoint
//				+ prismatic_compensation_offset;
//
//		// Set velocity based on revolute motion
//		prismatic_cascade.velocity_setpoint = -revolute_velocity;
//		is_prismatic_compensating = true;
//	} else {
//		is_prismatic_compensating = false;
//	}
//
//	// Handle trajectory generation if a move is in progress
//	if (move_in_progress) {
//		Trapezoid_Update(&revolute_trap);
//		if (Trapezoid_IsComplete(&revolute_trap)) {
//			move_in_progress = 0;
//		}
//	}
//
//	// Determine setpoints based on whether a move is in progress
//	float position_setpoint;
//	float velocity_setpoint;
//	float acceleration_setpoint;
//
//	if (move_in_progress) {
//		// Using trapezoid profile
//		position_setpoint = revolute_trap.position;
//		velocity_setpoint = revolute_trap.velocity;
//		acceleration_setpoint = revolute_trap.acceleration;
//	} else {
//		// Using direct setpoints
//		position_setpoint = revolute_tuning.position_setpoint;
//		velocity_setpoint = revolute_cascade.velocity_setpoint;
//		acceleration_setpoint = 0.0f;
//	}
//
//	// Get current control signal and estimate state using Kalman filter
//	float control_signal = revolute_cascade.current_setpoint;
//	MotorModelKalman_Estimate(&revolute_motor_filter, control_signal,
//			revolute_encoder.position);
//
//	// Skip control if disabled
//	if (!revolute_tuning.control_enabled) {
//		SetRevoluteMotorPWM(0.0f, revolute_tuning.max_voltage,
//				revolute_tuning.pwm_period);
//		return;
//	}
//
//	// Apply filtering and phase lead compensation to velocity
//	float filtered_velocity = revolute_motor_filter.velocity;
//	float compensated_velocity = ApplyPhaseLead(filtered_velocity, 0.5f, 0.6f);
//
//	// Calculate control components
//	float v_feedback = CascadeController_Update(&revolute_cascade,
//			position_setpoint, revolute_encoder.position,
//			revolute_motor_filter.velocity, 1.0f);
//
//	float v_feedforward = RevoluteFeedforward(position_setpoint,
//			velocity_setpoint, acceleration_setpoint, &revolute_motor_params,
//			&system_params);
//
//	float v_disturbance = RevoluteDisturbanceComp(revolute_encoder.position,
//			&revolute_disturbance, &revolute_motor_params);
//
//	// Sum control components and apply to motor
//	float control_signal_output = v_feedback + v_feedforward + v_disturbance;
//	SetRevoluteMotorPWM(control_signal_output, revolute_tuning.max_voltage,
//			revolute_tuning.pwm_period);
//
//	// Store current position and velocity for next iteration
//	latest_revolute_position = revolute_encoder.position;
//	latest_revolute_velocity = revolute_encoder.speed;
//}
//
//void PrismaticControlLoop(void) {
//	Encoder_Read(&prismatic_encoder);
//	Encoder_CalculateSpeed(&prismatic_encoder);
//
//	// Update oscillation targets if active
//	if (prismatic_motion_active) {
//		UpdatePrismaticTargets();
//	}
//
//	if (!prismatic_tuning.control_enabled) {
//		SetPrismaticMotorPWM(0.0f, prismatic_tuning.max_voltage,
//				prismatic_tuning.pwm_period);
//		return;
//	}
//
//	// Select max velocity based on whether we're compensating
//	float max_velocity =
//			is_prismatic_compensating ?
//					prismatic_max_velocity_compensating :
//					prismatic_max_velocity_normal;
//
//	// Calculate control components
//	float v_feedforward = PrismaticFeedforward(
//			prismatic_tuning.position_setpoint,
//			prismatic_cascade.velocity_setpoint, 0.0f, &prismatic_motor_params,
//			&system_params);
//
//	float v_disturbance = PrismaticDisturbanceComp(prismatic_encoder.position,
//			&prismatic_disturbance, &prismatic_motor_params);
//
//	// Use the selected max velocity limit here
//	float v_feedback = CascadeController_Update(&prismatic_cascade,
//			prismatic_tuning.position_setpoint, prismatic_encoder.position,
//			prismatic_motor_filter.velocity, max_velocity);
//
//	float control_signal_output = v_feedback + v_feedforward + v_disturbance;
//	SetPrismaticMotorPWM(v_feedback, prismatic_tuning.max_voltage,
//			prismatic_tuning.pwm_period);
//}

void StopPrismaticOscillation(void) {
	prismatic_motion_active = false;
}

void SetPrismaticTargetToZero(void) {
	prismatic_tuning.control_enabled = true;

	if (prismatic_motion_active) {
		StopPrismaticOscillation();
	}

	SetPrismaticRawTarget(0.0f);
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
		revolute_tuning.position_setpoint =
				traversal->points[traversal->current_point].revolute_pos;
		prismatic_tuning.position_setpoint =
				traversal->points[traversal->current_point].prismatic_pos;
	}

	if (HAL_GetTick() - point_start_time > 2000) {
		traversal->current_point++;
		point_start_time = HAL_GetTick();

		if (traversal->current_point >= traversal->num_points) {
			traversal->active = 0;
			traversal->current_point = 0;
			return;
		}

		revolute_tuning.position_setpoint =
				traversal->points[traversal->current_point].revolute_pos;
		prismatic_tuning.position_setpoint =
				traversal->points[traversal->current_point].prismatic_pos;

	}
}

// Utility functions
int sign(float x) {
	if (x == 0.0f) {
		return 0;
	} else if (x < 0.0f) {
		return -1;
	} else {
		return 1;
	}
}

uint64_t micros(void) {
	return (uint64_t) HAL_GetTick() * 1000ULL;
}

// Enhanced trajectory initialization with automatic profile selection
void init_trapezoidal(Trajectory *trj, float init_pos, float final_pos) {
	// Reset all timing
	trj->t[Previous] = 0ULL;
	trj->t[Current] = 0ULL;
	trj->t_elapsed = 0.0f;

	// Set initial and final positions
	trj->Init_pos = init_pos;
	trj->Final_pos = final_pos;
	trj->delta_pos = final_pos - init_pos;
	trj->dir = (float) sign(trj->delta_pos);
	trj->delta_pos = fabs(trj->delta_pos);

	// Initialize current state
	trj->ang_pos = trj->Init_pos;
	trj->ang_velo = 0.0f;
	trj->ang_acc = 0.0f;
	trj->phase = PHASE_IDLE;

	// Calculate time to reach maximum velocity
	float t_to_max_vel = trj->desire_av / trj->desire_ac;

	// Calculate distance needed to reach maximum velocity and decelerate
	float s_to_max_vel = 0.5f * trj->desire_ac * t_to_max_vel * t_to_max_vel;
	float s_total_accel_decel = 2.0f * s_to_max_vel;

	// Automatic profile selection based on distance
	if (trj->delta_pos >= s_total_accel_decel) {
		// TRAPEZOIDAL PROFILE - Can reach maximum velocity
		trj->profile_type = PROFILE_TRAPEZOIDAL;

		trj->t_acc = t_to_max_vel;
		trj->t_dec = t_to_max_vel;
		trj->s_acc = s_to_max_vel;
		trj->s_dec = s_to_max_vel;
		trj->s_const = trj->delta_pos - trj->s_acc - trj->s_dec;
		trj->t_const = trj->s_const / trj->desire_av;
		trj->t_total = trj->t_acc + trj->t_const + trj->t_dec;
		trj->peak_velocity = 40.0f;

	} else {
		// TRIANGULAR PROFILE - Cannot reach maximum velocity
		trj->profile_type = PROFILE_TRIANGULAR;

		// Calculate time for acceleration (same as deceleration)
		trj->t_acc = sqrtf(trj->delta_pos / trj->desire_ac);
		trj->t_dec = trj->t_acc;
		trj->t_const = 0.0f;
		trj->t_total = trj->t_acc + trj->t_dec;

		// Calculate distances
		trj->s_acc = 0.5f * trj->desire_ac * trj->t_acc * trj->t_acc;
		trj->s_dec = trj->s_acc;
		trj->s_const = 0.0f;

		// Peak velocity for triangular profile
		trj->peak_velocity = trj->desire_ac * trj->t_acc;
	}
}

// Enhanced trajectory generation with smooth profiles
void generate_trapezoidal(Trajectory *trj) {
	if (!trj->active) {
		return;
	}

	// Initialize timing on first call
	if (trj->t[Previous] == 0ULL) {
		trj->t[Previous] = micros();
		trj->t[Current] = micros();
		trj->phase = PHASE_ACCELERATION;
		return;
	}

	// Update timing
	trj->t[Current] = micros();
	float dt = (float) (trj->t[Current] - trj->t[Previous]) * 1e-6f;
	trj->t_elapsed += dt;
	trj->t[Previous] = trj->t[Current];

	// Prevent division by zero and handle invalid dt
	if (dt <= 0.0f || dt > 0.1f) {
		return;
	}

	if (trj->profile_type == PROFILE_TRIANGULAR) {
		if (trj->t_elapsed < trj->t_acc) {
			// Acceleration phase
			trj->phase = PHASE_ACCELERATION;
			trj->ang_acc = trj->desire_ac * trj->dir;
//			trj->ang_velo = trj->desire_av * trj->dir;
			trj->ang_velo += trj->ang_acc * dt;
			trj->ang_pos += trj->ang_velo * dt + 0.5f * trj->ang_acc * dt * dt;

		} else if (trj->t_elapsed < trj->t_total) {
			// Deceleration phase
			trj->phase = PHASE_DECELERATION;
			trj->ang_acc = -trj->desire_ac * trj->dir;
			trj->ang_velo += trj->ang_acc * dt;
			trj->ang_pos += trj->ang_velo * dt + 0.5f * trj->ang_acc * dt * dt;

		} else {
			// Trajectory complete
			trj->phase = PHASE_COMPLETE;
			trj->ang_pos = trj->Final_pos;
			trj->ang_velo = 0.0f;
			trj->ang_acc = 0.0f;
			trj->active = 0;
		}

	} else {
		// TRAPEZOIDAL PROFILE IMPLEMENTATION
		if (trj->t_elapsed < trj->t_acc) {
			// Acceleration phase
			trj->phase = PHASE_ACCELERATION;
			trj->ang_acc = trj->desire_ac * trj->dir;
			trj->ang_velo += trj->ang_acc * dt;
			trj->ang_pos += trj->ang_velo * dt + 0.5f * trj->ang_acc * dt * dt;

		} else if (trj->t_elapsed < (trj->t_acc + trj->t_const)) {
			// Constant velocity phase
			trj->phase = PHASE_CONSTANT_VELOCITY;
			trj->ang_acc = 0.0f;
			trj->ang_velo = trj->desire_av * trj->dir;
			trj->ang_pos += trj->ang_velo * dt;

		} else if (trj->t_elapsed < trj->t_total) {
			// Deceleration phase
			trj->phase = PHASE_DECELERATION;
			trj->ang_acc = -trj->desire_ac * trj->dir;
			trj->ang_velo += trj->ang_acc * dt;
			trj->ang_pos += trj->ang_velo * dt + 0.5f * trj->ang_acc * dt * dt;

		} else {
			// Trajectory complete
			trj->phase = PHASE_COMPLETE;
			trj->ang_pos = trj->Final_pos;
			trj->ang_velo = 0.0f;
			trj->ang_acc = 0.0f;
			trj->active = 0;
		}
	}

	// Safety bounds checking
	if ((trj->dir > 0 && trj->ang_pos >= trj->Final_pos)
			|| (trj->dir < 0 && trj->ang_pos <= trj->Final_pos)) {
		trj->ang_pos = trj->Final_pos;
		trj->ang_velo = 0.0f;
		trj->ang_acc = 0.0f;
		trj->active = 0;
		trj->phase = PHASE_COMPLETE;
	}
}

// Trajectory control functions
void activate_generator(Trajectory *trj) {
	trj->active = 1;
	trj->phase = PHASE_IDLE;
}

void deactivate_generator(Trajectory *trj) {
	trj->active = 0;
	trj->phase = PHASE_COMPLETE;
	trj->ang_velo = 0.0f;
	trj->ang_acc = 0.0f;
}

uint8_t check_status(Trajectory *trj) {
	return trj->active;
}

// Get trajectory phase as string (for debugging)
const char* get_phase_string(TrajectoryPhase phase) {
	switch (phase) {
	case PHASE_IDLE:
		return "IDLE";
	case PHASE_ACCELERATION:
		return "ACCEL";
	case PHASE_CONSTANT_VELOCITY:
		return "CONST";
	case PHASE_DECELERATION:
		return "DECEL";
	case PHASE_COMPLETE:
		return "COMPLETE";
	default:
		return "UNKNOWN";
	}
}

// Get profile type as string (for debugging)
const char* get_profile_string(ProfileType type) {
	switch (type) {
	case PROFILE_TRIANGULAR:
		return "TRIANGULAR";
	case PROFILE_TRAPEZOIDAL:
		return "TRAPEZOIDAL";
	default:
		return "UNKNOWN";
	}
}

// Initialize trajectory generators with appropriate parameters
void InitializeTrajectoryGenerators(void) {
	revolute_trajectory.desire_av = 1.0f;    // 1 rad/s max velocity
	revolute_trajectory.desire_ac = 0.4f;    // 0.4 rad/s² acceleration
	revolute_trajectory.active = 0;
	revolute_trajectory.phase = PHASE_IDLE;

	prismatic_trajectory.desire_av = 40.0f;   // 0.5 rad/s max velocity
	prismatic_trajectory.desire_ac = 20.0f;   // 2.0 rad/s² acceleration
	prismatic_trajectory.active = 0;
	prismatic_trajectory.phase = PHASE_IDLE;
}

// Move functions with enhanced trajectory generationz
void MoveRevoluteToPositionNew(float target_position_rad) {
//	ResetRevolutePIDIntegrals();
	init_trapezoidal(&revolute_trajectory, revolute_encoder.position,
			target_position_rad);
	activate_generator(&revolute_trajectory);
}

void MovePrismaticToPositionNew(float target_position_rad) {
//	ResetPrismaticPIDIntegrals();
	init_trapezoidal(&prismatic_trajectory, prismatic_encoder.position,
			target_position_rad);
	activate_generator(&prismatic_trajectory);
}

// Coordinated movement with automatic profile selection
void MoveToCoordinatedPosition(float revolute_target, float prismatic_target) {
	MoveRevoluteToPositionNew(revolute_target);
	MovePrismaticToPositionNew(prismatic_target);
}

// Check if both trajectories are complete
uint8_t IsCoordinatedMoveComplete(void) {
	return (!check_status(&revolute_trajectory)
			&& !check_status(&prismatic_trajectory));
}

// Get trajectory progress (0.0 to 1.0)
float GetTrajectoryProgress(Trajectory *trj) {
	if (!trj->active || trj->t_total <= 0.0f) {
		return 0.0f;
	}

	float progress = trj->t_elapsed / trj->t_total;
	if (progress > 1.0f)
		progress = 1.0f;
	if (progress < 0.0f)
		progress = 0.0f;

	return progress;
}

// Set custom trajectory parameters
void SetTrajectoryParameters(Trajectory *trj, float max_velocity,
		float acceleration) {
	if (!trj->active) {
		trj->desire_av = max_velocity;
		trj->desire_ac = acceleration;
	}
}

// Emergency stop trajectory
void EmergencyStopTrajectory(Trajectory *trj) {
	if (trj->active) {
		trj->Final_pos = trj->ang_pos;  // Set current position as target
		trj->ang_velo = 0.0f;
		trj->ang_acc = 0.0f;
		trj->active = 0;
		trj->phase = PHASE_COMPLETE;
	}
}


void RevoluteControlLoopWithTrajectory(void) {
    Encoder_Read(&revolute_encoder);
    Encoder_CalculateSpeed(&revolute_encoder);

    // Smart coupling: Only when revolute is actively moving (has trajectory or velocity command)
    float revolute_position = revolute_encoder.position;
    float revolute_velocity = revolute_encoder.speed;
    float revolute_change = revolute_position - prev_revolute_position;
    prev_revolute_position = revolute_position;

    // Apply coupling ONLY when revolute is actively moving
    uint8_t revolute_is_moving = (check_status(&revolute_trajectory) ||
                                 fabs(revolute_cascade.velocity_setpoint) > 0.01f ||
                                 fabs(revolute_velocity) > 0.01f);

    if (revolute_is_moving && fabs(revolute_change) > 0.0001f) {
        // Enhanced compensation for belt coupling
        float coupling_ratio = 1.5f; // Adjust this value (try 0.8, 1.2, 1.5)
        float prismatic_compensation = -revolute_change * coupling_ratio;

        // Add velocity-based compensation
        float velocity_compensation = -revolute_velocity * coupling_ratio;

        prismatic_tuning.position_setpoint += prismatic_compensation;
        prismatic_cascade.velocity_setpoint = velocity_compensation;
    }

    // Handle trajectory generation if active
    if (check_status(&revolute_trajectory)) {
        generate_trapezoidal(&revolute_trajectory);
    }

    float position_setpoint;
    float velocity_setpoint;
    float acceleration_setpoint;

    if (check_status(&revolute_trajectory)) {
        position_setpoint = revolute_trajectory.ang_pos;
        velocity_setpoint = revolute_trajectory.ang_velo;
        acceleration_setpoint = revolute_trajectory.ang_acc;
    } else {
        revolute_tuning.position_setpoint = revolute_trajectory.ang_pos;
        revolute_cascade.velocity_setpoint = revolute_trajectory.ang_velo;
        position_setpoint = revolute_tuning.position_setpoint;
        velocity_setpoint = revolute_cascade.velocity_setpoint;
        acceleration_setpoint = 0.0f;
    }

    float control_signal = revolute_cascade.current_setpoint;
    MotorModelKalman_Estimate(&revolute_motor_filter, control_signal,
            revolute_encoder.position);

    if (!revolute_tuning.control_enabled) {
        SetRevoluteMotorPWM(0.0f, revolute_tuning.max_voltage,
                revolute_tuning.pwm_period);
        return;
    }

    float filtered_velocity = revolute_motor_filter.velocity;
    float compensated_velocity = ApplyPhaseLead(filtered_velocity, 0.5f, 0.6f);

    float v_feedback = CascadeController_Update(&revolute_cascade,
            position_setpoint, revolute_encoder.position, filtered_velocity,
            1.0f, velocity_setpoint);

    float v_feedforward = RevoluteFeedforward(position_setpoint,
            velocity_setpoint, acceleration_setpoint, &revolute_motor_params,
            &system_params);

    float v_disturbance = RevoluteDisturbanceComp(revolute_encoder.position,
            &revolute_disturbance, &revolute_motor_params);

    float control_signal_output = v_feedback + v_feedforward + v_disturbance;
    SetRevoluteMotorPWM(control_signal_output, revolute_tuning.max_voltage,
            revolute_tuning.pwm_period);

    latest_revolute_position = revolute_encoder.position;
    latest_revolute_velocity = revolute_encoder.speed;
}


void PrismaticControlLoopWithTrajectory(void) {
	Encoder_Read(&prismatic_encoder);
	Encoder_CalculateSpeed(&prismatic_encoder);

	// Handle trajectory generation if active
	if (check_status(&prismatic_trajectory)) {
		generate_trapezoidal(&prismatic_trajectory);
	}

	// Update oscillation targets if active (only when not using trajectory)
	if (prismatic_motion_active && !check_status(&prismatic_trajectory)) {
		UpdatePrismaticTargets();
	}

	float control_signal = prismatic_cascade.current_setpoint;
	MotorModelKalman_Estimate(&prismatic_motor_filter, control_signal,
			prismatic_encoder.position);

	if (!prismatic_tuning.control_enabled) {
		SetPrismaticMotorPWM(0.0f, prismatic_tuning.max_voltage,
				prismatic_tuning.pwm_period);
		return;
	}

	// Determine setpoints based on trajectory status
	float position_setpoint;
	float velocity_setpoint;
	float acceleration_setpoint = 0.0f;

	if (check_status(&prismatic_trajectory)) {
		// Using enhanced trajectory generator with smooth curves
		position_setpoint = prismatic_trajectory.ang_pos;
		velocity_setpoint = prismatic_trajectory.ang_velo;
		acceleration_setpoint = prismatic_trajectory.ang_acc;
		prismatic_tuning.position_setpoint = prismatic_trajectory.ang_pos;

	} else {
		prismatic_tuning.position_setpoint = prismatic_trajectory.ang_pos;
		prismatic_cascade.velocity_setpoint = prismatic_trajectory.ang_velo;
		position_setpoint = prismatic_tuning.position_setpoint;
		velocity_setpoint = prismatic_cascade.velocity_setpoint;
	}

	float max_velocity =
			is_prismatic_compensating ?
					prismatic_max_velocity_compensating :
					prismatic_max_velocity_normal;

	// Calculate control components
	float v_feedforward = PrismaticFeedforward(position_setpoint,
			velocity_setpoint, acceleration_setpoint, &prismatic_motor_params,
			&system_params);

	float v_disturbance = PrismaticDisturbanceComp(prismatic_encoder.position,
			&prismatic_disturbance, &prismatic_motor_params);

	float v_feedback = CascadeController_Update(&prismatic_cascade,
			position_setpoint, prismatic_encoder.position,
			prismatic_encoder.speed, max_velocity, velocity_setpoint);

	float control_signal_output = v_feedback + v_feedforward + v_disturbance;
	SetPrismaticMotorPWM(v_feedback, prismatic_tuning.max_voltage,
			prismatic_tuning.pwm_period);
}

void TestTrajectoryProfiles(void) {
	static uint32_t last_test_time = 0;
	static uint8_t test_index = 0;

	// Test positions with different distances to demonstrate auto profile selection
	static float test_positions[] = { 0.0f,           // Start
			0.05f,          // Small movement (will use triangular profile)
			0.0f,           // Back to start
			0.3f,           // Large movement (will use trapezoidal profile)
			0.0f,           // Back to start
			-0.2f,          // Medium movement
			0.0f            // Back to start
			};

	static const uint8_t num_tests = sizeof(test_positions)
			/ sizeof(test_positions[0]);

	// Only start new trajectory if current one is complete and enough time has passed
	if (!check_status(&revolute_trajectory)
			&& (HAL_GetTick() - last_test_time > 3000)) {
		test_index = (test_index + 1) % num_tests;

		// Set different parameters for demonstration
		if (test_index % 2 == 0) {
			// Faster movements for even indices
			SetTrajectoryParameters(&revolute_trajectory, 1.5f, 0.6f);
		} else {
			// Slower movements for odd indices
			SetTrajectoryParameters(&revolute_trajectory, 0.8f, 0.3f);
		}

		MoveRevoluteToPositionNew(test_positions[test_index]);
		last_test_time = HAL_GetTick();
	}
}

// Coordinated smooth trajectory test
//void TestCoordinatedSmoothTrajectory(void) {
//	static uint32_t last_move_time = 0;
//	static uint8_t move_index = 0;
//
//	// Define coordinated test positions that will use both profile types
//	static float revolute_positions[] = { 0.0f,           // Start
//			M_PI / 8.0f,      // Small angle (triangular profile likely)
//			M_PI / 4.0f,      // Medium angle
//			M_PI / 2.0f,      // Large angle (trapezoidal profile likely)
//			0.0f            // Back to start
//			};
//
//	static float prismatic_positions[] = { 0.0f,           // Start
//			0.05f,          // Small movement (triangular profile likely)
//			0.15f,          // Medium movement
//			-0.1f,          // Large movement (trapezoidal profile likely)
//			0.0f            // Back to start
//			};
//
//	static const uint8_t num_positions = sizeof(revolute_positions)
//			/ sizeof(revolute_positions[0]);
//
//	// Start new coordinated movement if previous one is complete
//	if (IsCoordinatedMoveComplete()
//			&& (HAL_GetTick() - last_move_time > 2500)) {
//		move_index = (move_index + 1) % num_positions;
//
//		// Vary trajectory parameters for different moves
//		switch (move_index) {
//		case 0: // Slow precise movement
//			SetTrajectoryParameters(&revolute_trajectory, 0.5f, 0.2f);
//			SetTrajectoryParameters(&prismatic_trajectory, 0.3f, 1.0f);
//			break;
//
//		case 1: // Medium speed
//			SetTrajectoryParameters(&revolute_trajectory, 1.0f, 0.4f);
//			SetTrajectoryParameters(&prismatic_trajectory, 0.5f, 2.0f);
//			break;
//
//		case 2: // Fast movement
//			SetTrajectoryParameters(&revolute_trajectory, 1.5f, 0.6f);
//			SetTrajectoryParameters(&prismatic_trajectory, 0.8f, 3.0f);
//			break;
//
//		default: // Default parameters
//			SetTrajectoryParameters(&revolute_trajectory, 1.0f, 0.4f);
//			SetTrajectoryParameters(&prismatic_trajectory, 0.5f, 2.0f);
//			break;
//		}
//
//		MoveToCoordinatedPosition(revolute_positions[move_index],
//				prismatic_positions[move_index]);
//		last_move_time = HAL_GetTick();
//	}
//}
// Test complete
// You could add code here to log results or display success

//
//void MotorControlUpdate(void) {
//    RevoluteControlLoop();
//    PrismaticControlLoop();
//}

// Add this function to your code

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

CartesianPosition forward_kinematics(JointPosition joints) {
	CartesianPosition cart;

	float linear_prismatic = -joints.prismatic_pos / COUPLING_RATIO;
	float actual_prismatic_extension = joints.prismatic_pos
			- joints.revolute_angle;

	float effective_radius = -actual_prismatic_extension / COUPLING_RATIO;

	// Convert to Cartesian coordinates
	cart.x = effective_radius * cos(joints.revolute_angle);
	cart.y = effective_radius * sin(joints.revolute_angle);

	return cart;
}

JointPosition inverse_kinematics(CartesianPosition target) {
	JointPosition joints;

	// Convert Cartesian to polar coordinates
	float r = sqrt(target.x * target.x + target.y * target.y);
	float theta = atan2(target.y, target.x);

	joints.revolute_angle = theta;

	float required_linear_displacement = r;

	joints.prismatic_pos = -required_linear_displacement * COUPLING_RATIO;

//    joints.prismatic_pos += theta;
	joints.prismatic_pos -= joints.revolute_angle;

	for (int iter = 0; iter < 2; iter++) {
		CartesianPosition current = forward_kinematics(joints);
		float error_x = target.x - current.x;
		float error_y = target.y - current.y;
		float error_magnitude = sqrt(error_x * error_x + error_y * error_y);

		if (error_magnitude < PRISMATIC_TOLERANCE) {
			break; // Converged within tolerance
		}

		// Apply small correction
		float correction = error_magnitude * COUPLING_RATIO * 0.1f;
		if ((error_x * cos(theta) + error_y * sin(theta)) > 0) {
			joints.prismatic_pos += correction;
		} else {
			joints.prismatic_pos -= correction;
		}
	}

	// Apply joint limits based on your system constraints
	float max_prismatic_rad = MAX_PRISMATIC_RANGE * COUPLING_RATIO;
	if (joints.prismatic_pos > max_prismatic_rad) {
		joints.prismatic_pos = max_prismatic_rad;
	} else if (joints.prismatic_pos < -max_prismatic_rad) {
		joints.prismatic_pos = -max_prismatic_rad;
	}

	return joints;
}

void MoveToCartesianPosition(CartesianPosition target) {
	JointPosition joints = inverse_kinematics(target);

	MoveRevoluteToPositionNew(joints.revolute_angle);
	MovePrismaticToPositionNew(joints.prismatic_pos);

}

void MoveToCartesianPositionSynced(CartesianPosition target) {
	JointPosition joints = inverse_kinematics(target);

	float current_revolute = revolute_encoder.position;
	float current_prismatic = prismatic_encoder.position;

	// For demonstration, assume current position is (0,0)
	float revolute_distance = fabs(joints.revolute_angle - 0.0f);
	float prismatic_distance = fabs(joints.prismatic_pos - 0.0f);

	float revolute_time = CalculateTrajectoryTime(revolute_distance,
	TRAJECTORY_MAX_VEL_REVOLUTE,
	TRAJECTORY_MAX_ACCEL_REVOLUTE);
	float prismatic_time = CalculateTrajectoryTime(prismatic_distance,
	TRAJECTORY_MAX_VEL_PRISMATIC,
	TRAJECTORY_MAX_ACCEL_PRISMATIC);

	float sync_time = fmaxf(revolute_time, prismatic_time);

	float revolute_velocity = revolute_distance / (sync_time * 0.8f);
	float prismatic_velocity = prismatic_distance / (sync_time * 0.8f);

	SetTrajectoryParameters(&revolute_trajectory, revolute_velocity,
	TRAJECTORY_MAX_ACCEL_REVOLUTE);
	SetTrajectoryParameters(&prismatic_trajectory, prismatic_velocity,
	TRAJECTORY_MAX_ACCEL_PRISMATIC);
	MoveRevoluteToPositionNew(joints.revolute_angle);
	MovePrismaticToPositionNew(joints.prismatic_pos);
}

uint8_t ValidateCartesianTarget(float x, float y) {
	float distance = sqrt(x * x + y * y);
	return (distance <= WORKSPACE_RADIUS) && (distance >= 0.0f);
}

void MoveToCartesianPositionSTM32(float x, float y) {
	CartesianPosition target = { x, y };

	// Check if target is reachable
	if (!ValidateCartesianTarget(x, y)) {
		return; // Skip unreachable targets
	}

	// Calculate joint positions using inverse kinematics
	joints = inverse_kinematics(target);

	// Calculate synchronized timing
	float revolute_distance = fabs(
			joints.revolute_angle - revolute_encoder.position);
	float prismatic_distance = fabs(
			joints.prismatic_pos - prismatic_encoder.position);

	float revolute_time = CalculateTrajectoryTime(revolute_distance, 1.0f,
			0.4f);
	float prismatic_time = CalculateTrajectoryTime(prismatic_distance, 40.0f,
			5.0f);
	float sync_time = fmaxf(revolute_time, prismatic_time);

	// Adjust velocities for synchronization
	float revolute_velocity = revolute_distance / (sync_time * 0.8f);
	float prismatic_velocity = prismatic_distance / (sync_time * 0.8f);

	// Limit velocities to maximum values
	if (revolute_velocity > 1.0f)
		revolute_velocity = 1.0f;
	if (prismatic_velocity > 40.0f)
		prismatic_velocity = 40.0f;

	SetTrajectoryParameters(&revolute_trajectory, revolute_velocity, 0.4f);
	SetTrajectoryParameters(&prismatic_trajectory, prismatic_velocity, 5.0f);

	MoveRevoluteToPositionNew(joints.revolute_angle);
	MovePrismaticToPositionNew(joints.prismatic_pos);
}

/**
 * DrawCartesianLine implementation for STM32 Timer4
 * This is the ONLY test function that will run
 */
void DrawCartesianLineSTM32(CartesianPosition start, CartesianPosition end,
		int num_points) {
	static uint8_t line_active = 0;
	static uint8_t point_index = 0;
	static uint32_t last_point_time = 0;
	static CartesianPosition line_start, line_end;
	static int total_points;

	// Initialize line drawing on first call
	if (!line_active) {
		line_active = 1;
		point_index = 0;
		line_start = start;
		line_end = end;
		total_points = num_points;
		last_point_time = HAL_GetTick();
		return;
	}

	// Move to next point every 3 seconds and only if previous movement is complete
	if (HAL_GetTick() - last_point_time > 3000 && IsCoordinatedMoveComplete()) {
		if (point_index <= total_points) {
			float t = (float) point_index / total_points;
			CartesianPosition current_point;
			current_point.x = line_start.x + t * (line_end.x - line_start.x);
			current_point.y = line_start.y + t * (line_end.y - line_start.y);

			// Move to the calculated point
			MoveToCartesianPositionSTM32(current_point.x, current_point.y);

			point_index++;
		} else {
			// Line complete, reset for next line
			line_active = 0;
			point_index = 0;
		}

		last_point_time = HAL_GetTick();
	}
}

/**
 * Get current Cartesian position from encoders
 */
CartesianPosition GetCurrentCartesianPosition(void) {
	JointPosition current_joints;
	current_joints.revolute_angle = revolute_encoder.position;
	current_joints.prismatic_pos = prismatic_encoder.position;

	return forward_kinematics(current_joints);
}

void ExecuteCoordPositionsWithInverseKinematics(void) {
	static uint32_t last_move_time = 0;
	static uint8_t move_index = 0;

	static CoordPosition coord_positions[] = { { 0.0f, 0.0f }, // Both at zero
			{ M_PI / 4.0f, -0.05 * 2.0 * M_PI / 0.08f - M_PI / 4.0f }, // Both move together
			{ M_PI / 2.0f, -0.1 * 2.0 * M_PI / 0.08f - M_PI / 2.0f }, // Both move together
			{ 0.0f, -0.2 * 2.0 * M_PI / 0.08f } // Both return to zero
	};

	static const uint8_t num_positions = sizeof(coord_positions)
			/ sizeof(coord_positions[0]);

	if (IsCoordinatedMoveComplete()
			&& (HAL_GetTick() - last_move_time > 3000)) {
		CoordPosition current_coord = coord_positions[move_index];

		JointPosition joints = { current_coord.revolute_pos,
				current_coord.prismatic_pos };
		CartesianPosition target_cartesian = forward_kinematics(joints);
		MoveToCartesianPositionSTM32(target_cartesian.x, target_cartesian.y);

		move_index = (move_index + 1) % num_positions;
		last_move_time = HAL_GetTick();
	}
}

void SimpleCartesianInverseTest(void) {
	static uint32_t last_move_time = 0;
	static uint8_t move_index = 0;

	static CartesianPosition simple_pattern[] = { { 0.0f, 0.05f }, // Forward 5cm
			{ 0.05f, 0.05f },     // Right 5cm, forward 5cm
			{ 0.05f, 0.0f },      // Right 5cm
			{ 0.0f, 0.0f }        // Back to origin
	};

	static const uint8_t num_positions = sizeof(simple_pattern)
			/ sizeof(simple_pattern[0]);

	if (IsCoordinatedMoveComplete()
			&& (HAL_GetTick() - last_move_time > 3000)) {
		CartesianPosition target = simple_pattern[move_index];

		JointPosition calculated_joints = inverse_kinematics(target);

		MoveToCartesianPositionSTM32(target.x, target.y);

		move_index = (move_index + 1) % num_positions;
		last_move_time = HAL_GetTick();
	}
}

void PreprocessTextPath(CartesianPosition *input_points, int num_points,
		OptimizedPoint *output_points) {
	for (int i = 0; i < num_points; i++) {
		output_points[i].pos = input_points[i];
		output_points[i].speed_factor = 1.0f;
		output_points[i].is_corner = 0;

		// Detect corners (sharp direction changes)
		if (i > 1 && i < num_points - 1) {
			CartesianPosition prev = input_points[i - 1];
			CartesianPosition curr = input_points[i];
			CartesianPosition next = input_points[i + 1];

			// Calculate vectors
			float v1_x = curr.x - prev.x;
			float v1_y = curr.y - prev.y;
			float v2_x = next.x - curr.x;
			float v2_y = next.y - curr.y;

			// Calculate magnitudes
			float v1_mag = sqrt(v1_x * v1_x + v1_y * v1_y);
			float v2_mag = sqrt(v2_x * v2_x + v2_y * v2_y);

			if (v1_mag > 0.001f && v2_mag > 0.001f) {
				// Normalize vectors
				v1_x /= v1_mag;
				v1_y /= v1_mag;
				v2_x /= v2_mag;
				v2_y /= v2_mag;

				// Calculate dot product (cosine of angle)
				float dot_product = v1_x * v2_x + v1_y * v2_y;

				// If angle is sharp (< 135 degrees), mark as corner
				if (dot_product < -0.707f) {
					output_points[i].is_corner = 1;
					output_points[i].speed_factor = 0.2f; // Much slower for sharp corners
				} else if (dot_product < 0.0f) {
					// Medium turn
					output_points[i].speed_factor = 0.5f;
				}

				// Very small segments need slower movement
				if (v1_mag < 0.002f || v2_mag < 0.002f) {
					output_points[i].speed_factor *= 0.3f;
					output_points[i].is_corner = 1;
				}
			}
		}

		if (i == 0 || i == num_points - 1) {
			output_points[i].speed_factor = 0.3f;
		}
	}
}

void UpdateLookAheadBuffer(LookAheadBuffer *buffer, CartesianPosition new_point) {
	buffer->positions[buffer->index] = new_point;
	buffer->index = (buffer->index + 1) % 5;
	if (buffer->count < 5) {
		buffer->count++;
	}
}

CartesianPosition GetSmoothedPosition(LookAheadBuffer *buffer,
		float smoothing_factor) {
	if (buffer->count < 2) {
		return buffer->positions[0]; // Not enough points for smoothing
	}

	CartesianPosition smoothed = { 0, 0 };
	float total_weight = 0;

	for (int i = 0; i < buffer->count; i++) {
		int idx = (buffer->index - 1 - i + 5) % 5;
		float weight = expf(-i * smoothing_factor);

		smoothed.x += buffer->positions[idx].x * weight;
		smoothed.y += buffer->positions[idx].y * weight;
		total_weight += weight;
	}

	if (total_weight > 0) {
		smoothed.x /= total_weight;
		smoothed.y /= total_weight;
	} else {
		smoothed = buffer->positions[0];
	}

	return smoothed;
}

void CalculateVelocityFeedforward(CartesianPosition current,
		CartesianPosition target, float dt, float speed_factor, float *vel_x,
		float *vel_y) {
	if (dt <= 0) {
		*vel_x = 0;
		*vel_y = 0;
		return;
	}

	*vel_x = (target.x - current.x) / dt;
	*vel_y = (target.y - current.y) / dt;

	// Apply speed factor
	*vel_x *= speed_factor;
	*vel_y *= speed_factor;

	float max_cartesian_vel = 0.03f; // 3cm/s max for good drawing quality
	float vel_mag = sqrt((*vel_x) * (*vel_x) + (*vel_y) * (*vel_y));

	if (vel_mag > max_cartesian_vel) {
		float scale = max_cartesian_vel / vel_mag;
		*vel_x *= scale;
		*vel_y *= scale;
	}
}

void CartesianToJointVelocity(float cart_vel_x, float cart_vel_y,
		float current_revolute, float current_prismatic,
		float *joint_vel_revolute, float *joint_vel_prismatic) {

	float r = -current_prismatic / COUPLING_RATIO;

	if (fabs(r) > 0.005f) { // Avoid division by very small numbers
		// Jacobian for polar to Cartesian conversion
		// x = r * cos(theta), y = r * sin(theta)
		// dx/dt = dr/dt * cos(theta) - r * sin(theta) * dtheta/dt
		// dy/dt = dr/dt * sin(theta) + r * cos(theta) * dtheta/dt

		float cos_theta = cos(current_revolute);
		float sin_theta = sin(current_revolute);

		float det = r; // Simplified determinant
		if (fabs(det) > 0.001f) {
			*joint_vel_revolute = (cart_vel_x * (-sin_theta)
					+ cart_vel_y * cos_theta) / r;
			*joint_vel_prismatic = -COUPLING_RATIO
					* (cart_vel_x * cos_theta + cart_vel_y * sin_theta);
		} else {
			*joint_vel_revolute = 0;
			*joint_vel_prismatic = 0;
		}
	} else {
		*joint_vel_revolute = 0;
		*joint_vel_prismatic = -COUPLING_RATIO
				* sqrt(cart_vel_x * cart_vel_x + cart_vel_y * cart_vel_y);
	}

	float max_revolute_vel = 2.0f; // rad/s
	float max_prismatic_vel = 50.0f; // rad/s

	if (fabs(*joint_vel_revolute) > max_revolute_vel) {
		*joint_vel_revolute =
				(*joint_vel_revolute > 0) ?
						max_revolute_vel : -max_revolute_vel;
	}
	if (fabs(*joint_vel_prismatic) > max_prismatic_vel) {
		*joint_vel_prismatic =
				(*joint_vel_prismatic > 0) ?
						max_prismatic_vel : -max_prismatic_vel;
	}
}

uint8_t IsPositionReached(CartesianPosition target) {
	CartesianPosition current = GetCurrentCartesianPosition();

	float error_x = target.x - current.x;
	float error_y = target.y - current.y;
	float position_error = sqrt(error_x * error_x + error_y * error_y);

	return (position_error < drawing_mode.position_tolerance);
}

// Helper function: Adjust PID parameters for drawing
void AdjustPIDForDrawing(uint8_t fast_mode) {
	if (fast_mode) {
		// Optimized gains for drawing - faster response, good stability
		UpdateRevoluteTuningParameters(8.0f, 0.03f, 1.2f, 1.2f, 0.06f, 0.0f);
		UpdatePrismaticTuningParameters(35.0f, 0.12f, 4.5f, 1.0f, 0.45f, 0.0f);
	} else {
		// Restore original conservative gains
		UpdateRevoluteTuningParameters(5.0f, 0.000000000000000000000000001f,
				0.0f, 0.8475f, 0.00625f, 0.0f);
		UpdatePrismaticTuningParameters(25.575f, 0.0755f, 2.5f, 0.63f, 0.3f,
				0.0f);
	}
}

//#define FIBO_COMMANDS_COUNT (sizeof(fibo_g08_commands) / sizeof(fibo_g08_commands[0]))

static CartesianPosition FIBO_G08_complete[] = {
//		// Letter F
//		{0.085000f, 0.000000f},{0.085000f, 0.004444f},
//		{0.085000f, 0.008889f},{0.085000f, 0.013333f},
//		{0.085000f, 0.017778f},{0.085000f, 0.022222f},
//		{0.085000f, 0.026667f},{0.085000f, 0.031111f},
//		{0.085000f, 0.035556f},{0.085000f, 0.040000f},
//		{0.083333f, 0.040000f},{0.081667f, 0.040000f},
//		{0.080000f, 0.040000f},{0.078333f, 0.040000f},
//		{0.076667f, 0.040000f},{0.075000f, 0.040000f},
//		{0.073333f, 0.040000f},{0.071667f, 0.040000f},
//		{0.070000f, 0.040000f},{0.070000f, 0.036667f},
//		{0.070000f, 0.033333f},{0.070000f, 0.030000f},
//		{0.070000f, 0.026667f},{0.070000f, 0.023333f},
//		{0.070000f, 0.020000f},{0.070000f, 0.016667f},
//		{0.070000f, 0.013333f},{0.070000f, 0.010000f},
//		{0.068333f, 0.010000f},{0.066667f, 0.010000f},
//		{0.065000f, 0.010000f},{0.063333f, 0.010000f},
//		{0.061667f, 0.010000f},{0.060000f, 0.010000f},
//		{0.058333f, 0.010000f},{0.056667f, 0.010000f},
//		{0.055000f, 0.010000f},{0.055000f, 0.013333f},
//		{0.055000f, 0.016667f},{0.055000f, 0.020000f},
//		{0.055000f, 0.023333f},{0.055000f, 0.026667f},
//		{0.055000f, 0.030000f},{0.055000f, 0.033333f},
//		{0.055000f, 0.036667f},{0.055000f, 0.040000f},
//		{0.053333f, 0.040000f},{0.051667f, 0.040000f},
//		{0.050000f, 0.040000f},{0.048333f, 0.040000f},
//		{0.046667f, 0.040000f},{0.045000f, 0.040000f},
//		{0.043333f, 0.040000f},{0.041667f, 0.040000f},
//		{0.040000f, 0.040000f},{0.040000f, 0.036667f},
//		{0.040000f, 0.033333f},{0.040000f, 0.030000f},
//		{0.040000f, 0.026667f},{0.040000f, 0.023333f},
//		{0.040000f, 0.020000f},{0.040000f, 0.016667f},
//		{0.040000f, 0.013333f},{0.040000f, 0.010000f},
//		{0.035556f, 0.010000f},{0.031111f, 0.010000f},
//		{0.026667f, 0.010000f},{0.022222f, 0.010000f},
//		{0.017778f, 0.010000f},{0.013333f, 0.010000f},
//		{0.008889f, 0.010000f},{0.004444f, 0.010000f},
//		{0.000000f, 0.010000f},{0.000000f, 0.000000f},

		// Letter F (offset)

		{ 0.000000f, -0.200000f }, { 0.009444f, -0.200000f }, { 0.018889f,
				-0.200000f }, { 0.028333f, -0.200000f },
		{ 0.037778f, -0.200000f }, { 0.047222f, -0.200000f }, { 0.056667f,
				-0.200000f }, { 0.066111f, -0.200000f },
		{ 0.075556f, -0.200000f }, { 0.085000f, -0.200000f }, { 0.085000f,
				-0.200000f }, { 0.085000f, -0.195556f },
		{ 0.085000f, -0.191111f }, { 0.085000f, -0.186667f }, { 0.085000f,
				-0.182222f }, { 0.085000f, -0.177778f },
		{ 0.085000f, -0.173333f }, { 0.085000f, -0.168889f }, { 0.085000f,
				-0.164444f }, { 0.085000f, -0.160000f },

		{ 0.083333f, -0.160000f }, { 0.081667f, -0.160000f }, { 0.080000f,
				-0.160000f }, { 0.078333f, -0.160000f },
		{ 0.076667f, -0.160000f }, { 0.075000f, -0.160000f }, { 0.073333f,
				-0.160000f }, { 0.071667f, -0.160000f },

		{ 0.070000f, -0.160000f }, { 0.070000f, -0.163333f }, { 0.070000f,
				-0.166667f }, { 0.070000f, -0.170000f },
		{ 0.070000f, -0.173333f }, { 0.070000f, -0.176667f }, { 0.070000f,
				-0.180000f }, { 0.070000f, -0.183333f },
		{ 0.070000f, -0.186667f }, { 0.070000f, -0.190000f },

		{ 0.068333f, -0.190000f }, { 0.066667f, -0.190000f }, { 0.065000f,
				-0.190000f }, { 0.063333f, -0.190000f },
		{ 0.061667f, -0.190000f }, { 0.060000f, -0.190000f }, { 0.058333f,
				-0.190000f }, { 0.056667f, -0.190000f },
		{ 0.055000f, -0.190000f }, { 0.055000f, -0.186667f }, { 0.055000f,
				-0.183333f }, { 0.055000f, -0.180000f },
		{ 0.055000f, -0.176667f }, { 0.055000f, -0.173333f }, { 0.055000f,
				-0.170000f }, { 0.055000f, -0.166667f },
		{ 0.055000f, -0.163333f }, { 0.055000f, -0.160000f }, { 0.053333f,
				-0.160000f }, { 0.051667f, -0.160000f },
		{ 0.050000f, -0.160000f }, { 0.048333f, -0.160000f }, { 0.046667f,
				-0.160000f }, { 0.045000f, -0.160000f },
		{ 0.043333f, -0.160000f }, { 0.041667f, -0.160000f }, { 0.040000f,
				-0.160000f }, { 0.040000f, -0.163333f },
		{ 0.040000f, -0.166667f }, { 0.040000f, -0.170000f }, { 0.040000f,
				-0.173333f }, { 0.040000f, -0.176667f },
		{ 0.040000f, -0.180000f }, { 0.040000f, -0.183333f }, { 0.040000f,
				-0.186667f }, { 0.040000f, -0.190000f },
		{ 0.035556f, -0.190000f }, { 0.031111f, -0.190000f }, { 0.026667f,
				-0.190000f }, { 0.022222f, -0.190000f },
		{ 0.017778f, -0.190000f }, { 0.013333f, -0.190000f }, { 0.008889f,
				-0.190000f }, { 0.004444f, -0.190000f },
		{ 0.000000f, -0.190000f }, { 0.000000f, -0.200000f },

//		 Letter F (offset)
//		{ 0.000000f, -0.200000f },
//		{ 0.085000f, -0.200000f },
//		{ 0.085000f, -0.180000f },
//		{ 0.085000f, -0.160000f },
//
//		{ 0.077500f, -0.160000f },
//		{ 0.070000f, -0.160000f },
//		{ 0.070000f, -0.175000f },
//		{ 0.070000f, -0.190000f },
//		{ 0.060000f, -0.190000f },
//
//		{ 0.060000f, -0.160000f },
//		{ 0.055000f, -0.160000f },
//		{ 0.040000f, -0.160000f },
//		{ 0.045000f, -0.160000f },
//
//		{ 0.045000f, -0.190000f },
//		{ 0.022500f, -0.190000f },
//		{ 0.000000f, -0.190000f },
//
//		{ 0.000000f, -0.200000f },

//		// Move to I
//		{0.0f, 0.050000f},
//		// Letter I
//		{0.009444f, 0.050000f},{0.018889f, 0.050000f},
//		{0.028333f, 0.050000f},{0.037778f, 0.050000f},
//		{0.047222f, 0.050000f},{0.056667f, 0.050000f},
//		{0.066111f, 0.050000f},{0.075556f, 0.050000f},
//		{0.085000f, 0.050000f},{0.085000f, 0.052222f},
//		{0.085000f, 0.054444f},{0.085000f, 0.056667f},
//		{0.085000f, 0.058889f},{0.085000f, 0.060000f},
//		{0.075556f, 0.060000f},{0.066111f, 0.060000f},
//		{0.056667f, 0.060000f},{0.047222f, 0.060000f},
//		{0.037778f, 0.060000f},{0.028333f, 0.060000f},
//		{0.018889f, 0.060000f},{0.009444f, 0.060000f},
//		{0.000000f, 0.060000f},{0.000000f, 0.050000f},

		// Move to I(offset)
		{ 0.000000f, -0.1500000f },
		// Letter I(offset)
		{ 0.009444f, -0.150000f }, { 0.018889f, -0.150000f }, { 0.028333f,
				-0.150000f }, { 0.037778f, -0.150000f },
		{ 0.047222f, -0.150000f }, { 0.056667f, -0.150000f }, { 0.066111f,
				-0.150000f }, { 0.075556f, -0.150000f },
		{ 0.085000f, -0.150000f }, { 0.085000f, -0.147778f }, { 0.085000f,
				-0.145556f }, { 0.085000f, -0.143333f },
		{ 0.085000f, -0.141111f }, { 0.085000f, -0.140000f }, { 0.075556f,
				-0.140000f }, { 0.066111f, -0.140000f },
		{ 0.056667f, -0.140000f }, { 0.047222f, -0.140000f }, { 0.037778f,
				-0.140000f }, { 0.028333f, -0.140000f },
		{ 0.018889f, -0.140000f }, { 0.009444f, -0.140000f }, { 0.000000f,
				-0.140000f }, { 0.000000f, -0.150000f },
//
////		// Move to B
////		{0.0f, 0.070000f},
////      // Letter B
////		{0.009444f, 0.070000f},{0.018889f, 0.070000f},
////		{0.028333f, 0.070000f},{0.037778f, 0.070000f},
////		{0.047222f, 0.070000f},{0.056667f, 0.070000f},
////		{0.066111f, 0.070000f},{0.075556f, 0.070000f},
////		{0.085f, 0.07f},{0.085f, 0.08f},{0.085f, 0.09f},
////		{0.085f, 0.10f},
////		{0.085f, 0.11}, {0.06375f, 0.12f},{0.0425f,0.11},{0.02125f, 0.12f}, {0.0f, 0.11f}, {0.0f, 0.070000f},
////		//Inner letter B
////		{0.0f, 0.08f},{0.01f, 0.08f},{0.02f, 0.08f},{0.03f, 0.08f},{0.04f, 0.08f},{0.05f, 0.08f},{0.06f, 0.08f},{0.07f, 0.08f}
////		,{0.07f, 0.10f},{0.06f, 0.1f},{0.055f, 0.1f},{0.05f, 0.08f},{0.038f, 0.08f},{0.04f, 0.09f},{0.035f, 0.1f},{0.02f, 0.1f},{0.015f, 0.09f}
////		,{0.01f, 0.08f},{0.0f, 0.08f},
//
		// Move to B(offset)
		{ 0.000000f, -0.130000f },
		// Letter B(offset)
		{ 0.009444f, -0.130000f }, { 0.018889f, -0.130000f }, { 0.028333f,
				-0.130000f }, { 0.037778f, -0.130000f },
		{ 0.047222f, -0.130000f }, { 0.056667f, -0.130000f }, { 0.066111f,
				-0.130000f }, { 0.075556f, -0.130000f },
		{ 0.085000f, -0.130000f }, { 0.085000f, -0.120000f }, { 0.085000f,
				-0.110000f }, { 0.085000f, -0.100000f },
		{ 0.085000f, -0.090000f }, { 0.063750f, -0.080000f }, { 0.042500f,
				-0.090000f }, { 0.021250f, -0.080000f },
		{ 0.000000f, -0.090000f }, { 0.000000f, -0.130000f },
		//Inner letter B (offset)
		{ 0.000000f, -0.120000f }, { 0.010000f, -0.120000f }, { 0.020000f,
				-0.120000f }, { 0.030000f, -0.120000f },
		{ 0.040000f, -0.120000f }, { 0.050000f, -0.120000f }, { 0.060000f,
				-0.120000f }, { 0.070000f, -0.120000f },
		{ 0.070000f, -0.100000f }, { 0.060000f, -0.100000f }, { 0.055000f,
				-0.100000f }, { 0.050000f, -0.120000f },
		{ 0.038000f, -0.120000f }, { 0.040000f, -0.110000f }, { 0.035000f,
				-0.100000f }, { 0.020000f, -0.100000f },
		{ 0.015000f, -0.110000f }, { 0.010000f, -0.120000f }, { 0.000000f,
				-0.120000f },
//
////		// Move to O
////		{0.0f, 0.120000f},
////		// Letter O
////		{0.000000f, 0.120000f},{0.004444f, 0.120000f},
////		{0.008889f, 0.120000f},{0.013333f, 0.120000f},
////		{0.017778f, 0.120000f},{0.022222f, 0.120000f},
////		{0.026667f, 0.120000f},{0.031111f, 0.120000f},
////		{0.035556f, 0.120000f},{0.040000f, 0.120000f},
////		{0.044444f, 0.120000f},{0.048889f, 0.120000f},
////		{0.053333f, 0.120000f},{0.057778f, 0.120000f},
////		{0.062222f, 0.120000f},{0.066667f, 0.120000f},
////		{0.071111f, 0.120000f},{0.075556f, 0.120000f},
////		{0.080000f, 0.120000f},{0.085000f, 0.120000f},
////		{0.085000f, 0.124444f},{0.085000f, 0.128889f},
////		{0.085000f, 0.133333f},{0.085000f, 0.137778f},
////		{0.085000f, 0.142222f},{0.085000f, 0.146667f},
////		{0.085000f, 0.151111f},{0.085000f, 0.155556f},
////		{0.085000f, 0.160000f},{0.080000f, 0.160000f},{0.075556f, 0.160000f},
////		{0.071111f, 0.160000f},{0.066667f, 0.160000f},
////		{0.062222f, 0.160000f},{0.057778f, 0.160000f},
////		{0.053333f, 0.160000f},{0.048889f, 0.160000f},
////		{0.044444f, 0.160000f},{0.040000f, 0.160000f},
////		{0.035556f, 0.160000f},{0.031111f, 0.160000f},
////		{0.026667f, 0.160000f},{0.022222f, 0.160000f},
////		{0.017778f, 0.160000f},{0.013333f, 0.160000f},
////		{0.008889f, 0.160000f},{0.004444f, 0.160000f},
////		{0.000000f, 0.160000f},
////		{0.000000f, 0.155556f},{0.000000f, 0.151111f},
////		{0.000000f, 0.146667f},{0.000000f, 0.142222f},
////		{0.000000f, 0.137778f},{0.000000f, 0.133333f},
////		{0.000000f, 0.128889f},{0.000000f, 0.124444f},
////		{0.000000f, 0.120000f},
//
//		 Move to O (offset)
		{ 0.0f, -0.070000f },
		// Letter O - Outer outline (offset)
		{ 0.000000f, -0.070000f }, { 0.004444f, -0.070000f }, { 0.008889f,
				-0.070000f }, { 0.013333f, -0.070000f },
		{ 0.017778f, -0.070000f }, { 0.022222f, -0.070000f }, { 0.026667f,
				-0.070000f }, { 0.031111f, -0.070000f },
		{ 0.035556f, -0.070000f }, { 0.040000f, -0.070000f }, { 0.044444f,
				-0.070000f }, { 0.048889f, -0.070000f },
		{ 0.053333f, -0.070000f }, { 0.057778f, -0.070000f }, { 0.062222f,
				-0.070000f }, { 0.066667f, -0.070000f },
		{ 0.071111f, -0.070000f }, { 0.075556f, -0.070000f }, { 0.080000f,
				-0.070000f }, { 0.085000f, -0.070000f },
		// Right side going up (offset)
		{ 0.085000f, -0.065556f }, { 0.085000f, -0.061111f }, { 0.085000f,
				-0.056667f }, { 0.085000f, -0.052222f },
		{ 0.085000f, -0.047778f }, { 0.085000f, -0.043333f }, { 0.085000f,
				-0.038889f }, { 0.085000f, -0.034444f },
		{ 0.085000f, -0.030000f },
		// Top edge going left (offset)
		{ 0.080000f, -0.030000f }, { 0.075556f, -0.030000f }, { 0.071111f,
				-0.030000f }, { 0.066667f, -0.030000f },
		{ 0.062222f, -0.030000f }, { 0.057778f, -0.030000f }, { 0.053333f,
				-0.030000f }, { 0.048889f, -0.030000f },
		{ 0.044444f, -0.030000f }, { 0.040000f, -0.030000f }, { 0.035556f,
				-0.030000f }, { 0.031111f, -0.030000f },
		{ 0.026667f, -0.030000f }, { 0.022222f, -0.030000f }, { 0.017778f,
				-0.030000f }, { 0.013333f, -0.030000f },
		{ 0.008889f, -0.030000f }, { 0.004444f, -0.030000f }, { 0.000000f,
				-0.030000f },
		// Left side going down (offset)
		{ 0.000000f, -0.034444f }, { 0.000000f, -0.038889f }, { 0.000000f,
				-0.043333f }, { 0.000000f, -0.047778f },
		{ 0.000000f, -0.052222f }, { 0.000000f, -0.056667f }, { 0.000000f,
				-0.061111f }, { 0.000000f, -0.065556f },
		{ 0.000000f, -0.070000f },
		// Inner hole - move to inner starting point (offset)
		{ 0.020000f, -0.060000f },
		// Inner outline (clockwise to create hole) (offset)
		{ 0.020000f, -0.055556f }, { 0.020000f, -0.051111f }, { 0.020000f,
				-0.046667f }, { 0.020000f, -0.042222f },
		{ 0.020000f, -0.040000f }, { 0.024444f, -0.040000f }, { 0.028889f,
				-0.040000f }, { 0.033333f, -0.040000f },
		{ 0.037778f, -0.040000f }, { 0.042222f, -0.040000f }, { 0.046667f,
				-0.040000f }, { 0.051111f, -0.040000f },
		{ 0.055556f, -0.040000f }, { 0.060000f, -0.040000f }, { 0.065000f,
				-0.040000f }, { 0.065000f, -0.042222f },
		{ 0.065000f, -0.046667f }, { 0.065000f, -0.051111f }, { 0.065000f,
				-0.055556f }, { 0.065000f, -0.060000f },
		{ 0.060000f, -0.060000f }, { 0.055556f, -0.060000f }, { 0.051111f,
				-0.060000f }, { 0.046667f, -0.060000f },
		{ 0.042222f, -0.060000f }, { 0.037778f, -0.060000f }, { 0.033333f,
				-0.060000f }, { 0.028889f, -0.060000f },
		{ 0.024444f, -0.060000f }, { 0.020000f, -0.060000f }, { 0.000000f,
				-0.060000f },

//		//underscore
////		{0.000000f, 0.1700000f},{0.0000000f, 0.200000f},

//		//underscore (offset)
		{ 0.000000f, 0.0000000f }, { 0.005000f, 0.0000000f }, { 0.010000f,
				0.0000000f }, { 0.010000f, 0.0050000f },
		{ 0.010000f, 0.0100000f }, { 0.010000f, 0.0150000f }, { 0.010000f,
				0.0200000f }, { 0.005000f, 0.0200000f },
		{ 0.000000f, 0.0200000f }, { 0.000000f, 0.0150000f }, { 0.000000f,
				0.0100000f }, { 0.000000f, 0.0050000f },
		{ 0.000000f, 0.0000000f },

//		// Move to G
////		{ 0.0f, 0.21f }
//		// Letter G
////		{0.085000f, 0.210f},{0.085000f, 0.218f},
////		{0.085000f, 0.226f},{0.085000f, 0.234f},
////		{0.085000f, 0.242f},{0.085000f, 0.250f},
////		{0.082000f, 0.250f},{0.079000f, 0.250f},
////		{0.076000f, 0.250f},{0.073000f, 0.250f},
////		{0.070000f, 0.250f},{0.070000f, 0.244f},
////		{0.070000f, 0.238f},{0.070000f, 0.232f},
////		{0.070000f, 0.226f},{0.070000f, 0.220f},
////		{0.058000f, 0.220f},{0.046000f, 0.220f},
////		{0.034000f, 0.220f},{0.022000f, 0.220f},
////		{0.010000f, 0.220f},{0.010000f, 0.223f},
////		{0.010000f, 0.226f},{0.010000f, 0.229f},
////		{0.010000f, 0.232f},{0.010000f, 0.235f},
////		{0.014000f, 0.235f},{0.018000f, 0.235f},
////		{0.022000f, 0.235f},{0.026000f, 0.235f},
////		{0.030000f, 0.235f},{0.030000f, 0.233f},
////		{0.030000f, 0.231f},{0.030000f, 0.229f},
////		{0.030000f, 0.227f},{0.030000f, 0.225f},
////		{0.033000f, 0.225f},{0.036000f, 0.225f},
////		{0.039000f, 0.225f},{0.042000f, 0.225f},
////		{0.045000f, 0.225f},{0.045000f, 0.230f},
////		{0.045000f, 0.235f},{0.045000f, 0.240f},
////		{0.045000f, 0.245f},{0.045000f, 0.250f},
////		{0.036000f, 0.250f},{0.027000f, 0.250f},
////		{0.018000f, 0.250f},{0.009000f, 0.250f},
////		{0.000000f, 0.250f},{0.000000f, 0.242f},
////		{0.000000f, 0.234f},{0.000000f, 0.226f},
////		{0.000000f, 0.218f},{0.000000f, 0.210f},
////
		// Move to G (offset)
		{ 0.000000f, 0.030f },
		// Letter G (offset)
		{ 0.009444f, 0.030f }, { 0.018889f, 0.030f }, { 0.028333f, 0.030f }, {
				0.037778f, 0.030f }, { 0.047222f, 0.030f },
		{ 0.056667f, 0.030f }, { 0.066111f, 0.030f }, { 0.075556f, 0.030f }, {
				0.085000f, 0.030f }, { 0.085000f, 0.038f },
		{ 0.085000f, 0.046f }, { 0.085000f, 0.054f }, { 0.085000f, 0.062f }, {
				0.085000f, 0.070f }, { 0.082000f, 0.070f },
		{ 0.079000f, 0.070f }, { 0.076000f, 0.070f }, { 0.073000f, 0.070f }, {
				0.070000f, 0.070f }, { 0.070000f, 0.064f },
		{ 0.070000f, 0.058f }, { 0.070000f, 0.052f }, { 0.070000f, 0.046f }, {
				0.070000f, 0.040f }, { 0.058000f, 0.040f },
		{ 0.046000f, 0.040f }, { 0.034000f, 0.040f }, { 0.022000f, 0.040f }, {
				0.010000f, 0.040f }, { 0.010000f, 0.043f },
		{ 0.010000f, 0.046f }, { 0.010000f, 0.049f }, { 0.010000f, 0.052f }, {
				0.010000f, 0.055f }, { 0.014000f, 0.055f },
		{ 0.018000f, 0.055f }, { 0.022000f, 0.055f }, { 0.026000f, 0.055f }, {
				0.030000f, 0.055f }, { 0.030000f, 0.053f },
		{ 0.030000f, 0.051f }, { 0.030000f, 0.049f }, { 0.030000f, 0.047f }, {
				0.030000f, 0.045f }, { 0.033000f, 0.045f },
		{ 0.036000f, 0.045f }, { 0.039000f, 0.045f }, { 0.042000f, 0.045f }, {
				0.045000f, 0.045f }, { 0.045000f, 0.050f },
		{ 0.045000f, 0.055f }, { 0.045000f, 0.060f }, { 0.045000f, 0.065f }, {
				0.045000f, 0.070f }, { 0.036000f, 0.070f },
		{ 0.027000f, 0.070f }, { 0.018000f, 0.070f }, { 0.009000f, 0.070f }, {
				0.000000f, 0.070f }, { 0.000000f, 0.030f },
//
////		// Move to 0(zero)
////		{0.0f, 0.260000f},
////		// Letter 0(zero)
////		{0.000000f, 0.260000f},{0.004444f, 0.260000f},
////		{0.008889f, 0.260000f},{0.013333f, 0.260000f},
////		{0.017778f, 0.260000f},{0.022222f, 0.260000f},
////		{0.026667f, 0.260000f},{0.031111f, 0.260000f},
////		{0.035556f, 0.260000f},{0.040000f, 0.260000f},
////		{0.044444f, 0.260000f},{0.048889f, 0.260000f},
////		{0.053333f, 0.260000f},{0.057778f, 0.260000f},
////		{0.062222f, 0.260000f},{0.066667f, 0.260000f},
////		{0.071111f, 0.260000f},{0.075556f, 0.260000f},
////		{0.080000f, 0.260000f},{0.085000f, 0.260000f},
////		{0.085000f, 0.264444f},{0.085000f, 0.268889f},
////		{0.085000f, 0.273333f},{0.085000f, 0.277778f},
////		{0.085000f, 0.282222f},{0.085000f, 0.286667f},
////		{0.085000f, 0.291111f},{0.085000f, 0.295556f},
////		{0.085000f, 0.300000f},
////		{0.080000f, 0.300000f},{0.075556f, 0.300000f},
////		{0.071111f, 0.300000f},{0.066667f, 0.300000f},
////		{0.062222f, 0.300000f},{0.057778f, 0.300000f},
////		{0.053333f, 0.300000f},{0.048889f, 0.300000f},
////		{0.044444f, 0.300000f},{0.040000f, 0.300000f},
////		{0.035556f, 0.300000f},{0.031111f, 0.300000f},
////		{0.026667f, 0.300000f},{0.022222f, 0.300000f},
////		{0.017778f, 0.300000f},{0.013333f, 0.300000f},
////		{0.008889f, 0.300000f},{0.004444f, 0.300000f},
////		{0.000000f, 0.300000f},
////		{0.000000f, 0.295556f},{0.000000f, 0.291111f},
////		{0.000000f, 0.286667f},{0.000000f, 0.282222f},
////		{0.000000f, 0.277778f},{0.000000f, 0.273333f},
////		{0.000000f, 0.268889f},{0.000000f, 0.264444f},
////		{0.000000f, 0.260000f},
////		// Inner letter 0(zero)
////		{0.020000f, 0.270000f},
////		{0.020000f, 0.274444f},{0.020000f, 0.278889f},
////		{0.020000f, 0.283333f},{0.020000f, 0.287778f},
////		{0.020000f, 0.290000f},{0.024444f, 0.290000f},
////		{0.028889f, 0.290000f},{0.033333f, 0.290000f},
////		{0.037778f, 0.290000f},{0.042222f, 0.290000f},
////		{0.046667f, 0.290000f},{0.051111f, 0.290000f},
////		{0.055556f, 0.290000f},{0.060000f, 0.290000f},
////		{0.065000f, 0.290000f},{0.065000f, 0.287778f},
////		{0.065000f, 0.283333f},{0.065000f, 0.278889f},
////		{0.065000f, 0.274444f},{0.065000f, 0.270000f},
////		{0.060000f, 0.270000f},{0.055556f, 0.270000f},
////		{0.051111f, 0.270000f},{0.046667f, 0.270000f},
////		{0.042222f, 0.270000f},{0.037778f, 0.270000f},
////		{0.033333f, 0.270000f},{0.028889f, 0.270000f},
////		{0.024444f, 0.270000f},{0.020000f, 0.270000f},
//
		// Move to O (zero)offset
		{ 0.000000f, 0.080000f },
		// Letter O - Outer outline
		{ 0.000000f, 0.080000f }, { 0.004444f, 0.080000f }, { 0.008889f,
				0.080000f }, { 0.013333f, 0.080000f }, { 0.017778f, 0.080000f },
		{ 0.022222f, 0.080000f }, { 0.026667f, 0.080000f }, { 0.031111f,
				0.080000f }, { 0.035556f, 0.080000f }, { 0.040000f, 0.080000f },
		{ 0.044444f, 0.080000f }, { 0.048889f, 0.080000f }, { 0.053333f,
				0.080000f }, { 0.057778f, 0.080000f }, { 0.062222f, 0.080000f },
		{ 0.066667f, 0.080000f }, { 0.071111f, 0.080000f }, { 0.075556f,
				0.080000f }, { 0.080000f, 0.080000f }, { 0.085000f, 0.080000f },
		// Right side going up
		{ 0.085000f, 0.084444f }, { 0.085000f, 0.088889f }, { 0.085000f,
				0.093333f }, { 0.085000f, 0.097778f }, { 0.085000f, 0.102222f },
		{ 0.085000f, 0.106667f }, { 0.085000f, 0.111111f }, { 0.085000f,
				0.115556f }, { 0.085000f, 0.120000f },
		// Top edge going left
		{ 0.080000f, 0.120000f }, { 0.075556f, 0.120000f }, { 0.071111f,
				0.120000f }, { 0.066667f, 0.120000f }, { 0.062222f, 0.120000f },
		{ 0.057778f, 0.120000f }, { 0.053333f, 0.120000f }, { 0.048889f,
				0.120000f }, { 0.044444f, 0.120000f }, { 0.040000f, 0.120000f },
		{ 0.035556f, 0.120000f }, { 0.031111f, 0.120000f }, { 0.026667f,
				0.120000f }, { 0.022222f, 0.120000f }, { 0.017778f, 0.120000f },
		{ 0.013333f, 0.120000f }, { 0.008889f, 0.120000f }, { 0.004444f,
				0.120000f }, { 0.000000f, 0.120000f },
		// Left side going down
		{ 0.000000f, 0.115556f }, { 0.000000f, 0.111111f }, { 0.000000f,
				0.106667f }, { 0.000000f, 0.102222f }, { 0.000000f, 0.097778f },
		{ 0.000000f, 0.093333f }, { 0.000000f, 0.088889f }, { 0.000000f,
				0.084444f }, { 0.000000f, 0.080000f },
		// Inner hole - move to inner starting point
		{ 0.020000f, 0.090000f },
		// Inner outline (clockwise to create hole)
		{ 0.020000f, 0.094444f }, { 0.020000f, 0.098889f }, { 0.020000f,
				0.103333f }, { 0.020000f, 0.107778f }, { 0.020000f, 0.110000f },
		{ 0.024444f, 0.110000f }, { 0.028889f, 0.110000f }, { 0.033333f,
				0.110000f }, { 0.037778f, 0.110000f }, { 0.042222f, 0.110000f },
		{ 0.046667f, 0.110000f }, { 0.051111f, 0.110000f }, { 0.055556f,
				0.110000f }, { 0.060000f, 0.110000f }, { 0.065000f, 0.110000f },
		{ 0.065000f, 0.107778f }, { 0.065000f, 0.103333f }, { 0.065000f,
				0.098889f }, { 0.065000f, 0.094444f }, { 0.065000f, 0.090000f },
		{ 0.060000f, 0.090000f }, { 0.055556f, 0.090000f }, { 0.051111f,
				0.090000f }, { 0.046667f, 0.090000f }, { 0.042222f, 0.090000f },
		{ 0.037778f, 0.090000f }, { 0.033333f, 0.090000f }, { 0.028889f,
				0.090000f }, { 0.024444f, 0.090000f }, { 0.020000f, 0.090000f },
//
		// move to Number 8 (offset)
		{ 0.000000f, 0.140000f },
		// Number 8
//		{0.00f, 0.16f},{0.01f, 0.17f},{0.075f, 0.17f},{0.085f, 0.16f},{0.085f, 0.14f},{0.075f, 0.13f},{0.01f, 0.13f},{0.00f, 0.14f},
//		//Inner Number 8
//		{0.0525f, 0.14f},{0.075f, 0.14f},{0.075f, 0.16f},{0.0525f, 0.16f},{0.0525f, 0.14f},{0.01f, 0.14f},{0.0325f, 0.14f},{0.0325f, 0.16f},{0.01f, 0.16f},{0.01f, 0.14f},{0.0f, 0.14f},

		//Number 8 (offset)
		{ 0.000000f, 0.160000f }, { 0.010000f, 0.170000f }, { 0.031667f,
				0.170000f }, { 0.053333f, 0.170000f }, { 0.075000f, 0.170000f },
		{ 0.085000f, 0.160000f }, { 0.085000f, 0.153333f }, { 0.085000f,
				0.146667f }, { 0.085000f, 0.140000f }, { 0.075000f, 0.130000f },
		{ 0.053333f, 0.130000f }, { 0.031667f, 0.130000f }, { 0.010000f,
				0.130000f }, { 0.000000f, 0.140000f },
		//Inner number 8 (offset)
		{ 0.017500f, 0.140000f }, { 0.035000f, 0.140000f }, { 0.052500f,
				0.140000f }, { 0.060000f, 0.140000f }, { 0.067500f, 0.140000f },
		{ 0.075000f, 0.140000f }, { 0.075000f, 0.146667f }, { 0.075000f,
				0.153333f }, { 0.075000f, 0.160000f }, { 0.067500f, 0.160000f },
		{ 0.060000f, 0.160000f }, { 0.052500f, 0.160000f }, { 0.052500f,
				0.153333f }, { 0.052500f, 0.146667f }, { 0.052500f, 0.140000f },
		{ 0.038333f, 0.140000f }, { 0.024167f, 0.140000f }, { 0.010000f,
				0.140000f }, { 0.017500f, 0.140000f }, { 0.025000f, 0.140000f },
		{ 0.032500f, 0.140000f }, { 0.032500f, 0.146667f }, { 0.032500f,
				0.153333f }, { 0.032500f, 0.160000f }, { 0.025000f, 0.160000f },
		{ 0.017500f, 0.160000f }, { 0.010000f, 0.160000f }, { 0.010000f,
				0.153333f }, { 0.010000f, 0.146667f }, { 0.010000f, 0.140000f },
		{ 0.006667f, 0.140000f }, { 0.003333f, 0.140000f }, { 0.000000f,
				0.140000f } };
void WriteFIBO_G08_Optimized(void) {
	static uint32_t last_update_time = 0;
	static uint32_t point_index = 0;
	static uint8_t initialization_complete = 0;
	static uint32_t pattern_start_time = 0;
	static uint8_t movement_started = 0;

	uint32_t current_time = HAL_GetTick();

	if (!initialization_complete) {
		main_servo = 1;

		int total_points = sizeof(FIBO_G08_complete)
				/ sizeof(FIBO_G08_complete[0]);
		PreprocessTextPath(FIBO_G08_complete, total_points, optimized_points);

		AdjustPIDForDrawing(0);
		drawing_mode.fast_mode_active = 1;

		lookahead_buffer.count = 0;
		lookahead_buffer.index = 0;

		ResetPIDIntegrals();

		initialization_complete = 1;
		pattern_start_time = current_time;

		return;
	}

	if (!movement_started && (current_time - pattern_start_time < 200)) {
		return;
	}
	movement_started = 1;

	uint32_t update_interval = drawing_mode.update_rate_ms; // Base 5ms (200Hz)

	int total_points = sizeof(FIBO_G08_complete) / sizeof(FIBO_G08_complete[0]);

	if (point_index < total_points) {
		OptimizedPoint current_point = optimized_points[point_index];

		if (current_point.is_corner) {
			update_interval = 2;
		} else if (current_point.speed_factor < 0.5f) {
			update_interval = 3;
		}
	}

	if (current_time - last_update_time < update_interval) {
		return;
	}
	last_update_time = current_time;

	if (point_index < total_points) {
		OptimizedPoint target_point = optimized_points[point_index];

		UpdateLookAheadBuffer(&lookahead_buffer, target_point.pos);

		CartesianPosition target_position;
		if (target_point.is_corner || lookahead_buffer.count < 3) {
			target_position = target_point.pos;
		} else {
			target_position = GetSmoothedPosition(&lookahead_buffer, 0.3f);
		}

		CartesianPosition current_pos = GetCurrentCartesianPosition();
		float cart_vel_x, cart_vel_y;

		CalculateVelocityFeedforward(current_pos, target_position,
				update_interval / 1000.0f, target_point.speed_factor,
				&cart_vel_x, &cart_vel_y);

		JointPosition target_joints = inverse_kinematics(target_position);

		if (fabs(target_joints.revolute_angle) > M_PI
				|| fabs(target_joints.prismatic_pos) > 50.0f) {
			point_index++;
			return;
		}

		// ===== JOINT VELOCITY CALCULATION =====
		float joint_vel_revolute, joint_vel_prismatic;
		CartesianToJointVelocity(cart_vel_x, cart_vel_y,
				revolute_encoder.position, prismatic_encoder.position,
				&joint_vel_revolute, &joint_vel_prismatic);

		// ===== APPLY CONTROL COMMANDS =====
		// Set position setpoints
		revolute_tuning.position_setpoint = target_joints.revolute_angle;
		prismatic_tuning.position_setpoint = target_joints.prismatic_pos;

		// Apply velocity feedforward (scaled down for stability)
		revolute_cascade.velocity_setpoint = joint_vel_revolute * 0.7f;
		prismatic_cascade.velocity_setpoint = joint_vel_prismatic * 0.7f;

		// ===== ADVANCEMENT LOGIC =====
		// Check if close enough to current target to advance
		if (IsPositionReached(target_position)) {
			point_index++;

			// Optional: Small pause at corners for better accuracy
			if (target_point.is_corner && point_index < total_points) {
				last_update_time += 10; // 10ms extra pause at corners
			}
		} else {
			// Also advance if spending too much time on one point (timeout)
			static uint32_t point_start_time = 0;
			static uint32_t last_point_index = 0;

			if (last_point_index != point_index) {
				point_start_time = current_time;
				last_point_index = point_index;
			}

			// Timeout: force advance after 200ms
			if (current_time - point_start_time > 200) {
				point_index++;
			}
		}

	} else {
		// ===== PATTERN COMPLETION =====
		point_index = 0;
		initialization_complete = 0;
		movement_started = 0;

		// Restore original PID parameters
		AdjustPIDForDrawing(0);
		drawing_mode.fast_mode_active = 0;

		// Set pen up (although we're ignoring pen states)
		main_servo = 2;
		PWM_set = 400;

		// Reset system state
		revolute_cascade.velocity_setpoint = 0.0f;
		prismatic_cascade.velocity_setpoint = 0.0f;

		// Pause before restarting pattern
		last_update_time = current_time + 3000; // 3 second pause
	}
}

// Optional: Status monitoring function for debugging
void GetDrawingStatus(uint32_t *current_point, uint32_t *total_points,
		float *completion_percentage, uint8_t *is_active) {
	static uint32_t point_index_monitor = 0;
	int total = sizeof(FIBO_G08_complete) / sizeof(FIBO_G08_complete[0]);

	*current_point = point_index_monitor;
	*total_points = total;
	*completion_percentage = (float) point_index_monitor / total * 100.0f;
	*is_active = drawing_mode.fast_mode_active;
}

// Emergency stop function
void StopDrawing(void) {
	drawing_mode.fast_mode_active = 0;
	AdjustPIDForDrawing(0); // Restore original gains
	revolute_cascade.velocity_setpoint = 0.0f;
	prismatic_cascade.velocity_setpoint = 0.0f;
	main_servo = 2; // Pen up
	PWM_set = 400;
}

//#define FIBO_COMMANDS_COUNT (sizeof(fibo_g08_commands) / sizeof(fibo_g08_commands[0]))

CartesianPosition ConvertGcodeToRobot(float gcode_x, float gcode_y) {
	CartesianPosition robot_pos;

	// Scale and offset to fit robot workspace
	robot_pos.x = (gcode_x * SCALE_FACTOR) + OFFSET_X;
	robot_pos.y = (gcode_y * SCALE_FACTOR) + OFFSET_Y;

	// Validate workspace limits
	float distance = sqrt(
			robot_pos.x * robot_pos.x + robot_pos.y * robot_pos.y);
	if (distance > ROBOT_SAFE_AREA) {
		// Scale down if outside safe area
		float scale_factor = ROBOT_SAFE_AREA / distance;
		robot_pos.x *= scale_factor;
		robot_pos.y *= scale_factor;
	}

	return robot_pos;
}

void ResetRevolutePIDIntegrals(void) {
	revolute_cascade.position_pid.error_integral = 0.0f;
	revolute_cascade.position_pid.error_prev = 0.0f;
	revolute_cascade.position_pid.last_derivative = 0.0f;

	revolute_cascade.velocity_pid.prev_error = 0.0f;
	revolute_cascade.velocity_pid.prev_prev_error = 0.0f;
	revolute_cascade.velocity_pid.prev_output = 0.0f;

	revolute_cascade.position_setpoint = revolute_encoder.position;
	revolute_cascade.velocity_setpoint = 0.0f;
	revolute_cascade.current_setpoint = 0.0f;
}

void ResetPrismaticPIDIntegrals(void) {
	prismatic_cascade.position_pid.error_integral = 0.0f;
	prismatic_cascade.position_pid.error_prev = 0.0f;
	prismatic_cascade.position_pid.last_derivative = 0.0f;

	prismatic_cascade.velocity_pid.prev_error = 0.0f;
	prismatic_cascade.velocity_pid.prev_prev_error = 0.0f;
	prismatic_cascade.velocity_pid.prev_output = 0.0f;

	prismatic_cascade.position_setpoint = prismatic_encoder.position;
	prismatic_cascade.velocity_setpoint = 0.0f;
	prismatic_cascade.current_setpoint = 0.0f;
}

void ResetPIDIntegrals(void) {

	revolute_cascade.position_pid.error_integral = 0.0f;
	revolute_cascade.position_pid.error_prev = 0.0f;
	revolute_cascade.position_pid.last_derivative = 0.0f;

	revolute_cascade.velocity_pid.prev_error = 0.0f;
	revolute_cascade.velocity_pid.prev_prev_error = 0.0f;
	revolute_cascade.velocity_pid.prev_output = 0.0f;

	prismatic_cascade.position_pid.error_integral = 0.0f;
	prismatic_cascade.position_pid.error_prev = 0.0f;
	prismatic_cascade.position_pid.last_derivative = 0.0f;

	prismatic_cascade.velocity_pid.prev_error = 0.0f;
	prismatic_cascade.velocity_pid.prev_prev_error = 0.0f;
	prismatic_cascade.velocity_pid.prev_output = 0.0f;

	revolute_cascade.position_setpoint = revolute_encoder.position;
	revolute_cascade.velocity_setpoint = 0.0f;
	revolute_cascade.current_setpoint = 0.0f;

	prismatic_cascade.position_setpoint = prismatic_encoder.position;
	prismatic_cascade.velocity_setpoint = 0.0f;
	prismatic_cascade.current_setpoint = 0.0f;

	revolute_cascade.last_position_update_time = HAL_GetTick();
	revolute_cascade.last_velocity_update_time = HAL_GetTick();

	prismatic_cascade.last_position_update_time = HAL_GetTick();
	prismatic_cascade.last_velocity_update_time = HAL_GetTick();

}
void WriteFIBO_G08_Pattern(void) {
	static uint32_t last_move_time = 0;
	static uint32_t point_index = 0;
	static uint32_t total_points = sizeof(FIBO_G08_complete)
			/ sizeof(FIBO_G08_complete[0]);

//	total_points = total_points*8;

	if (IsCoordinatedMoveComplete()
			&& (abs(joints.revolute_angle - revolute_encoder.position) < 0.25f)
			&& (abs(joints.prismatic_pos - prismatic_encoder.position) < 0.25f)) {
		if (point_index < total_points) {
			ResetPrismaticPIDIntegrals();

			CartesianPosition target = FIBO_G08_complete[point_index];

//			if (ValidateCartesianTarget(target.x, target.y)) {
			MoveToCartesianPositionSTM32(target.x, target.y);

//			}
			point_index++;
		} else {
			point_index = 0;
			ResetPrismaticPIDIntegrals();
			last_move_time = HAL_GetTick() + 5000;
		}
		last_move_time = HAL_GetTick();
	}
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
	MX_TIM8_Init();
	/* USER CODE BEGIN 2 */

	InitializeRevoluteTuning();
	InitializePrismaticTuning();

	EnableControl();
	InitializeTrajectoryGenerators();
	//	UpdateTuningParameters(5.0f, 0.000000000000000000000000000f, 0.000000f, 0.8475f,0.00625f, 0.0f); // rever

	//	UpdateTuningParameters(5.575f, 0.0455f, 2.5f, 0.000001f, 1.1f, 0.0f);
	//	UpdateTuningParameters(0.0f, 0.0f, 0.0f, 0.3f, 20.0f, 0.0f);

//		UpdateTuningParameters(25.575f, 0.0755f, 2.5f, 0.75f, 0.015f, 0.0f);  //pris

	UpdateRevoluteTuningParameters(7.1f, 0.049f, 2.5f, 0.22f, 0.012f, 0.0f); // new
////	UpdateRevoluteTuningParameters(5.0f, 0.0f,0.0f, 0.8475f, 0.00625f, 0.0f);
////	UpdatePrismaticTuningParameters(25.575f, 0.0755f, 2.8f, 1.2f, 0.6f, 0.0f);
	UpdatePrismaticTuningParameters(4.5f, 0.0018f, 1.75f, 0.43f, 0.041f, 0.0f); // new
//	UpdatePrismaticTuningParameters(8.0f, 0.02f, 0.5f, 0.2f, 0.003f, 0.0f);

//	UpdateRevoluteTuningParameters(5.0f, 0.0000000000000000000000000001f,0.000000f, 0.8475f, 0.00625f, 0.0f); // OLD
	//	UpdatePrismaticTuningParameters(25.575f, 0.0755f, 2.8f, 1.2f, 0.6f, 0.0f);
//		UpdatePrismaticTuningParameters(25.575f, 0.0755f, 2.5f, 0.63f, 0.3f, 0.0f);
//	UpdatePrismaticTuningParameters(25.575f, 0.0755f, 2.5f, 0.63f, 0.02f, 0.0f); // OLD
	// Set pen to DOWN (always drawing)

//    revolute_tuning.position_setpoint = 3.14;
//	DrawCircle_Start(50.0f, 50.0f);
//
//	WorkspacePoint custom_points[10] = { { 0.3f, 0.0f },         // 0°
//			{ 0.27f, 0.13f },       // 36°
//			{ 0.15f, 0.26f },       // 72°
//			{ 0.0f, 0.3f },         // 108°
//			{ -0.15f, 0.26f },      // 144°
//			{ -0.27f, 0.13f },      // 180°
//			{ -0.3f, 0.0f },        // 216°
//			{ -0.27f, -0.13f },     // 252°
//			{ -0.15f, -0.26f },     // 288°
//			{ 0.0f, -0.3f }         // 324°
//	};
//
//	PointTraversal_Init(&custom_test, custom_points, 10, 1.5f, 2.0f);
//	PointTraversal_Start(&custom_test);

//	SetPrismaticTargetToZero();
//	SmoothCircleDrawer_Init(&circle, 0.0f, 0.0f, 0.3f, 1.5f, 1.0f);
//
//	// Start drawing when needed
//	SmoothCircleDrawer_Start(&circle);

	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2);

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
		if (main_servo == 1) {
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 200);
		} else if (main_servo == 2) {
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, PWM_set);
		}

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
 * @brief TIM8 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM8_Init(void) {

	/* USER CODE BEGIN TIM8_Init 0 */

	/* USER CODE END TIM8_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM8_Init 1 */

	/* USER CODE END TIM8_Init 1 */
	htim8.Instance = TIM8;
	htim8.Init.Prescaler = 1699;
	htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim8.Init.Period = 1999;
	htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim8.Init.RepetitionCounter = 0;
	htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim8) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim8) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2)
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
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM8_Init 2 */

	/* USER CODE END TIM8_Init 2 */
	HAL_TIM_MspPostInit(&htim8);

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
	htim17.Init.Period = 2005;
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

//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
//    if (htim == &htim4) {
//        // Run control loops
//        RevoluteControlLoopWithTrajectory();
//        PrismaticControlLoopWithTrajectory();
//
//        // Coordinated movement sequence
//        static uint32_t last_coordinated_move = 0;
//        static uint8_t coord_index = 0;
//
//        // Define coordinated positions
//        typedef struct {
//            float revolute_pos;
//            float prismatic_pos;
//        } CoordPosition;
//
////        static CoordPosition coord_positions[] = {
////            {0.0f,      -0.3/8/RAD_PER_DEGREE},      // Start
////            {M_PI/4.0f, -0.3*4/8/RAD_PER_DEGREE},      // 45° with forward
////            {M_PI/2.0f, -0.3/8/RAD_PER_DEGREE},    // 90° with slight back
////            {M_PI/6.0f, -0.3*4/8/RAD_PER_DEGREE},     // 30° with slight forward
////            {0.0f,     -0.3/8/RAD_PER_DEGREE}       // Back to start
////        };
//
//        static CoordPosition coord_positions[] = {
//                    {0.0f,      -0.0f},      // Start
//                    {M_PI/4.0f, -M_PI*2/4.0f},      // 45° with forward
//                    {M_PI/2.0f, -M_PI*2/2.0f},    // 90° with slight back
//                    {M_PI/6.0f, -M_PI*2/6.0f},     // 30° with slight forward
//                    {0.0f,     -0.0f}       // Back to start
//                };
//
//
//        static const uint8_t num_coord_positions = sizeof(coord_positions) / sizeof(coord_positions[0]);
//
//        if (IsCoordinatedMoveComplete() && (HAL_GetTick() - last_coordinated_move > 2500)) {
//            coord_index = (coord_index + 1) % num_coord_positions;
//
//            SetTrajectoryParameters(&revolute_trajectory, 1.0f, 0.4f);
//            SetTrajectoryParameters(&prismatic_trajectory, 30.0f, 15.0f);
//
//            MoveToCoordinatedPosition(coord_positions[coord_index].revolute_pos,
//                                    coord_positions[coord_index].prismatic_pos);
//
//            last_coordinated_move = HAL_GetTick();
//        }
//    }
//}
//void TestCoordinatedMovement(void) {
//    static uint32_t last_move_time = 0;
//    static uint8_t move_index = 0;
//
//    // ปรับตำแหน่งให้เป็นลำดับ step ตามที่ต้องการ
//    static float prismatic_positions[] = {
//        -0.05 * 2.0 * M_PI / 0.08,    // Step 1
//        -0.10 * 2.0 * M_PI / 0.08,    // Step 2
//        -0.20 * 2.0 * M_PI / 0.08,    // Step 3
//    };
//
//    static const uint8_t num_positions = sizeof(prismatic_positions) / sizeof(prismatic_positions[0]);
//
//    // ทุก 3 วินาทีให้ไปยังตำแหน่งถัดไป
//    if (IsCoordinatedMoveComplete() && (HAL_GetTick() - last_move_time > 3000)) {
//        move_index = (move_index + 1) % num_positions;
//
//        // ตั้งค่า Trajectory ใหม่
//        SetTrajectoryParameters(&prismatic_trajectory, 40.0f, 20.0f);
//        MovePrismaticToPositionNew(prismatic_positions[move_index]);
//
//        // อัปเดตเวลา
//        last_move_time = HAL_GetTick();
//    }
//}
void TestCoordinatedMovementFixed(void) {
	static uint32_t last_move_time = 0;
	static uint8_t move_index = 0;

	// Define simultaneous movements for both joints
	static CoordPosition coord_positions[] = { { 0.0f, 0.0f },   // Both at zero

//			{ M_PI / 2.0f,  0}, // Both move together
			{ -M_PI / 2.0f, (-0.1 * 2.0 * M_PI / 0.08) - M_PI / 2.0f }, // Both move together
//			{ M_PI / 6.0f,  (-0.30 * 2.0 * M_PI / 0.08) } ,	// Both return to zero
			{ -M_PI / 6.0f, (-0.20 * 2.0 * M_PI / 0.08) - M_PI / 6.0f }
//			{ -M_PI / 2.0f,  -0.1  * 2.0* M_PI / 0.08 }
			, { 0.0f, 0.0f } };

//
//    static CoordPosition coord_positions[] = {
//            { 0.0f,      0.0f },                    // Both at zero
//            { M_PI/4.0f, -M_PI/4.0f}, // Both move together
//            { M_PI/2.0f, - M_PI/2.0f},  // Both move together
//            { 0.0f,      -0.0f }                     // Both return to zero
//        };

	static const uint8_t num_positions = sizeof(coord_positions)
			/ sizeof(coord_positions[0]);

//	if (IsCoordinatedMoveComplete() && (abs(prismatic_encoder.position - coord_positions[move_index].prismatic_pos) < 0.25f) &&(abs(revolute_encoder.position - coord_positions[move_index].revolute_pos) < 0.2f)) {
	if (IsCoordinatedMoveComplete()
			&& (abs(
					revolute_encoder.position
							- coord_positions[move_index].revolute_pos) < 0.1f)&& (abs(
									prismatic_encoder.position
											- coord_positions[move_index].prismatic_pos) < 0.005f)) {

		move_index = (move_index + 1) % num_positions;
		SetTrajectoryParameters(&revolute_trajectory, 1.0f, 0.4f);
		SetTrajectoryParameters(&prismatic_trajectory, 40.0f, 100.0f);

		MoveToCoordinatedPositionSynced(
				coord_positions[move_index].revolute_pos,
				coord_positions[move_index].prismatic_pos);

		last_move_time = HAL_GetTick();
	}
}

//void TestCoordinatedMovementFixed(void) {
//    static uint32_t last_move_time = 0;
//    static uint8_t move_index = 0;
//
//    static CoordPosition coord_positions[] = {
//        { 0.0f, 0.0f },                                    // Start position - both at zero
//        { M_PI/6.0f, -0.05 * 2.0 * M_PI / 0.08f },       // 30° revolute + 5cm prismatic
//        { M_PI/4.0f, -0.10 * 2.0 * M_PI / 0.08f },       // 45° revolute + 10cm prismatic
//        { M_PI/3.0f, -0.15 * 2.0 * M_PI / 0.08f },       // 60° revolute + 15cm prismatic
//        { M_PI/2.0f, -0.20 * 2.0 * M_PI / 0.08f },       // 90° revolute + 20cm prismatic
//        { M_PI/4.0f, -0.10 * 2.0 * M_PI / 0.08f },       // Return to 45°
//        { 0.0f, 0.0f }                                     // Return to start
//    };
//
//    static const uint8_t num_positions = sizeof(coord_positions) / sizeof(coord_positions[0]);
//
//    bool revolute_reached = fabs(revolute_encoder.position - coord_positions[move_index].revolute_pos) < 0.05f;
//    bool prismatic_reached = fabs(prismatic_encoder.position - coord_positions[move_index].prismatic_pos) < 0.5f;
//
//    if (IsCoordinatedMoveComplete() && revolute_reached && prismatic_reached ) {
//
//        move_index = (move_index + 1) % num_positions;
//
//        SetTrajectoryParameters(&revolute_trajectory, 1.0f, 0.4f);
//        SetTrajectoryParameters(&prismatic_trajectory, 40.0f, 100.0f);
//
//        MoveToCoordinatedPositionSynced(
//            coord_positions[move_index].revolute_pos,
//            coord_positions[move_index].prismatic_pos);
//
//        last_move_time = HAL_GetTick();
//    }
//}

//void TestCoordinatedMovementFixed(void) {
//    static uint32_t last_coordinated_move = 0;
//    static uint8_t coord_index = 0;
//
//    // กำหนดตำแหน่งแต่ละจุด (revolute, prismatic)
//    static CoordPosition coord_positions[] = {
//        { 0.0f, -0.0f },                              // Position 1
//        { M_PI/4.0f, -0.05 * 2.0 * M_PI / 0.08f },   // Position 2
//        { M_PI/2.0f, -0.1 * 2.0 * M_PI / 0.08f },    // Position 3
//        { M_PI/3.0f, -0.2 * 2.0 * M_PI / 0.08f }     // Position 4 (แก้จาก M_PI/1.0f เป็น M_PI/3.0f)
//    };
////    static CoordPosition coord_positions[] = {
////            { 0.0f, -0.0f },                              // Position 1
////            { M_PI/4.0f, -M_PI/4.0f },   // Position 2
////            { M_PI/2.0f, -M_PI/2.0f },    // Position 3
////            { M_PI/3.0f, -M_PI/3.0f }     // Position 4 (แก้จาก M_PI/1.0f เป็น M_PI/3.0f)
////        };
//
//    static const uint8_t num_coord_positions = sizeof(coord_positions) / sizeof(coord_positions[0]);
//
//    if (IsCoordinatedMoveComplete() && (HAL_GetTick() - last_coordinated_move > 2500)) {
//        coord_index = (coord_index + 1) % num_coord_positions;
//
//        SetTrajectoryParameters(&revolute_trajectory, 1.0f, 0.4f);
//        SetTrajectoryParameters(&prismatic_trajectory, 30.0f, 15.0f);
//
//        MoveToCoordinatedPosition(coord_positions[coord_index].revolute_pos,
//                                coord_positions[coord_index].prismatic_pos);
//
//        last_coordinated_move = HAL_GetTick();
//    }
//}
// Timer 1 interrupt callback
void TestCoordinatedMovement(void) {
	static uint32_t last_move_time = 0;
	//
	static float prismatic_positions[] = { -0.025 * 2.0 * M_PI / 0.08, -0.10
			* 2.0 * M_PI / 0.08, -0.2 * 2.0 * M_PI / 0.08 }; // ต้องสร้างอันนี้ขึ้นมาอีกอัน
//	static uint8_t move_index = 0;
	static int8_t direction = 1;  // 1 = forward, -1 = backward
	static const uint8_t num_positions = 3;

	if (IsCoordinatedMoveComplete()
			&& (HAL_GetTick() - last_move_time > 2500)) {

		// เปลี่ยนทิศทางเมื่อถึงขอบ
		if (move_index == num_positions - 1) {
			direction = -1;  // เปลี่ยนเป็นถอยหลัง
		} else if (move_index == 0) {
			direction = 1;   // เปลี่ยนเป็นไปข้างหน้า
		}

		move_index += direction;  // เพิ่มหรือลดตาม direction
		if (move_index >= num_positions) {
			move_index = num_positions - 1;
		}
		SetTrajectoryParameters(&prismatic_trajectory, 40.0f, 20.0f);
		MovePrismaticToPositionNew(prismatic_positions[move_index]);

		last_move_time = HAL_GetTick();
	}
}
//

//		MoveRevoluteToPositionNew(revolute_positions[move_index]);    // จูน revolute เปิดตรงนี้

//  เปิด ตั้งแต่ตรงนี้ มันคือ จูน 2 แกน ให้ทำงานพร้อมกัน

//		static CoordPosition coord_positions[] = { { 0.0f, -0.0f },     //  ใส่เป็น radius นะสูตรอยู่ตรงตาราง
//				{ 0.0f, -0.0f },
//				{ 0.0f, -0.0f },
//				{ 0.00f, -0.0 },
//				{ 0.0f, -0.0f }
//		};
//
//		static const uint8_t num_coord_positions = sizeof(coord_positions)
//				/ sizeof(coord_positions[0]);
//
//		if (IsCoordinatedMoveComplete()
//				&& (HAL_GetTick() - last_coordinated_move > 2500)) {
//			coord_index = (coord_index + 1) % num_coord_positions;
//
//			SetTrajectoryParameters(&revolute_trajectory, 1.0f, 0.4f);
//			SetTrajectoryParameters(&prismatic_trajectory, 30.0f, 15.0f);
//
//			MoveToCoordinatedPosition(coord_positions[coord_index].revolute_pos,
//					coord_positions[coord_index].prismatic_pos);
//
//			last_coordinated_move = HAL_GetTick();
//		}
//
//		last_move_time = HAL_GetTick();
//	}

//  เปิดถึงตรงนี้ ตั้งแต่ตรงนี้ มันคือ จูน 2 แกน ให้ทำงานพร้อมกัน แต่ก็อย่าลืมปิดไอพวกข้างบน
//}

void TestPrismaticFixedVoltage(void) {
	static uint32_t last_change_time = 0;
	static uint8_t voltage_index = 0;

	static float test_voltages[] = { 0.0f,    // หยุด
			1.5f, -1.5f, 3.0f, -3.0f, 6.0f,    // ต่ำ
			-6.0f, 10.0f,   // ปานกลาง
			-10.0f, 12.0f,   // ปานกลางสูง
			-12.0f, 18.0f,   // สูง
			-18.0f, 24.0f,   // สูงสุด
			-24.0f

	};

	static const uint8_t num_voltages = sizeof(test_voltages)
			/ sizeof(test_voltages[0]);

	// เช็คเวลาทุก 1000ms (1 วินาที)
	if (HAL_GetTick() - last_change_time >= 500) {
		last_change_time = HAL_GetTick();

		// เลือก voltage
		float voltage = test_voltages[voltage_index];

		// *** ใช้โค้ดตามที่ผู้ใช้ให้มา ***
		float max_voltage = 24.0f;
		uint32_t period = 999;  // จาก htim1 period

		float duty_cycle = fabsf(voltage) / max_voltage;
		uint32_t pwm_value = (uint32_t) (duty_cycle * period);

		// ตั้งค่า PWM
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_value);

		// ตั้งค่าทิศทาง
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1,
				voltage >= 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);

		// เปลี่ยนไปค่าถัดไป
		voltage_index = (voltage_index + 1) % num_voltages;

	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
//	InitializeRevoluteTuning();
//	    InitializePrismaticTuning();
//	    EnableControl();
//	    UpdateRevoluteTuningParameters(5.0f, 0.00000000000001f, 0.1f, 0.5f,42.0f, 0.0f);
//	    UpdatePrismaticTuningParameters(5.575f, 0.0455f, 2.5f, 0.000001f, 1.1f, 0.0f);
//	revolute_tuning.position_setpoint = 3.14;
//	revolute_cascade.velocity_setpoint  = 1.0f;
//	static bool oscillation_started = false;
	if (htim == &htim17) {
//    	TestRevolutePositionStep();
//    	test += 1;
//        RevoluteControlLoop();
//        PrismaticControlLoop();

	}
	if (htim == &htim4) {
//		TestPrismaticFixedVoltage();
//    	test += 1;
//		revolute_tuning.position_setpoint = position_setpoint_deg
//				* (M_PI / 180.0f);

//			 if (!oscillation_started) {
//			            StartPrismaticOscillation();
//			            oscillation_started = true;
//			        }
//		TestPositionStep();
//		RotateRevoluteJoint();
//		RevoluteControlLoop();
//		PrismaticControlLoop();
//	}

		RevoluteControlLoopWithTrajectory(); // เปิดตรงนี้ ตอนจูน Revolute
		PrismaticControlLoopWithTrajectory();
//		ExecuteCoordPositionsWithInverseKinematics();

//		SimpleCartesianInverseTest();

//		WriteFIBO_G08_FromGcode();

//		WriteFIBO_G08_Pattern();

//		WriteFIBO_G08_Optimized();
//		TestPrismaticFixedVoltage();
		// Single movement test
//		static uint32_t startup_time = 0;
//		static uint8_t movement_done = 0;
//
//		if (startup_time == 0) {
//			startup_time = HAL_GetTick();
//		}
//
//		if (!movement_done && (HAL_GetTick() - startup_time > 2000)) {
//
////			SetTrajectoryParameters(&prismatic_trajectory, 30.0f, 15.0f);
////
////			MovePrismaticToPositionNew(0.3/8/RAD_PER_DEGREE);
////			0.3/8/RAD_PER_DEGREE
//
////			SetTrajectoryParameters(&revolute_trajectory, 1.0f, 0.4f);
////
////			MoveRevoluteToPositionNew(M_PI / 4.0f);  // Move to 45 degrees
////			TestCoordinatedMovement()

		TestCoordinatedMovementFixed();

//			movement_done = 1;  // Only move once
//		}
//
//		if (movement_done && !check_status(&revolute_trajectory)
//				&& (HAL_GetTick() - startup_time > 8000)) {
//			movement_done = 0;
//			startup_time = HAL_GetTick();
//		}
//
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
//		}SetTrajectoryParameters
//	}
void MoveToCoordinatedPositionSynced(float revolute_target,
		float prismatic_target) {
	float revolute_distance = fabsf(
			revolute_target - revolute_encoder.position);
	float prismatic_distance = fabsf(
			prismatic_target - prismatic_encoder.position);

	// If distances are proportional to max speeds, sync is possible
	float distance_ratio = prismatic_distance / revolute_distance;

	if (distance_ratio < 15.0f) {
		float revolute_time = CalculateTrajectoryTime(revolute_distance, 1.0f,
				0.4f);
		float sync_prismatic_vel = fminf(40.0f,
				prismatic_distance / revolute_time);

		SetTrajectoryParameters(&revolute_trajectory, 1.0f, 0.4f);
		SetTrajectoryParameters(&prismatic_trajectory, sync_prismatic_vel,
				100.0f);
	} else {
		SetTrajectoryParameters(&revolute_trajectory, 1.0f, 0.4f);
		SetTrajectoryParameters(&prismatic_trajectory, 40.0f, 100.0f);
	}

	MoveRevoluteToPositionNew(revolute_target);
	MovePrismaticToPositionNew(prismatic_target);
}
//void MoveToCoordinatedPositionSynced(float revolute_target, float prismatic_target) {
//    // Force 40 rad/s without sync calculations
//    SetTrajectoryParameters(&revolute_trajectory, 1.0f, 0.4f);
//    SetTrajectoryParameters(&prismatic_trajectory, 40.0f, 100.0f);
//
//    MoveRevoluteToPositionNew(revolute_target);
//    MovePrismaticToPositionNew(prismatic_target);
//}
//

float CalculateTrajectoryTime(float distance, float max_vel, float max_accel) {
	float accel_time = max_vel / max_accel;
	float accel_distance = 0.5f * max_accel * accel_time * accel_time;

	if (distance > 2.0f * accel_distance) {
		// Trapezoidal profile
		float const_distance = distance - 2.0f * accel_distance;
		return 2.0f * accel_time + (const_distance / max_vel);
	} else {
		// Triangular profile
		return 2.0f * sqrtf(distance / max_accel);
	}
}

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
