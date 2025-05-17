/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "main.h"
#include "arm_math.h"
#include <math.h>
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
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
float move_in_progress;


float position_setpoint_deg = 0.0f;
static float target_velocity = 1.0f;
static uint8_t use_motor_model = 0;
static uint32_t filter_switch_time = 0;

static uint32_t motion_start_time = 0;
static float current_velocity_setpoint = 0.0f;
static uint8_t motion_complete = 0;
static uint8_t motion_direction = 1; // 1 for positive, 0 for negative
static uint8_t profile_state = 0; // 0: accel, 1: constant, 2: decel, 3: rest

float process_noise = 0.15f;
float meas_noise = 0.4f;

// For kinematic model filter
float process_noise_pos = 0.0005f;
float process_noise_vel = 0.05f;

static float test_velocity = 1.0f;
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

/* USER CODE BEGIN PV */

typedef struct {
    TIM_HandleTypeDef* htim;        // Timer handle
    int32_t count;                  // Encoder count
    int32_t last_count;             // Previous encoder count
    int32_t last_count_for_speed;   // Count used for speed calculation
    float position;                 // Current position in radians
    float speed;                    // Current speed in rad/s
    float speed_filtered;           // Filtered speed (original alpha filter)
    uint32_t last_speed_calc_time;
} EncoderData;

typedef struct {
    PIDController position_controller;
    PIDController velocity_controller;
    float velocity_setpoint;
    float current_setpoint;
    float max_velocity;
} CascadeController;


typedef struct {
    /* Position PID parameters */
    float position_kp;
    float position_ki;
    float position_kd;

    /* Velocity PID parameters */
    float velocity_kp;
    float velocity_ki;
    float velocity_kd;

    /* Control limits */
    float max_velocity;        // Maximum velocity setpoint (rad/s)
    float max_voltage;         // Maximum motor voltage

    /* Reference values */
    float position_setpoint;   // Target position in radians

    /* Flags */
    uint8_t control_enabled;   // Control enable flag

    /* PWM settings */
    uint32_t pwm_period;       // PWM timer period

    /* Test parameters */
    uint8_t test_running;
    uint32_t test_start_time;
    float test_amplitude;
    float test_frequency;
} TuningParameters;

typedef struct {
    float R;           // Motor resistance (Ohms)
    float L;           // Motor inductance (H)
    float Ke;          // Back EMF constant (V·s/rad)
    float Kt;          // Torque constant (N·m/A)
    float Jm;          // Motor inertia (kg·m²)
    float Bm;          // Motor friction coefficient (N·m·s/rad)
    float n_gear;      // Gear ratio
    float r_pulley;    // Pulley radius (m)
} MotorParams;


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
} SystemParams;

EncoderData encoder;
CascadeController cascade;
TuningParameters tuning;

MotorParams motorParams;
SystemParams systemParams;
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
    float A[MOTOR_MODEL_NUM_STATES * MOTOR_MODEL_NUM_STATES];  /* Continuous state transition */
    float A_d[MOTOR_MODEL_NUM_STATES * MOTOR_MODEL_NUM_STATES]; /* Discrete state transition */
    float B[MOTOR_MODEL_NUM_STATES * MOTOR_MODEL_NUM_INPUTS];  /* Continuous input matrix */
    float B_d[MOTOR_MODEL_NUM_STATES * MOTOR_MODEL_NUM_INPUTS]; /* Discrete input matrix */
    float C[MOTOR_MODEL_NUM_OUTPUTS * MOTOR_MODEL_NUM_STATES]; /* Output matrix */

    /* Noise matrices */
    float Q[MOTOR_MODEL_NUM_STATES * MOTOR_MODEL_NUM_STATES];  /* Process noise - continuous */
    float Q_d[MOTOR_MODEL_NUM_STATES * MOTOR_MODEL_NUM_STATES]; /* Process noise - discrete */
    float R[MOTOR_MODEL_NUM_OUTPUTS];                          /* Measurement noise */

    /* Kalman gain */
    float K[MOTOR_MODEL_NUM_STATES];

    /* Motor parameters */
    float dt;              /* Sample time (seconds) */
    float J;               /* Motor inertia (kg*m^2) */
    float b;               /* Viscous friction coefficient (N*m*s) */
    float Kt;              /* Torque constant (N*m/A) */
    float Ke;              /* Back-EMF constant (V*s/rad) */
    float Ra;              /* Armature resistance (Ohms) */
    float La;              /* Armature inductance (H) */

    /* Estimated states */
    float position;
    float velocity;
    float load_torque;
    float current;
} MotorModelKalman;

/***************************************************************
 *                  KINEMATIC MODEL FILTER                     *
 ***************************************************************/

/* Number of states in kinematic model */
#define KINEMATIC_MODEL_NUM_STATES 2

/* Kinematic Model Kalman Filter Structure */
typedef struct {
    /* State vector [position; velocity] */
    float X[KINEMATIC_MODEL_NUM_STATES];

    /* State covariance matrix */
    float P[KINEMATIC_MODEL_NUM_STATES * KINEMATIC_MODEL_NUM_STATES];

    /* System matrices */
    float A[KINEMATIC_MODEL_NUM_STATES * KINEMATIC_MODEL_NUM_STATES];  /* State transition */
    float C[KINEMATIC_MODEL_NUM_STATES];                               /* Output matrix */

    /* Noise matrices */
    float Q[KINEMATIC_MODEL_NUM_STATES * KINEMATIC_MODEL_NUM_STATES];  /* Process noise */
    float R;                                                           /* Measurement noise */

    /* Kalman gain */
    float K[KINEMATIC_MODEL_NUM_STATES];

    /* Parameters */
    float dt;                /* Sample time (seconds) */
    float process_noise_pos; /* Process noise for position */
    float process_noise_vel; /* Process noise for velocity */
    float meas_noise;        /* Measurement noise */

    /* Estimated states */
    float position;
    float velocity;
} KinematicModelKalman;

/* Global filter instances */
MotorModelKalman motorFilter;
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

    uint8_t state;             // Current state (0=idle, 1=accel, 2=cruise, 3=decel, 4=done)

    // Output values
    float position;            // Current position (rad)
    float velocity;            // Current velocity (rad/s)
    float acceleration;        // Current acceleration (rad/s²)
} SimpleTrapezoid;

SimpleTrapezoid trap;
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
/* USER CODE BEGIN PFP */
void InitializeTuning(void);
void PID_Init(PIDController* pid, float kp, float ki, float kd, float out_min, float out_max, float sample_time);
float PID_Update(PIDController* pid, float setpoint, float actual);
void Cascade_Init(CascadeController* cascade, float pos_kp, float pos_ki, float pos_kd, float vel_kp, float vel_ki, float vel_kd, float dt);
float Cascade_Update(CascadeController* cascade, float pos_ref, float pos_actual, float vel_actual);
void Encoder_Init(EncoderData* enc, TIM_HandleTypeDef* htim);
void Encoder_Read(EncoderData* enc);
void Encoder_CalculateSpeed(EncoderData* enc);
void SetMotorPWM(float voltage, float max_voltage, uint32_t period);
void ControlLoop(void);
void TestPositionStep(void);
float GenerateTestReference(void);
void robot_start_trajectoryy(void);
void update_velocity_profile(void);

/* Function prototypes for DC Motor Model Kalman filter */
void MotorModelKalman_Init(MotorModelKalman* filter, float dt, float J, float b,
                          float Kt, float Ke, float Ra, float La,
                          float process_noise, float meas_noise);

void MotorModelKalman_DiscretizeSystem(MotorModelKalman* filter);
void MotorModelKalman_Reset(MotorModelKalman* filter);
void MotorModelKalman_Predict(MotorModelKalman* filter, float voltage_input);
void MotorModelKalman_Update(MotorModelKalman* filter, float position_measurement);
void MotorModelKalman_Estimate(MotorModelKalman* filter, float voltage_input, float position_measurement);

/* Function prototypes for Kinematic Model Kalman filter */
void KinematicModelKalman_Init(KinematicModelKalman* filter, float dt,
                               float process_noise_pos, float process_noise_vel,
                               float meas_noise);

void KinematicModelKalman_Reset(KinematicModelKalman* filter);
void KinematicModelKalman_Predict(KinematicModelKalman* filter);
void KinematicModelKalman_Update(KinematicModelKalman* filter, float position_measurement);
void KinematicModelKalman_Estimate(KinematicModelKalman* filter, float position_measurement);

/***************************************************************
 *                   UTILITY FUNCTIONS                         *
 ***************************************************************/

/* Utility matrix functions */
void MatrixMultiply(float* A, float* B, float* C, int rows_a, int cols_a, int cols_b);
void MatrixAdd(float* A, float* B, float* C, int rows, int cols);
void MatrixSubtract(float* A, float* B, float* C, int rows, int cols);
void MatrixTranspose(float* A, float* AT, int rows, int cols);
void MatrixScale(float* A, float* B, float scale, int rows, int cols);
void MatrixCopy(float* src, float* dst, int size);
void MatrixIdentity(float* A, int size);

/* Generate test data for velocity estimation testing */
void GenerateStepTestData(float* position, float* actual_velocity, float* input_voltage,
                         int size, float amplitude, float step_time);
void GenerateRampTestData(float* position, float* actual_velocity, float* input_voltage,
                         int size, float slope, float max_amplitude);
void GenerateSineTestData(float* position, float* actual_velocity, float* input_voltage,
                         int size, float amplitude, float frequency);

/* Testing function to compare filters */
void CompareFilters(float* position_measurements, float* input_voltage, float* actual_velocity, int size);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void PID_Init(PIDController* pid, float kp, float ki, float kd, float out_min, float out_max, float sample_time) {
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


float PID_Update(PIDController* pid, float setpoint, float actual) {
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
        error_derivative = 0.2f * error_derivative + 0.8f * pid->last_derivative;
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


void Cascade_Init(CascadeController* cascade, float pos_kp, float pos_ki, float pos_kd,
                 float vel_kp, float vel_ki, float vel_kd, float dt) {
    PID_Init(&cascade->position_controller, pos_kp, pos_ki, pos_kd, -1.0f, 1.0f, dt);

    PID_Init(&cascade->velocity_controller, vel_kp, vel_ki, vel_kd, -24.0f, 24.0f, dt);

    cascade->velocity_setpoint = 0.0f;
    cascade->current_setpoint = 0.0f;
    cascade->max_velocity = 1.0f;
}


void TestPositionStep(void) {
    static uint32_t last_step_time = 0;
    static uint8_t step_index = 0;
//    tuning.test_running = 1;
    static float position_setpoints[] = {
        0.0f,                // Starting position (0 degrees)
//        M_PI/2.0f,       // 30 degrees
//        0.0f,                // Back to 0
//        M_PI/4.0f,           // 45 degrees
//        0.0f,                // Back to 0
        M_PI/3.0f,           // 60 degrees
//        0.0f,                // Back to 0
        M_PI/2.0f,          // 90 degrees
        0.0f                 // Back to 0
    };
    static const uint8_t num_setpoints = sizeof(position_setpoints) / sizeof(position_setpoints[0]);

    if (HAL_GetTick() - last_step_time > 3000) {
        step_index = (step_index + 1) % num_setpoints;

        tuning.position_setpoint = position_setpoints[step_index];

        last_step_time = HAL_GetTick();
    }
}


float Cascade_Update(CascadeController* cascade, float pos_ref, float pos_actual, float vel_actual) {
    float raw_velocity_setpoint = PID_Update(&cascade->position_controller, pos_ref, pos_actual);

    float max_velocity_change = 0.2f;

    if (raw_velocity_setpoint - cascade->velocity_setpoint > max_velocity_change) {
        cascade->velocity_setpoint += max_velocity_change;
    } else if (cascade->velocity_setpoint - raw_velocity_setpoint > max_velocity_change) {
        cascade->velocity_setpoint -= max_velocity_change;
    } else {
        cascade->velocity_setpoint = raw_velocity_setpoint;
    }

    if (cascade->velocity_setpoint > cascade->max_velocity) {
        cascade->velocity_setpoint = cascade->max_velocity;
    }
    if (cascade->velocity_setpoint < -cascade->max_velocity) {
        cascade->velocity_setpoint = -cascade->max_velocity;
    }
//
    cascade->current_setpoint = PID_Update(&cascade->velocity_controller,
                                          cascade->velocity_setpoint, vel_actual);

//
//	cascade->current_setpoint = PID_Update(&cascade->velocity_controller,
//											raw_velocity_setpoint , vel_actual);

    return cascade->current_setpoint;
}


void Encoder_Init(EncoderData* enc, TIM_HandleTypeDef* htim) {
    HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);

    enc->htim = htim;
    enc->count = 0;
    enc->last_count = 0;
    enc->last_count_for_speed = 0;
    enc->position = 0.0f;
    enc->speed = 0.0f;
    enc->speed_filtered = 0.0f;
    enc->last_speed_calc_time = HAL_GetTick();


}




void Encoder_Read(EncoderData* enc) {
    uint16_t current_count = __HAL_TIM_GET_COUNTER(enc->htim);

    int16_t count_diff;

    if (current_count > enc->last_count) {
        if (current_count - enc->last_count > 32768) {
            count_diff = (int16_t)((int32_t)current_count - 65536 - (int32_t)enc->last_count);
        } else {
            count_diff = (int16_t)(current_count - enc->last_count);
        }
    } else {
        if (enc->last_count - current_count > 32768) {
            count_diff = (int16_t)((int32_t)current_count + 65536 - (int32_t)enc->last_count);
        } else {
            count_diff = (int16_t)(current_count - enc->last_count);
        }
    }

    enc->count += count_diff;
    enc->last_count = current_count;

    enc->position = (float)enc->count / ENCODER_CPR * 2.0f * M_PI / GEAR_RATIO;
}

void Encoder_CalculateSpeed(EncoderData* enc) {
    uint32_t current_time = HAL_GetTick();
    uint32_t time_diff = current_time - enc->last_speed_calc_time;

    if (time_diff >= 10) {  // Calculate speed every 10ms (matches control loop)
        float dt = time_diff / 1000.0f;  // Convert to seconds

        int32_t count_diff = enc->count - enc->last_count_for_speed;

        float raw_speed = (float)count_diff / ENCODER_CPR * 2.0f * M_PI / GEAR_RATIO / dt;

        float alpha = 0.0f;
        enc->speed_filtered = alpha * raw_speed + (1.0f - alpha) * enc->speed_filtered;



        float voltage_input = cascade.current_setpoint;


        enc->speed = raw_speed;
//        enc->kalman_velocity

        enc->last_count_for_speed = enc->count;
        enc->last_speed_calc_time = current_time;
    }
}


//void Encoder_CalculateSpeed(EncoderData* enc) {
//    uint32_t current_time = HAL_GetTick();
//    uint32_t time_diff = current_time - enc->last_speed_calc_time;
//
//    if (time_diff >= 10) {
//        float dt = time_diff / 1000.0f;
//
//        int32_t count_diff = enc->count - enc->last_count_for_speed;
//
//        float raw_speed = (float)count_diff / ENCODER_CPR * 2.0f * M_PI / GEAR_RATIO / dt;
//
//        float alpha = 0.2f;
//        enc->speed_filtered = alpha * raw_speed + (1.0f - alpha) * enc->speed_filtered;
//
//        enc->last_count_for_speed = enc->count;
//        enc->last_speed_calc_time = current_time;
//        enc->speed = enc->speed_filtered;
//    }
//}


void SetMotorPWM(float voltage, float max_voltage, uint32_t period) {
    if (voltage > max_voltage) {
        voltage = max_voltage;
    }
    if (voltage < -max_voltage) {
        voltage = -max_voltage;
    }

    float duty_cycle = fabsf(voltage) / max_voltage;

    uint32_t pwm_value = (uint32_t)(duty_cycle * period);
     __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm_value);


     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1,
     			voltage >= 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
     	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12,
     			voltage >= 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);

//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, voltage >= 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
}


void InitializeTuning(void) {
    tuning.position_kp = 0.0f;
    tuning.position_ki = 0.0f;
    tuning.position_kd = 0.0f;

    tuning.velocity_kp = 0.0f;
    tuning.velocity_ki = 0.0f;
    tuning.velocity_kd = 0.0f;

    tuning.max_velocity = 1.0f;
    tuning.max_voltage = 24.0f;
    tuning.pwm_period = 19999;

    tuning.position_setpoint = 0.0f;
    tuning.control_enabled = 0;

    tuning.test_running = 0;
    tuning.test_start_time = 0;
    tuning.test_amplitude = M_PI / 4.0f;  // 45 degrees
    tuning.test_frequency = 0.2f;

    motorParams.R = 3.237f;        // Ohms
    motorParams.L = 0.076f;        // Henry
    motorParams.Ke = 0.1219f;       // V·s/rad
    motorParams.Kt = 1.551f;       // N·m/A
    motorParams.Jm = 0.0003f;       // kg·m²
    motorParams.Bm = 0.25f;       // N·m·s/rad


    motorParams.R = 5.236883729;
    motorParams.L = 0.1755341958;
	motorParams.Ke    = 1.573781418;
	motorParams.Jm = 0.009335780353;
	motorParams.Bm = 0.1999320178;

//    motorParams.R = 1.233f;        // Ohms
////    motorParams.L = 0.176f;        // Henry
//    motorParams.L = 0.03893f;        // Henry
//    motorParams.Ke = 0.011219f;       // V·s/rad
//    motorParams.Kt = 1.551f;       // N·m/A
//    motorParams.Jm = 0.0003f;       // kg·m²
//    motorParams.Bm = 0.00219f;       // N·m·s/rad

    motorParams.n_gear = 4.0f;     // Gear ratio
    motorParams.r_pulley = 0.01273f; // m

    systemParams.m_total = 0.6f;   // kg (estimate)
    systemParams.m_diff = 0.0f;    // kg (balanced system)
    systemParams.g = 9.81f;        // m/s²
    systemParams.J_arm = 0.02f;    // kg·m² (estimate)
    systemParams.c_revolute = 0.02f; // N·m·s/rad (estimate)

    Cascade_Init(&cascade,
                tuning.position_kp, tuning.position_ki, tuning.position_kd,
                tuning.velocity_kp, tuning.velocity_ki, tuning.velocity_kd,
                0.001f);  // 10ms sample time


    cascade.max_velocity = tuning.max_velocity;

    // Initialize encoder
    Encoder_Init(&encoder, &htim2);

    // Start PWM timer
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,0);
//    HAL_Delay(1000);


    float Q_increased = 0.3f;
    float R_decreased = 0.01f;

    MotorModelKalman_Init(&motorFilter, 0.001f, motorParams.Jm, motorParams.Bm,
                          motorParams.Kt, motorParams.Ke, motorParams.R, motorParams.L,
                          Q_increased, R_decreased);
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

float GenerateTestReference(void) {
    if (!tuning.test_running) {
        return tuning.position_setpoint;
    }

    uint32_t elapsed_time = HAL_GetTick() - tuning.test_start_time;
    float time_sec = elapsed_time / 1000.0f;

    return tuning.test_amplitude * sinf(2.0f * M_PI * tuning.test_frequency * time_sec);
}

float revolute_feedforward(float theta_ref, float omega_ref, float alpha_ref,
                          MotorParams* motor, SystemParams* system) {
    float d = 0.3f;
    float J_total = system->J_arm + system->m_total * d * d;
    float T_inertia = J_total * alpha_ref;
    float T_gravity = system->m_diff * d * system->g * cosf(theta_ref);
    float T_friction = system->c_revolute * omega_ref;

    float T_motor_inertia = motor->Jm * alpha_ref * motor->n_gear * motor->n_gear;
    float T_motor_friction = motor->Bm * omega_ref * motor->n_gear;

    float T_total = T_inertia + T_gravity + T_friction + T_motor_inertia + T_motor_friction;

    float current_required = T_total / (motor->Kt * motor->n_gear);
    float v_feedforward = motor->R * current_required + motor->Ke * omega_ref * motor->n_gear;

    return v_feedforward;
}

void robot_start_trajectoryy(void) {
    static uint8_t initialized = 0;

    if (!initialized) {
        cascade.velocity_controller.error_integral = 0.0f;  // Reset integral term
        target_params.max_velocity = 1.0f;                 // Target velocity (rad/s)
        target_params.max_acceleration = 0.4f;             // Max acceleration (rad/s²)
        target_params.start_time = HAL_GetTick();
        target_params.test_active = 1;

        initialized = 1;
    }
}



void TrapezoidalVelocityProfile(void) {
    const float max_velocity = 1.0f;        // rad/s
    const float max_acceleration = 0.4f;    // rad/s²

    const float accel_time = max_velocity / max_acceleration;  // Time to reach max velocity

    uint32_t current_time = HAL_GetTick();
    if (motion_start_time == 0) {
        motion_start_time = current_time;
        current_velocity_setpoint = 0.0f;
        motion_complete = 0;
    }

    float elapsed_time = (current_time - motion_start_time) / 1000.0f;

    if (elapsed_time < accel_time) {
        current_velocity_setpoint = max_acceleration * elapsed_time;
    } else if (elapsed_time < (accel_time + 3.0f)) {  // 3 seconds at max velocity
        current_velocity_setpoint = max_velocity;
    } else if (elapsed_time < (2 * accel_time + 3.0f)) {
        float decel_time = elapsed_time - (accel_time + 3.0f);
        current_velocity_setpoint = max_velocity - (max_acceleration * decel_time);
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

    cascade.velocity_setpoint = current_velocity_setpoint;
}

void Trapezoid_Init(SimpleTrapezoid* t, float max_vel, float max_accel) {
    t->max_velocity = max_vel;
    t->max_acceleration = max_accel;
    t->state = 0; // IDLE
    t->position = 0.0f;
    t->velocity = 0.0f;
    t->acceleration = 0.0f;
}

// Set a new target position
void Trapezoid_SetTarget(SimpleTrapezoid* t, float current_pos, float target_pos) {
    t->start_position = current_pos;
    t->target_position = target_pos;
    t->start_time = HAL_GetTick();

    // Calculate trajectory parameters
    float distance = fabsf(target_pos - current_pos);
    float direction = (target_pos > current_pos) ? 1.0f : -1.0f;

    // Time to reach max velocity
    t->accel_time = t->max_velocity / t->max_acceleration;

    // Distance covered during acceleration/deceleration
    float accel_distance = 0.5f * t->max_acceleration * t->accel_time * t->accel_time;

    // Check if we can reach max velocity
    if (distance > 2.0f * accel_distance) {
        // We have a cruise phase
        t->decel_time = t->accel_time;
        float cruise_distance = distance - 2.0f * accel_distance;
        t->cruise_time = cruise_distance / t->max_velocity;
    } else {
        // We can't reach max velocity - triangular profile
        t->accel_time = sqrtf(distance / t->max_acceleration);
        t->cruise_time = 0.0f;
        t->decel_time = t->accel_time;
    }

    t->total_time = t->accel_time + t->cruise_time + t->decel_time;
    t->state = 1; // ACCELERATING
}

// Update the profile based on elapsed time
void Trapezoid_Update(SimpleTrapezoid* t) {
    if (t->state == 0 || t->state == 4) {
        return; // Nothing to do if idle or complete
    }

    float direction = (t->target_position > t->start_position) ? 1.0f : -1.0f;
    uint32_t elapsed_ms = HAL_GetTick() - t->start_time;
    float elapsed_time = elapsed_ms / 1000.0f; // Convert to seconds

    // Check if we're done
    if (elapsed_time >= t->total_time) {
        t->position = t->target_position;
        t->velocity = 0.0f;
        t->acceleration = 0.0f;
        t->state = 4; // DONE
        return;
    }

    // Calculate state based on elapsed time
    if (elapsed_time < t->accel_time) {
        // Acceleration phase
        t->state = 1; // ACCELERATING
        t->acceleration = t->max_acceleration * direction;
        t->velocity = t->acceleration * elapsed_time;
        t->position = t->start_position + 0.5f * t->acceleration * elapsed_time * elapsed_time;
    }
    else if (elapsed_time < (t->accel_time + t->cruise_time)) {
        // Constant velocity phase
        t->state = 2; // CRUISE
        t->acceleration = 0.0f;
        t->velocity = t->max_acceleration * t->accel_time * direction;

        float time_in_cruise = elapsed_time - t->accel_time;
        float distance_in_accel = 0.5f * t->max_acceleration * t->accel_time * t->accel_time;
        float distance_in_cruise = t->velocity * time_in_cruise;

        t->position = t->start_position + (distance_in_accel + distance_in_cruise) * direction;
    }
    else {
        // Deceleration phase
        t->state = 3; // DECELERATING
        float time_in_decel = elapsed_time - t->accel_time - t->cruise_time;

        t->acceleration = -t->max_acceleration * direction;
        t->velocity = (t->max_acceleration * t->accel_time -
                       t->max_acceleration * time_in_decel) * direction;

        float distance_in_accel = 0.5f * t->max_acceleration * t->accel_time * t->accel_time;
        float distance_in_cruise = t->max_acceleration * t->accel_time * t->cruise_time;
        float distance_in_decel = (t->max_acceleration * t->accel_time * time_in_decel -
                                  0.5f * t->max_acceleration * time_in_decel * time_in_decel);

        t->position = t->start_position +
                     (distance_in_accel + distance_in_cruise + distance_in_decel) * direction;
    }

    // Ensure we don't overshoot
    if ((direction > 0 && t->position > t->target_position) ||
        (direction < 0 && t->position < t->target_position)) {
        t->position = t->target_position;
        t->velocity = 0.0f;
        t->acceleration = 0.0f;
        t->state = 4; // DONE
    }
}

// Check if the trajectory is complete
uint8_t Trapezoid_IsComplete(SimpleTrapezoid* t) {
    return (t->state == 4); // DONE
}
void MoveToPosition(float target_position_rad) {
    Trapezoid_SetTarget(&trap, encoder.position, target_position_rad);
    move_in_progress = 1;
}


void ControlLoop(void) {
    Encoder_Read(&encoder);
    Encoder_CalculateSpeed(&encoder);

    if (move_in_progress) {
        Trapezoid_Update(&trap);

        if (Trapezoid_IsComplete(&trap)) {
            move_in_progress = 0;
        }
    }

    float position_setpoint = move_in_progress ? trap.position : tuning.position_setpoint;
    float velocity_setpoint = move_in_progress ? trap.velocity : cascade.velocity_setpoint;
    float acceleration_setpoint = move_in_progress ? trap.acceleration : 0.0f;

    // Rest of your control loop remains the same
    float control_signal = cascade.current_setpoint;
    MotorModelKalman_Estimate(&motorFilter, control_signal, encoder.position);

    motor_model_velocity = motorFilter.velocity * 0.5f;


    float compensated_velocity = ApplyPhaseLead(motor_model_velocity, 0.5f, 0.6f);

    tuning.position_setpoint = position_setpoint;

    if (move_in_progress) {
        cascade.velocity_setpoint = velocity_setpoint;
    }

    float v_feedforward = revolute_feedforward(
        position_setpoint,
        velocity_setpoint,
        acceleration_setpoint,
        &motorParams,
        &systemParams
    );

    float v_feedback = Cascade_Update(&cascade,
                                   position_setpoint,
                                   encoder.position,
                                   compensated_velocity);

    float control_signal_output = v_feedback + v_feedforward;


    SetMotorPWM(control_signal_output, tuning.max_voltage, tuning.pwm_period);
}


void StartStepTest(float amplitude) {
    tuning.test_running = 1;
    tuning.test_start_time = HAL_GetTick();
    tuning.test_amplitude = amplitude;
    tuning.position_setpoint = 0.0f;
}


void StopTest(void) {
    tuning.test_running = 0;
    tuning.position_setpoint = encoder.position;
}


void UpdateTuningParameters(float pos_kp, float pos_ki, float pos_kd,
                          float vel_kp, float vel_ki, float vel_kd) {
    // Update tuning parameters
    tuning.position_kp = pos_kp;
    tuning.position_ki = pos_ki;
    tuning.position_kd = pos_kd;
    tuning.velocity_kp = vel_kp;
    tuning.velocity_ki = vel_ki;
    tuning.velocity_kd = vel_kd;

    // Update the controllers directly
    cascade.position_controller.kp = pos_kp;
    cascade.position_controller.ki = pos_ki;
    cascade.position_controller.kd = pos_kd;

    cascade.velocity_controller.kp = vel_kp;
    cascade.velocity_controller.ki = vel_ki;
    cascade.velocity_controller.kd = vel_kd;

    // Reset the integral terms to avoid windup
    cascade.position_controller.error_integral = 0.0f;
    cascade.velocity_controller.error_integral = 0.0f;
}


void EnableControl(void) {
    cascade.position_controller.error_integral = 0.0f;
    cascade.velocity_controller.error_integral = 0.0f;

    tuning.position_setpoint = encoder.position;

    tuning.control_enabled = 1;
}

/**
 * Disable control
 */
void DisableControl(void) {
    tuning.control_enabled = 0;
    SetMotorPWM(0.0f, tuning.max_voltage, tuning.pwm_period);
}

void TestVelocityBidirectional(void) {
    static uint32_t last_velocity_change = 0;
    static uint8_t velocity_index = 0;

    // Array of velocity setpoints to cycle through
    const float velocity_setpoints[] = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
    const uint8_t num_setpoints = sizeof(velocity_setpoints) / sizeof(velocity_setpoints[0]);

    // Change velocity every 3 seconds (3000 ms)
    if (HAL_GetTick() - last_velocity_change > 3000) {
        // Move to next velocity setpoint
        velocity_index = (velocity_index + 1) % num_setpoints;

        // Update the test velocity
        test_velocity = velocity_setpoints[velocity_index];

        // Update last change timestamp
        last_velocity_change = HAL_GetTick();
    }

    // Set the velocity setpoint for the cascade controller
    cascade.velocity_setpoint = test_velocity;
}

/***************************************************************
 *              DC MOTOR MODEL IMPLEMENTATION                  *
 ***************************************************************/

void MotorModelKalman_Init(MotorModelKalman* filter, float dt, float J, float b,
                          float Kt, float Ke, float Ra, float La,
                          float process_noise, float meas_noise) {
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

    filter->A[0 * MOTOR_MODEL_NUM_STATES + 1] = 1.0f;               /* dθ/dt = ω */
    filter->A[1 * MOTOR_MODEL_NUM_STATES + 1] = (-filter->b / filter->J) * 1.5f;  /* Friction */
    filter->A[1 * MOTOR_MODEL_NUM_STATES + 2] = -1.0f / filter->J;       /* Load torque */
    filter->A[1 * MOTOR_MODEL_NUM_STATES + 3] = filter->Kt / filter->J;  /* Motor torque */
    filter->A[3 * MOTOR_MODEL_NUM_STATES + 1] = -filter->Ke / filter->La; /* Back-EMF */
    filter->A[3 * MOTOR_MODEL_NUM_STATES + 3] = -filter->Ra / filter->La; /* Resistance */

    memset(filter->B, 0, sizeof(filter->B));
    filter->B[3] = 1.0f / filter->La; /* Voltage input affects current */

    memset(filter->Q, 0, sizeof(filter->Q));
	filter->Q[0 * MOTOR_MODEL_NUM_STATES + 0] = (process_noise * 0.5f) * (process_noise * 0.5f); // Position
	filter->Q[1 * MOTOR_MODEL_NUM_STATES + 1] = (process_noise * 0.2f) * (process_noise * 0.2f); // Velocity - reduce from original
	filter->Q[2 * MOTOR_MODEL_NUM_STATES + 2] = (process_noise * 0.5f) * (process_noise * 0.5f); // Load torque
	filter->Q[3 * MOTOR_MODEL_NUM_STATES + 3] = (process_noise * 0.1f) * (process_noise * 0.1f); // Current

	filter->R[0] = meas_noise * meas_noise * 1.5f; // Increase by 50% to rely more on model

	MotorModelKalman_DiscretizeSystem(filter);
    }


void MotorModelKalman_DiscretizeSystem(MotorModelKalman* filter) {
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
            filter->A_d[i * MOTOR_MODEL_NUM_STATES + j] +=
                filter->A[i * MOTOR_MODEL_NUM_STATES + j] * filter->dt;
        }
    }

    /* B_d = B*dt */
    for (int i = 0; i < MOTOR_MODEL_NUM_STATES; i++) {
        filter->B_d[i] = filter->B[i] * filter->dt;
    }

    /* Q_d = Q*dt */
    for (int i = 0; i < MOTOR_MODEL_NUM_STATES; i++) {
        for (int j = 0; j < MOTOR_MODEL_NUM_STATES; j++) {
            filter->Q_d[i * MOTOR_MODEL_NUM_STATES + j] =
                filter->Q[i * MOTOR_MODEL_NUM_STATES + j] * filter->dt;
        }
    }
}

void MotorModelKalman_Reset(MotorModelKalman* filter) {
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

void MotorModelKalman_Predict(MotorModelKalman* filter, float voltage_input) {
    float temp_state[MOTOR_MODEL_NUM_STATES] = {0};
    float temp_P[MOTOR_MODEL_NUM_STATES * MOTOR_MODEL_NUM_STATES] = {0};

    /* 1. State prediction: X = A_d*X + B_d*u */

    /* Calculate A_d*X */
    for (int i = 0; i < MOTOR_MODEL_NUM_STATES; i++) {
        temp_state[i] = 0;
        for (int j = 0; j < MOTOR_MODEL_NUM_STATES; j++) {
            temp_state[i] += filter->A_d[i * MOTOR_MODEL_NUM_STATES + j] * filter->X[j];
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
                temp_P[i * MOTOR_MODEL_NUM_STATES + j] +=
                    filter->A_d[i * MOTOR_MODEL_NUM_STATES + k] *
                    filter->P[k * MOTOR_MODEL_NUM_STATES + j];
            }
        }
    }

    /* Now calculate (A_d*P)*A_d' -> filter->P */
    memset(filter->P, 0, sizeof(filter->P));
    for (int i = 0; i < MOTOR_MODEL_NUM_STATES; i++) {
        for (int j = 0; j < MOTOR_MODEL_NUM_STATES; j++) {
            for (int k = 0; k < MOTOR_MODEL_NUM_STATES; k++) {
                filter->P[i * MOTOR_MODEL_NUM_STATES + j] +=
                    temp_P[i * MOTOR_MODEL_NUM_STATES + k] *
                    filter->A_d[j * MOTOR_MODEL_NUM_STATES + k]; /* Note: A_d transpose */
            }
        }
    }

    /* Add Q_d */
    for (int i = 0; i < MOTOR_MODEL_NUM_STATES; i++) {
        for (int j = 0; j < MOTOR_MODEL_NUM_STATES; j++) {
            filter->P[i * MOTOR_MODEL_NUM_STATES + j] += filter->Q_d[i * MOTOR_MODEL_NUM_STATES + j];
        }
    }

    /* Update state estimates */
    filter->position = filter->X[0];
    filter->velocity = filter->X[1];
    filter->load_torque = filter->X[2];
    filter->current = filter->X[3];
}

void MotorModelKalman_Update(MotorModelKalman* filter, float position_measurement) {
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
            filter->P[i * MOTOR_MODEL_NUM_STATES + j] -=
                filter->K[i] * filter->P[0 * MOTOR_MODEL_NUM_STATES + j];
        }
    }

    /* Update state estimates */
    filter->position = filter->X[0];
    filter->velocity = filter->X[1];
    filter->load_torque = filter->X[2];
    filter->current = filter->X[3];
}

void MotorModelKalman_Estimate(MotorModelKalman* filter, float voltage_input, float position_measurement) {
    MotorModelKalman_Predict(filter, voltage_input);
    MotorModelKalman_Update(filter, position_measurement);
}


/***************************************************************
 *                   UTILITY FUNCTIONS                         *
 ***************************************************************/

void MatrixMultiply(float* A, float* B, float* C, int rows_a, int cols_a, int cols_b) {
    for (int i = 0; i < rows_a; i++) {
        for (int j = 0; j < cols_b; j++) {
            C[i * cols_b + j] = 0.0f;
            for (int k = 0; k < cols_a; k++) {
                C[i * cols_b + j] += A[i * cols_a + k] * B[k * cols_b + j];
            }
        }
    }
}

void MatrixAdd(float* A, float* B, float* C, int rows, int cols) {
    for (int i = 0; i < rows * cols; i++) {
        C[i] = A[i] + B[i];
    }
}

void MatrixSubtract(float* A, float* B, float* C, int rows, int cols) {
    for (int i = 0; i < rows * cols; i++) {
        C[i] = A[i] - B[i];
    }
}

void MatrixTranspose(float* A, float* AT, int rows, int cols) {
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            AT[j * rows + i] = A[i * cols + j];
        }
    }
}

void MatrixScale(float* A, float* B, float scale, int rows, int cols) {
    for (int i = 0; i < rows * cols; i++) {
        B[i] = A[i] * scale;
    }
}

void MatrixCopy(float* src, float* dst, int size) {
    for (int i = 0; i < size; i++) {
        dst[i] = src[i];
    }
}

void MatrixIdentity(float* A, int size) {
    memset(A, 0, size * size * sizeof(float));
    for (int i = 0; i < size; i++) {
        A[i * size + i] = 1.0f;
    }
}

float calculateDisplacement(float angle_rad) {
    return 0.0492f + 1.33E-03f*angle_rad - 2.32E-05f*angle_rad*angle_rad
           + 1.4E-07f*powf(angle_rad, 3) - 3.01E-10f*powf(angle_rad, 4)
           - 1.05E-13f*powf(angle_rad, 5);
}
void setPWMToPrismatic(float revolute_position, uint32_t prismatic_pwm_period, float prismatic_max_voltage) {
    float revolute_change = revolute_position - prev_revolute_position;

    prev_revolute_position = revolute_position;


    if (fabsf(revolute_change) < 0.0005f) {
        return;
    }

    displacement_change = calculateDisplacement(revolute_position)*50;
    float compensation_voltage;

    if (revolute_change > 0) {
        compensation_voltage = displacement_change;
    } else {
        compensation_voltage = -displacement_change;
    }


    float duty_cycle = fabsf(compensation_voltage) / prismatic_max_voltage;
    uint32_t pwm_value = (uint32_t)(duty_cycle * prismatic_pwm_period);

//    if (pwm_value < 500 && pwm_value > 0) {
//        pwm_value = 500;
//    }

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12,
                     (compensation_voltage >= 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm_value);

    prismatic_pwm = (compensation_voltage >= 0) ? pwm_value : -pwm_value;
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */  HAL_Init();

  /* USER CODE BEGIN Init */
  Encoder_Init(&encoder, &htim2);
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
  /* USER CODE BEGIN 2 */
  InitializeTuning();
  EnableControl();

//  cascade.velocity_setpoint = 1.0f;
//  UpdateTuningParameters(13.0f, 0.0000000000001f, 0.25f, 0.95f, 7.2f, 0.0f);
  UpdateTuningParameters(5.0f, 0.00000000000001f, 0.1f, 0.5f,42.0f, 0.0f);
  Trapezoid_Init(&trap, 1.0f, 4.0f);
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


//	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 1);
//
//	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 20000);
//
//
//  }

//  ControlLoop();
//  HAL_Delay(2); 500 HZ revo
//	  static uint32_t current_Hz = 0;
//	  if (HAL_GetTick() - current_Hz > 0.001) {
//		  current_Hz = HAL_GetTick();
//////		  TestPositionStep();
////		   TestPositionStep();
//
//		  tuning.position_setpoint = position_setpoint_deg * (M_PI / 180.0f);
////		  TestVelocityBidirectional();
////		  TrapezoidalVelocityProfile();
//		   ControlLoop();
//
//	  }



	  //trajec


	  static uint32_t last_control_time = 0;
	     uint32_t current_time = HAL_GetTick();

	     // Run control loop at 1kHz (every 1ms)
	     if (current_time - last_control_time >= 1) {
	         last_control_time = current_time;

	         // Run the control loop
	         ControlLoop();

	         // Optional: If you want to run a back-and-forth test automatically
	         static uint32_t last_move_time = 0;
	         static uint8_t position_index = 0;

	         // If currently not moving and it's been 3 seconds since last move
	         if (!move_in_progress && current_time - last_move_time > 3000) {
	             position_index = (position_index + 1) % 2;
	             float next_position = (position_index == 0) ? 0 : M_PI/2.0f;  // Alternate between 0 and 90 degrees

	             // Start the next movement
	             Trapezoid_SetTarget(&trap, encoder.position, next_position);
	             move_in_progress = 1;
	             last_move_time = current_time;
	         }
	     }


////	  TestPositionStep();
////	  ControlLoop();
//
////	  TestVelocityBidirectional();
////	  		   ControlLoop();
////	  HAL_Delay(2);
//
////  TestPositionStep();
  }
}
  /* USER CODE END 3 */

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */
  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

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
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
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
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
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
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */
  /* USER CODE END ADC3_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

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
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc3, &multimode) != HAL_OK)
  {
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
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
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
static void MX_I2C1_Init(void)
{

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
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
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
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */
  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */
  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 169;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 19999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 2000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
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
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
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
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */
  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

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
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
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
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */
  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

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
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */
  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */
  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 1699;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 1999;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */
  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */
  /* USER CODE END TIM17_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM17_Init 1 */
  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 169;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 2005;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim17, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_ACTIVE;
  sConfigOC.Pulse = 1433;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim17, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
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
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim17, &sBreakDeadTimeConfig) != HAL_OK)
  {
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
static void MX_USART1_UART_Init(void)
{

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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

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
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
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
  GPIO_InitStruct.Pin = LPUART1_TX_Pin|LPUART1_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF12_LPUART1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_15;
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
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_4
                          |GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
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

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
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
