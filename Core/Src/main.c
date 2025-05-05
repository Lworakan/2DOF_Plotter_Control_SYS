/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "encoder.h"
#include "robot_control.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    TEST_MODE_IDLE,
    TEST_MODE_ACCELERATION,
    TEST_MODE_CONSTANT_VELOCITY,
    TEST_MODE_POSITION_STEP,
    TEST_MODE_RETURN_HOME
} TestMode;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TARGET_ACCEL_REVOLUTE   0.4f    // Target acceleration for revolute joint (rad/s²)
#define TARGET_VELOCITY_REVOLUTE 1.0f    // Target velocity for revolute joint (rad/s)
#define TEST_AMPLITUDE           45.0f   // Test amplitude in degrees
#define ACCELERATION_TEST_PERIOD 3000    // Period for acceleration test (ms)
#define VELOCITY_TEST_DURATION   5000    // Duration for constant velocity test (ms)
#define CONTROL_UPDATE_PERIOD    10      // Control update period (ms)
#define DATA_LOGGING_PERIOD      50      // Data logging period (ms)
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
RobotController robot;

TestMode current_test_mode = TEST_MODE_CONSTANT_VELOCITY;
uint32_t test_start_time = 0;
uint32_t last_direction_change_time = 0;  // Fixed initialization
uint8_t current_direction = 0;      // 0 = positive, 1 = negative

uint32_t last_control_time = 0;
uint32_t last_send_time = 0;
uint32_t last_debug_time = 0;

#define UART_TX_BUFFER_SIZE 256
uint8_t uartTxBuffer[UART_TX_BUFFER_SIZE];

float revolute_angle = 0.0f;
float revolute_angle_kal = 0.0f;
float revolute_velocity = 0.0f;
float revolute_velocity_kal = 0.0f;
float target_position = 0.0f;
float target_velocity = 0.0f;

float debug_setpoint = 0.0f;
float debug_position = 0.0f;
float debug_error = 0.0f;
float debug_control_output = 0.0f;
float debug_velocity = 0.0f;
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
void StartAccelerationTest(void);
void StartConstantVelocityTest(void);
void StartPositionStepTest(void);
void UpdateTestMode(void);
void SendDataToUART(void);
void UpdateDebugVariables(void);
void SWV_SendDebugData(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Configure logging of specific ITM channels
#define ITM_Port32(n) (*((volatile unsigned long *)(0xE0000000+4*n)))

// Send float data to ITM port
void ITM_SendFloat(uint32_t port, float value) {
    uint32_t val_int = *((uint32_t*)&value);
    ITM_Port32(port) = val_int;
}

// Start test to verify acceleration limit
void StartAccelerationTest(void) {
    // Reset the robot to a known state
    robot_disable(&robot);
    HAL_Delay(100);
    robot_enable(&robot);

    // Set initial conditions
    robot_set_targets(&robot, 0.0f, 0.0f);

    // Set test parameters
    current_test_mode = TEST_MODE_ACCELERATION;
    test_start_time = HAL_GetTick();
    current_direction = 0;  // Start with positive direction

    // Log start of test
    sprintf((char*)uartTxBuffer, "ACCELERATION_TEST_START;TIME:%lu\r\n", test_start_time);
    HAL_UART_Transmit(&huart1, uartTxBuffer, strlen((char*)uartTxBuffer), 100);
}

// Start test to verify constant velocity
void StartConstantVelocityTest(void) {
    // Reset the robot to a known state
    robot_disable(&robot);
    HAL_Delay(100);
    robot_enable(&robot);

    // Set initial conditions
    robot_set_targets(&robot, 0.0f, 0.0f);

    // Set test parameters
    current_test_mode = TEST_MODE_CONSTANT_VELOCITY;
    test_start_time = HAL_GetTick();
    current_direction = 0;  // Start with positive direction

    // Log start of test
    sprintf((char*)uartTxBuffer, "CONSTANT_VELOCITY_TEST_START;TIME:%lu\r\n", test_start_time);
    HAL_UART_Transmit(&huart1, uartTxBuffer, strlen((char*)uartTxBuffer), 100);
}

// Start position step test
void StartPositionStepTest(void) {
    // Reset the robot to a known state
    robot_disable(&robot);
    HAL_Delay(100);
    robot_enable(&robot);

    // Set initial conditions
    robot_set_targets(&robot, 0.0f, 0.0f);

    // Set test parameters
    current_test_mode = TEST_MODE_POSITION_STEP;
    test_start_time = HAL_GetTick();
    last_direction_change_time = test_start_time;
    current_direction = 0;  // Start with positive direction

    // Log start of test
    sprintf((char*)uartTxBuffer, "POSITION_STEP_TEST_START;TIME:%lu\r\n", test_start_time);
    HAL_UART_Transmit(&huart1, uartTxBuffer, strlen((char*)uartTxBuffer), 100);
}

// Update test mode based on current state
void UpdateTestMode(void) {
    uint32_t current_time = HAL_GetTick();
    uint32_t elapsed_time = current_time - test_start_time;

    switch (current_test_mode) {
        case TEST_MODE_ACCELERATION:
            // In acceleration test, we alternate between accelerating and decelerating
            if (current_time - last_direction_change_time >= ACCELERATION_TEST_PERIOD) {
                // Change direction
                current_direction = !current_direction;
                last_direction_change_time = current_time;

                // Log direction change
                sprintf((char*)uartTxBuffer, "DIRECTION_CHANGE;DIR:%d;TIME:%lu\r\n",
                        current_direction, current_time);
                HAL_UART_Transmit(&huart1, uartTxBuffer, strlen((char*)uartTxBuffer), 100);
            }

            // Set position target far enough to ensure continuous motion
            float target_pos1 = current_direction ? -180.0f : 180.0f;
            robot_set_targets(&robot, target_pos1, 0.0f);  // ±180 degrees - far enough

            // End test after 4 cycles
            if (elapsed_time >= 4 * ACCELERATION_TEST_PERIOD) {
                // Return to home position
                current_test_mode = TEST_MODE_RETURN_HOME;
                test_start_time = current_time;

                // Log end of test
                sprintf((char*)uartTxBuffer, "ACCELERATION_TEST_END;TIME:%lu\r\n", current_time);
                HAL_UART_Transmit(&huart1, uartTxBuffer, strlen((char*)uartTxBuffer), 100);
            }
            break;

        case TEST_MODE_CONSTANT_VELOCITY:
            // In constant velocity test, we maintain velocity for a fixed duration

            // Set position target far enough to ensure continuous motion
            float target_pos2 = current_direction ? -180.0f : 180.0f;
            robot_set_targets(&robot, target_pos2, 0.0f);  // ±180 degrees - far enough

            // Change direction after fixed duration
            if (current_time - last_direction_change_time >= VELOCITY_TEST_DURATION) {
                // Change direction
                current_direction = !current_direction;
                last_direction_change_time = current_time;

                // Log direction change
                sprintf((char*)uartTxBuffer, "DIRECTION_CHANGE;DIR:%d;TIME:%lu\r\n",
                        current_direction, current_time);
                HAL_UART_Transmit(&huart1, uartTxBuffer, strlen((char*)uartTxBuffer), 100);
            }

            // End test after 2 cycles
            if (elapsed_time >= 2 * VELOCITY_TEST_DURATION) {
                // Return to home position
                current_test_mode = TEST_MODE_RETURN_HOME;
                test_start_time = current_time;

                // Log end of test
                sprintf((char*)uartTxBuffer, "CONSTANT_VELOCITY_TEST_END;TIME:%lu\r\n", current_time);
                HAL_UART_Transmit(&huart1, uartTxBuffer, strlen((char*)uartTxBuffer), 100);
            }
            break;

        case TEST_MODE_POSITION_STEP:
            // In position step test, we alternate between two fixed positions
            if (current_time - last_direction_change_time >= ACCELERATION_TEST_PERIOD) {
                // Change direction
                current_direction = !current_direction;
                last_direction_change_time = current_time;

                // Set new target position
                float target_pos3 = current_direction ? -TEST_AMPLITUDE : TEST_AMPLITUDE;
                robot_set_targets(&robot, target_pos3, 0.0f);

                // Log direction change
                sprintf((char*)uartTxBuffer, "POSITION_STEP;POS:%.2f;TIME:%lu\r\n",
                        target_pos3, current_time);
                HAL_UART_Transmit(&huart1, uartTxBuffer, strlen((char*)uartTxBuffer), 100);
            }

            // End test after 6 cycles
            if (elapsed_time >= 6 * ACCELERATION_TEST_PERIOD) {
                // Return to home position
                current_test_mode = TEST_MODE_RETURN_HOME;
                test_start_time = current_time;

                // Log end of test
                sprintf((char*)uartTxBuffer, "POSITION_STEP_TEST_END;TIME:%lu\r\n", current_time);
                HAL_UART_Transmit(&huart1, uartTxBuffer, strlen((char*)uartTxBuffer), 100);
            }
            break;

        case TEST_MODE_RETURN_HOME:
            // Return to home position
            robot_set_targets(&robot, 0.0f, 0.0f);

            // Check if we're close enough to home position
            float revolute_pos, revolute_vel;
            robot_get_state(&robot, &revolute_pos, &revolute_vel, NULL, NULL);

            if (fabsf(revolute_pos) < 1.0f && fabsf(revolute_vel) < 0.1f) {
                // Test completed
                current_test_mode = TEST_MODE_IDLE;

                // Log completion
                sprintf((char*)uartTxBuffer, "TEST_COMPLETED;TIME:%lu\r\n", current_time);
                HAL_UART_Transmit(&huart1, uartTxBuffer, strlen((char*)uartTxBuffer), 100);
            }
            break;

        case TEST_MODE_IDLE:
        default:
            // No active test
            break;
    }
}

// Send data to UART for logging
void SendDataToUART(void) {
    float revolute_pos, revolute_vel, prismatic_pos, prismatic_vel;
    robot_get_state(&robot, &revolute_pos, &revolute_vel, &prismatic_pos, &prismatic_vel);

    uint32_t current_time = HAL_GetTick();

    // Format data string based on current test mode
    switch (current_test_mode) {
        case TEST_MODE_ACCELERATION:
            sprintf((char*)uartTxBuffer,
                    "ACCEL;TIME:%lu;POS:%.2f;VEL:%.2f;SET_POS:%.2f;VEL_SET:%.2f\r\n",
                    current_time, revolute_pos, revolute_vel,
                    robot.revolute_target * 180.0f / M_PI,
                    robot.revolute_cascade.velocity_setpoint);
            break;

        case TEST_MODE_CONSTANT_VELOCITY:
            sprintf((char*)uartTxBuffer,
                    "CONST_VEL;TIME:%lu;POS:%.2f;VEL:%.2f;SET_POS:%.2f;VEL_SET:%.2f\r\n",
                    current_time, revolute_pos, revolute_vel,
                    robot.revolute_target * 180.0f / M_PI,
                    robot.revolute_cascade.velocity_setpoint);
            break;

        case TEST_MODE_POSITION_STEP:
            sprintf((char*)uartTxBuffer,
                    "POS_STEP;TIME:%lu;POS:%.2f;VEL:%.2f;SET_POS:%.2f;VEL_SET:%.2f;CTRL:%.2f\r\n",
                    current_time, revolute_pos, revolute_vel,
                    robot.revolute_target * 180.0f / M_PI,
                    robot.revolute_cascade.velocity_setpoint,
                    robot.revolute_cascade.current_setpoint);
            break;

        case TEST_MODE_RETURN_HOME:
            sprintf((char*)uartTxBuffer,
                    "HOME;TIME:%lu;POS:%.2f;VEL:%.2f;SET_POS:%.2f\r\n",
                    current_time, revolute_pos, revolute_vel,
                    robot.revolute_target * 180.0f / M_PI);
            break;

        case TEST_MODE_IDLE:
        default:
            sprintf((char*)uartTxBuffer,
                    "IDLE;TIME:%lu;POS:%.2f;VEL:%.2f\r\n",
                    current_time, revolute_pos, revolute_vel);
            break;
    }

    HAL_UART_Transmit(&huart1, uartTxBuffer, strlen((char*)uartTxBuffer), 100);
}

void UpdateDebugVariables(void) {
    float revolute_pos, revolute_vel, prismatic_pos, prismatic_vel;
    robot_get_state(&robot, &revolute_pos, &revolute_vel, &prismatic_pos, &prismatic_vel);

    revolute_angle = revolute_pos;
    revolute_velocity = revolute_vel;

    debug_position = revolute_pos;
    debug_velocity = revolute_vel;
    debug_setpoint = robot.revolute_target * 180.0f / M_PI;  // Convert rad to deg
    debug_error = debug_setpoint - debug_position;

    debug_control_output = robot.revolute_cascade.current_setpoint;
}

void SWV_SendDebugData(void) {
    ITM_SendFloat(0, debug_setpoint);     // Target position
    ITM_SendFloat(1, debug_position);     // Actual position
    ITM_SendFloat(2, debug_velocity);     // Actual velocity
    ITM_SendFloat(3, robot.revolute_cascade.velocity_setpoint);  // Velocity setpoint
    ITM_SendFloat(4, debug_control_output);  // Motor voltage

    // Send acceleration data (calculated from velocity change)
    static float prev_velocity = 0.0f;
    static uint32_t prev_time = 0;

    uint32_t current_time = HAL_GetTick();
    float dt = (current_time - prev_time) / 1000.0f;

    if (dt > 0.0f && dt < 0.1f) {  // Valid time interval
        float accel = (debug_velocity - prev_velocity) / dt;
        ITM_SendFloat(5, accel);  // Calculated acceleration
    }

    prev_velocity = debug_velocity;
    prev_time = current_time;
}

// Process UART commands
void ProcessUARTCommand(uint8_t* cmd) {
    if (strcmp((char*)cmd, "start_accel") == 0) {
        StartAccelerationTest();
    } else if (strcmp((char*)cmd, "start_vel") == 0) {
        StartConstantVelocityTest();
    } else if (strcmp((char*)cmd, "start_step") == 0) {
        StartPositionStepTest();
    } else if (strcmp((char*)cmd, "stop") == 0) {
        current_test_mode = TEST_MODE_RETURN_HOME;
        test_start_time = HAL_GetTick();
    }
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
  /* USER CODE BEGIN 2 */


	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

	__HAL_TIM_SET_COUNTER(&htim2, 0);
	  __HAL_TIM_SET_COUNTER(&htim3, 0);
	robot_init(&robot, &htim2, &htim3);

	robot.revolute_cascade.acceleration_limit = TARGET_ACCEL_REVOLUTE;
	  robot.revolute_cascade.max_velocity = TARGET_VELOCITY_REVOLUTE;

	// Initialize timing variables

	// Initialize UART
	HAL_UART_Transmit(&huart1, (uint8_t*)"PID Tuning System Ready\r\n", 25, 100);
	HAL_UART_Transmit(&huart1, (uint8_t*)"Send 'n' to start tuning\r\n", 26, 100);

	encoder_reset(&robot.revolute_encoder);
	  encoder_reset(&robot.prismatic_encoder);


	test_start_time = HAL_GetTick();
	robot_enable(&robot);
//	test_amplitude = 4.0f;  // Target velocity of 1 rad/s
//	test_direction = 0;     // Positive direction (can be changed to 0 for negative)
	last_control_time = HAL_GetTick();
	last_send_time = last_control_time;
	last_debug_time = last_control_time;

	StartPositionStepTest();

	// Turn on LED to indicate system is running
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

//	robot.revolute_cascade.velocity_controller.kp = 25.0f;
//	  robot.revolute_cascade.velocity_controller.ki = 5.0f;
//	  robot.revolute_cascade.velocity_controller.kd = 0.0f;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
//		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == 1){
//			stop = 1;
//		}
//		else if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == 0){
//					stop = 0;
//				}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		// Read encoder values and calculate positions
//	  ReadJoystickButtons();
//	  	  ReadJoystickAnalog();
//
//	  	  // Send data over UART
//	  	  SendJoystickData();
//
//	  	  // Short delay
//	  	  HAL_Delay(10);

//
//		static uint32_t start_tick = 0;
//		static uint8_t motor_running = 0;
//
//		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
//		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, direction1);
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, direction2);
//
//		if (START == 1 && !motor_running) {
//		    START = 0;
//		    motor_running = 1;
//		    start_tick = HAL_GetTick();
//
//		    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, REV_PWM);
//		}
//
//		if (motor_running && ((HAL_GetTick() - start_tick) >= TIME)) {
//		    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
//		    motor_running = 0;
//		}


//		 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
//				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, direction1);
//				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, direction2);
//				if (START==1) {
//					START=0;
//					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, REV_PWM);
//					HAL_Delay(TIME);
//					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
//					REV_PWM+=100;
//
//				}

		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == GPIO_PIN_SET) {
		        robot.stop_flag = 1;
		        // Set motors to zero immediately
		        set_motor_pwm(&htim1, TIM_CHANNEL_1, 0.0f, 24.0f, robot.pwm_period);
		        set_motor_pwm(&htim1, TIM_CHANNEL_2, 0.0f, 24.0f, robot.pwm_period);
		    } else {
		        robot.stop_flag = 0;
		    }

		    uint32_t current_time = HAL_GetTick();
		    if (current_time - last_control_time >= CONTROL_UPDATE_PERIOD) {
		        last_control_time = current_time;

		        UpdateTestMode();

		        robot_update(&robot);
		    }

//		    if (current_time - last_send_time >= DATA_LOGGING_PERIOD) {
//		        last_send_time = current_time;
//		        SendDataToUART();
//		    }

		    if (current_time - last_debug_time >= DATA_LOGGING_PERIOD) {
		        last_debug_time = current_time;
		        UpdateDebugVariables();
		        SWV_SendDebugData();
		    }
	}
}
  /* USER CODE END 3 */
}

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
