/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include <string.h>
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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
#define MPU6050_ADDR 0xD0 // 0x68 shifted left by 1 bit for STM32 HAL

// MPU6050 Raw & Processed Measurements
int16_t Accel_X_RAW = 0, Accel_Y_RAW = 0, Accel_Z_RAW = 0;
int16_t Gyro_X_RAW = 0, Gyro_Y_RAW = 0, Gyro_Z_RAW = 0;
float Ax, Ay, Az;
float Gx, Gy, Gz;

// Pitch & Roll Angle & Calibration Variables
float pitch = 0.0f, pitch_offset = 0.0f;
float roll  = 0.0f, roll_offset  = 0.0f;
uint32_t last_time = 0;

// Continuous Background Motor Speed (0% to 100%)
#define BACKGROUND_THROTTLE 30

// Pitch PID Parameters & State
float Kp_pitch = 1.2f, Ki_pitch = 0.05f, Kd_pitch = 0.15f;
float pitch_setpoint = 0.0f;
float pid_integral_pitch = 0.0f;
float last_pid_error_pitch = 0.0f;

// Roll PID Parameters & State
float Kp_roll = 1.2f, Ki_roll = 0.05f, Kd_roll = 0.15f;
float roll_setpoint = 0.0f;
float pid_integral_roll = 0.0f;
float last_pid_error_roll = 0.0f;

float integral_limit = 25.0f; // Anti-windup limit

// Bluetooth RX Variables
uint8_t rx_byte;
char rx_buffer[20];
uint8_t rx_index = 0;

// Flag to trigger soft reset safely outside ISR
volatile uint8_t reset_requested = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void MPU6050_Init(void);
void MPU6050_Read_All(void);
void MPU6050_Calibrate_Angles(void);
void Servo1_SetAngle(uint8_t angle);
void Servo2_SetAngle(uint8_t angle);
void BLDC_SetThrottle(uint8_t percent);
float Compute_PID_Pitch(float setpoint, float current_pitch, float dt);
float Compute_PID_Roll(float setpoint, float current_roll, float dt);
void Process_Command(char* cmd);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  // 1. Start PWM outputs for both Servos and the BLDC ESC
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // Servo 1 (PA0)
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // Servo 2 (PA1)
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); // BLDC ESC (PA2)

    // 2. Set servos to neutral center (90 deg) & send 0% throttle to arm the ESC
    Servo1_SetAngle(90);
    Servo2_SetAngle(90);
    BLDC_SetThrottle(0);

    // Wait 2 seconds for ESC arming signal sequence
    HAL_Delay(2000);

    // 3. Initialize MPU6050 Sensor
    MPU6050_Init();

    // 4. Calibrate baseline resting pitch AND roll offsets
    MPU6050_Calibrate_Angles();

    // 5. Spin BLDC motor up to continuous background throttle
    BLDC_SetThrottle(BACKGROUND_THROTTLE);

    // 6. Start Bluetooth UART RX Interrupt
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);

    last_time = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  // 1. Check for Bluetooth Soft Reset Flag
	    if (reset_requested)
	    {
	        char msg[] = ">> System Resetting...\r\n";
	        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
	        HAL_Delay(50); // Safe to use HAL_Delay inside main thread context
	        NVIC_SystemReset();
	    }

	    uint32_t current_time = HAL_GetTick();
	    float dt = (current_time - last_time) / 1000.0f; // dt in seconds

	    // Execute Control loop at 50 Hz (20 ms interval)
	    if (dt >= 0.02f)
	    {
	        last_time = current_time;

	        // 1. Read IMU raw acceleration and angular rates
	        MPU6050_Read_All();

	        // 2. Compute Pitch & Roll angles from accelerometer
	        float pitch_acc = atan2f(-Ax, sqrtf(Ay * Ay + Az * Az)) * 57.29578f;
	        float roll_acc  = atan2f(Ay, Az) * 57.29578f;

	        // 3. Complementary Filters
	        pitch = 0.98f * (pitch + Gx * dt) + 0.02f * (pitch_acc - pitch_offset);
	        roll  = 0.98f * (roll  + Gy * dt) + 0.02f * (roll_acc  - roll_offset);

	        // 4. Compute PID Corrections
	        float pid_pitch = Compute_PID_Pitch(pitch_setpoint, pitch, dt);
	        float pid_roll  = Compute_PID_Roll(roll_setpoint, roll, dt);

	        // 5. Elevon Control Surface Mixing Logic
	        float s1_target = 90.0f + pid_pitch + pid_roll;
	        float s2_uninverted = 90.0f + pid_pitch - pid_roll;
	        float s2_target = 180.0f - s2_uninverted;

	        // 6. Safety Clamping (10 to 170 deg)
	        if (s1_target > 170.0f) s1_target = 170.0f;
	        if (s1_target < 10.0f)  s1_target = 10.0f;

	        if (s2_target > 170.0f) s2_target = 170.0f;
	        if (s2_target < 10.0f)  s2_target = 10.0f;

	        // 7. Actuate Servos
	        Servo1_SetAngle((uint8_t)s1_target);
	        Servo2_SetAngle((uint8_t)s2_target);

	        // 8. RX Watchdog
	        if (huart1.RxState == HAL_UART_STATE_READY)
	        {
	            HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 1000;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
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
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
  * @brief Sets Servo 1 position angle (0 to 180 deg).
  */
void Servo1_SetAngle(uint8_t angle)
{
    if (angle > 180) angle = 180;
    uint16_t pulse = 500 + ((uint32_t)angle * 2000) / 180; // 500us to 2500us
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse);
}

/**
  * @brief Sets Servo 2 position angle (0 to 180 deg).
  */
void Servo2_SetAngle(uint8_t angle)
{
    if (angle > 180) angle = 180;
    uint16_t pulse = 500 + ((uint32_t)angle * 2000) / 180; // 500us to 2500us
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pulse);
}

/**
  * @brief Sets BLDC ESC Throttle (0 to 100 %).
  */
void BLDC_SetThrottle(uint8_t percent)
{
    if (percent > 100) percent = 100;
    uint16_t pulse = 1000 + ((uint32_t)percent * 1000) / 100; // 1000us to 2000us
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pulse);
}

/**
  * @brief Initializes MPU6050 power registers and DLPF settings.
  */
void MPU6050_Init(void)
{
    uint8_t check, data;
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x75, 1, &check, 1, 1000);
    if (check == 0x68)
    {
        data = 0x00; // Wake up MPU6050
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x6B, 1, &data, 1, 1000);
        data = 0x03; // DLPF ~42Hz bandwidth filter
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1A, 1, &data, 1, 1000);
        data = 0x00; // Accel range +/- 2g
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1C, 1, &data, 1, 1000);
        data = 0x00; // Gyro range +/- 250 deg/s
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1B, 1, &data, 1, 1000);
    }
}

/**
  * @brief Reads 14 sequential bytes from MPU6050 and converts raw readings.
  */
void MPU6050_Read_All(void)
{
    uint8_t raw[14];
    if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x3B, 1, raw, 14, 1000) == HAL_OK)
    {
        Accel_X_RAW = (int16_t)(raw[0] << 8 | raw[1]);
        Accel_Y_RAW = (int16_t)(raw[2] << 8 | raw[3]);
        Accel_Z_RAW = (int16_t)(raw[4] << 8 | raw[5]);
        Gyro_X_RAW  = (int16_t)(raw[8] << 8 | raw[9]);
        Gyro_Y_RAW  = (int16_t)(raw[10] << 8 | raw[11]);
        Gyro_Z_RAW  = (int16_t)(raw[12] << 8 | raw[13]);

        Ax = Accel_X_RAW / 16384.0f;
        Ay = Accel_Y_RAW / 16384.0f;
        Az = Accel_Z_RAW / 16384.0f;
        Gx = Gyro_X_RAW / 131.0f;
        Gy = Gyro_Y_RAW / 131.0f;
        Gz = Gyro_Z_RAW / 131.0f;
    }
}

// Keep your existing Servo1_SetAngle, Servo2_SetAngle, BLDC_SetThrottle,
// MPU6050_Init, and MPU6050_Read_All functions as they are.

/**
  * @brief Collects static samples at boot to establish Pitch AND Roll resting offsets.
  */
void MPU6050_Calibrate_Angles(void)
{
    float pitch_sum = 0.0f;
    float roll_sum  = 0.0f;

    for (int i = 0; i < 200; i++)
    {
        MPU6050_Read_All();
        float p = atan2f(-Ax, sqrtf(Ay * Ay + Az * Az)) * 57.29578f;
        float r = atan2f(Ay, Az) * 57.29578f;

        pitch_sum += p;
        roll_sum  += r;
        HAL_Delay(5);
    }
    pitch_offset = pitch_sum / 200.0f;
    roll_offset  = roll_sum  / 200.0f;
}

/**
  * @brief Computes PID correction output for submarine pitch stabilization.
  */
float Compute_PID_Pitch(float setpoint, float current_pitch, float dt)
{
    float error = setpoint - current_pitch;
    float p_term = Kp_pitch * error;

    pid_integral_pitch += error * dt;
    if (pid_integral_pitch > integral_limit)  pid_integral_pitch = integral_limit;
    if (pid_integral_pitch < -integral_limit) pid_integral_pitch = -integral_limit;
    float i_term = Ki_pitch * pid_integral_pitch;

    float derivative = (dt > 0.0f) ? ((error - last_pid_error_pitch) / dt) : 0.0f;
    last_pid_error_pitch = error;
    float d_term = Kd_pitch * derivative;

    return (p_term + i_term + d_term);
}

/**
  * @brief Computes PID correction output for submarine roll stabilization.
  */
float Compute_PID_Roll(float setpoint, float current_roll, float dt)
{
    float error = setpoint - current_roll;
    float p_term = Kp_roll * error;

    pid_integral_roll += error * dt;
    if (pid_integral_roll > integral_limit)  pid_integral_roll = integral_limit;
    if (pid_integral_roll < -integral_limit) pid_integral_roll = -integral_limit;
    float i_term = Ki_roll * pid_integral_roll;

    float derivative = (dt > 0.0f) ? ((error - last_pid_error_roll) / dt) : 0.0f;
    last_pid_error_roll = error;
    float d_term = Kd_roll * derivative;

    return (p_term + i_term + d_term);
}

/**
  * @brief Parses incoming text string for soft reset command.
  */
void Process_Command(char* cmd)
{
    // Strip trailing carriage returns or line feeds
    char *p = cmd;
    while (*p) {
        if (*p == '\r' || *p == '\n') {
            *p = '\0';
            break;
        }
        p++;
    }

    // Check for 'r' or 'R' command
    if (strcmp(cmd, "r") == 0 || strcmp(cmd, "R") == 0)
    {
        char msg[] = ">> System Resetting...\r\n";
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
        HAL_Delay(50);       // Wait for UART buffer to flush
        NVIC_SystemReset();  // Trigger Cortex-M Software Reset
    }
}
/**
  * @brief Non-blocking UART receive callback. Sets reset flag immediately on 'r' or 'R'.
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        // Immediate single-character soft reset check
        if (rx_byte == 'r' || rx_byte == 'R')
        {
            reset_requested = 1; // Flag reset to be executed in main loop
        }
        else if (rx_byte == '\r' || rx_byte == '\n')
        {
            if (rx_index > 0)
            {
                rx_buffer[rx_index] = '\0';

                // String command check for 'r' or 'R'
                if (strcmp(rx_buffer, "r") == 0 || strcmp(rx_buffer, "R") == 0)
                {
                    reset_requested = 1;
                }
                rx_index = 0;
            }
        }
        else
        {
            if (rx_index < sizeof(rx_buffer) - 1)
            {
                rx_buffer[rx_index++] = rx_byte;
            }
            else
            {
                rx_index = 0;
            }
        }

        // Re-enable interrupt for next byte
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
}

/**
  * @brief Handles UART overrun recovery automatically.
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
