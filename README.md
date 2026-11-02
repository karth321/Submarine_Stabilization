# Project Report: Submarine Automatic Pitch & Roll Stabilization

The project is ready to use as is. Make the required hardware connections.
If uploading via USB, upload Debug/PitchControl.hex to the board using STM32CubeProgrammer.
If uploading via ST Link, open the project in STM32CubeIDE, click run.

## 1. Project Overview

This project implements an automatic flight control system for a submarine using an **STM32F401CEU6 Black Pill** microcontroller. By fusing accelerometer and gyroscope data from an **MPU6050 IMU**, the system calculates the submarine's real-time pitch (nose up/down) and roll (tilt left/right).

Two dive plane servos are controlled via a **PID controller** using **Elevon Mixing**:

* **Pitch Correction:** Both servos move symmetrically.

* **Roll Correction:** Servos move differentially (opposite directions).

A continuous BLDC background motor provides forward propulsion, while an **HC-05 Bluetooth module** provides real-time telemetry and allows for a remote software reset.

## 2. Hardware Components & Wiring

### Bill of Materials

* STM32F401CEU6 Black Pill

* MPU6050 6-Axis IMU

* HC-05 Bluetooth Module

* 2x Standard Servos

* 1x BLDC Motor + ESC

* 11.1V Li-ion Battery + 5V Buck Converter (for MCU and Servos)

### Wiring Connections

| Component | Pin Name | Black Pill Target | Description / Notes | 
| ----- | ----- | ----- | ----- | 
| **MPU6050** | VCC | **3.3V / 5V** | Power supply | 
|  | GND | **GND** | Shared ground | 
|  | SCL | **PB6** | I2C1_SCL | 
|  | SDA | **PB7** | I2C1_SDA | 
|  | AD0 | **GND** | Address 0x68 (0xD0 HAL format) | 
| **HC-05** | VCC | **5V Pin** | 5V Power | 
|  | GND | **GND Pin** | Shared ground | 
|  | RXD | **PA9** | Connects to STM32 TX (USART1_TX) | 
|  | TXD | **PA10** | Connects to STM32 RX (USART1_RX) | 
| **Servo 1** | Signal | **PA0** | TIM2_CH1 PWM output | 
| **Servo 2** | Signal | **PA1** | TIM2_CH2 PWM output | 
| **BLDC ESC** | Signal | **PA2** | TIM2_CH3 PWM output | 

*(Note: Ensure the ESC ground, Buck converter ground, and STM32 ground are all tied together for common reference).*

## 3. STM32CubeMX Configuration

### A. Clock Setup

1. **System Core -> RCC:** Set **HSE** to **Crystal/Ceramic Resonator**.

2. **Clock Configuration Tab:** Set Input Frequency to **25 MHz** and **HCLK** (System Clock) to **84 MHz**.

### B. Peripherals Setup

1. **I2C1 (MPU6050):**

   * **Mode:** I2C

   * **Speed:** Standard Mode (100 kHz)

   * Verify pins: PB6 (SCL), PB7 (SDA).

2. **TIM2 (PWM Generation):**

   * **Clock Source:** Internal Clock

   * **Channel 1:** PWM Generation CH1 (PA0)

   * **Channel 2:** PWM Generation CH2 (PA1)

   * **Channel 3:** PWM Generation CH3 (PA2)

   * **Parameter Settings:**

     * Prescaler (PSC): `83` (Yields 1 MHz tick)

     * Counter Period (ARR): `19999` (20 ms period / 50 Hz)

     * Pulse CH1 & CH2: `1500` (1.5 ms = 90° center)

     * Pulse CH3: `1000` (1.0 ms = ESC 0% arming throttle)

3. **USART1 (Bluetooth):**

   * **Mode:** Asynchronous

   * **Baud Rate:** 9600 Bits/s (HC-05 default)

   * **NVIC Settings:** Check **USART1 global interrupt -> Enabled**.

## 4. STM32CubeIDE Setup

Before compiling the code, configure two essential project properties:

### Enable Float Formatting for Bluetooth Output

1. Right-click your Project -> **Properties** -> **C/C++ Build** -> **Settings**.

2. Go to the **Tool Settings** tab -> **MCU Settings**.

3. Check the box: **"Use float with printf from newlib-nano (-u \_printf_float)"**.

4. Click Apply.

### Generate Intel HEX File for USB Flashing

1. Still in **Properties** -> **C/C++ Build** -> **Settings**.

2. Go to **MCU Post build outputs**.

3. Check the box: **"Convert to Intel Hex file (-O ihex)"**.

4. Click Apply and Close.

5. Build the project (`Ctrl+B`). This generates a `.hex` file in your `Debug/` folder.

## 5. Main Code Implementation

The following is the complete, compiled `main.c` code handling I2C communication, PID calculation, servo mixing, and non-blocking UART interrupts.
Code is available in Core/Src/main.c

```
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - Submarine Auto-Pitch/Roll Control
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

int main(void)
{
  HAL_Init();
  SystemClock_Config();

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

  while (1)
  {
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

        // Read IMU and compute filtering
        MPU6050_Read_All();
        float pitch_acc = atan2f(-Ax, sqrtf(Ay * Ay + Az * Az)) * 57.29578f;
        float roll_acc  = atan2f(Ay, Az) * 57.29578f;

        pitch = 0.98f * (pitch + Gx * dt) + 0.02f * (pitch_acc - pitch_offset);
        roll  = 0.98f * (roll  + Gy * dt) + 0.02f * (roll_acc  - roll_offset);

        // Compute PID Corrections
        float pid_pitch = Compute_PID_Pitch(pitch_setpoint, pitch, dt);
        float pid_roll  = Compute_PID_Roll(roll_setpoint, roll, dt);

        // Elevon Control Surface Mixing Logic
        float s1_target = 90.0f + pid_pitch + pid_roll;
        float s2_uninverted = 90.0f + pid_pitch - pid_roll;
        float s2_target = 180.0f - s2_uninverted;

        // Safety Clamping (10 to 170 deg)
        if (s1_target > 170.0f) s1_target = 170.0f;
        if (s1_target < 10.0f)  s1_target = 10.0f;
        if (s2_target > 170.0f) s2_target = 170.0f;
        if (s2_target < 10.0f)  s2_target = 10.0f;

        // Actuate Servos
        Servo1_SetAngle((uint8_t)s1_target);
        Servo2_SetAngle((uint8_t)s2_target);

        // RX Watchdog to restart interrupt if overrun occurs
        if (huart1.RxState == HAL_UART_STATE_READY)
        {
            HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
        }
    }
  }
  /* USER CODE END 3 */
}

/* Include Auto-Generated Peripheral Init Functions Here (SystemClock_Config, MX_I2C1_Init, etc.) */

/* USER CODE BEGIN 4 */
void Servo1_SetAngle(uint8_t angle) {
    if (angle > 180) angle = 180;
    uint16_t pulse = 500 + ((uint32_t)angle * 2000) / 180; 
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse);
}

void Servo2_SetAngle(uint8_t angle) {
    if (angle > 180) angle = 180;
    uint16_t pulse = 500 + ((uint32_t)angle * 2000) / 180; 
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pulse);
}

void BLDC_SetThrottle(uint8_t percent) {
    if (percent > 100) percent = 100;
    uint16_t pulse = 1000 + ((uint32_t)percent * 1000) / 100;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pulse);
}

void MPU6050_Init(void) {
    uint8_t check, data;
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x75, 1, &check, 1, 1000);
    if (check == 0x68) {
        data = 0x00; HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x6B, 1, &data, 1, 1000);
        data = 0x03; HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1A, 1, &data, 1, 1000);
        data = 0x00; HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1C, 1, &data, 1, 1000);
        data = 0x00; HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1B, 1, &data, 1, 1000);
    }
}

void MPU6050_Read_All(void) {
    uint8_t raw[14];
    if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x3B, 1, raw, 14, 1000) == HAL_OK) {
        Accel_X_RAW = (int16_t)(raw[0] << 8 | raw[1]);
        Accel_Y_RAW = (int16_t)(raw[2] << 8 | raw[3]);
        Accel_Z_RAW = (int16_t)(raw[4] << 8 | raw[5]);
        Gyro_X_RAW  = (int16_t)(raw[8] << 8 | raw[9]);
        Gyro_Y_RAW  = (int16_t)(raw[10] << 8 | raw[11]);
        Gyro_Z_RAW  = (int16_t)(raw[12] << 8 | raw[13]);

        Ax = Accel_X_RAW / 16384.0f; Ay = Accel_Y_RAW / 16384.0f; Az = Accel_Z_RAW / 16384.0f;
        Gx = Gyro_X_RAW / 131.0f;    Gy = Gyro_Y_RAW / 131.0f;    Gz = Gyro_Z_RAW / 131.0f;
    }
}

void MPU6050_Calibrate_Angles(void) {
    float pitch_sum = 0.0f, roll_sum = 0.0f;
    for (int i = 0; i < 200; i++) {
        MPU6050_Read_All();
        pitch_sum += atan2f(-Ax, sqrtf(Ay * Ay + Az * Az)) * 57.29578f;
        roll_sum  += atan2f(Ay, Az) * 57.29578f;
        HAL_Delay(5);
    }
    pitch_offset = pitch_sum / 200.0f;
    roll_offset  = roll_sum  / 200.0f;
}

float Compute_PID_Pitch(float setpoint, float current_pitch, float dt) {
    float error = setpoint - current_pitch;
    float p_term = Kp_pitch * error;

    pid_integral_pitch += error * dt;
    if (pid_integral_pitch > integral_limit)  pid_integral_pitch = integral_limit;
    if (pid_integral_pitch < -integral_limit) pid_integral_pitch = -integral_limit;
    
    float derivative = (dt > 0.0f) ? ((error - last_pid_error_pitch) / dt) : 0.0f;
    last_pid_error_pitch = error;
    
    return (p_term + (Ki_pitch * pid_integral_pitch) + (Kd_pitch * derivative));
}

float Compute_PID_Roll(float setpoint, float current_roll, float dt) {
    float error = setpoint - current_roll;
    float p_term = Kp_roll * error;

    pid_integral_roll += error * dt;
    if (pid_integral_roll > integral_limit)  pid_integral_roll = integral_limit;
    if (pid_integral_roll < -integral_limit) pid_integral_roll = -integral_limit;
    
    float derivative = (dt > 0.0f) ? ((error - last_pid_error_roll) / dt) : 0.0f;
    last_pid_error_roll = error;

    return (p_term + (Ki_roll * pid_integral_roll) + (Kd_roll * derivative));
}

void Process_Command(char* cmd) {
    char *p = cmd;
    while (*p) { if (*p == '\r' || *p == '\n') { *p = '\0'; break; } p++; }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        if (rx_byte == 'r' || rx_byte == 'R') {
            reset_requested = 1; 
        }
        else if (rx_byte == '\r' || rx_byte == '\n') {
            if (rx_index > 0) {
                rx_buffer[rx_index] = '\0';
                if (strcmp(rx_buffer, "r") == 0 || strcmp(rx_buffer, "R") == 0) {
                    reset_requested = 1;
                }
                rx_index = 0;
            }
        }
        else {
            if (rx_index < sizeof(rx_buffer) - 1) rx_buffer[rx_index++] = rx_byte;
            else rx_index = 0;
        }
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
}
/* USER CODE END 4 */

```

## 6. How to Flash the STM32 via USB (No ST-LINK Required)

You will use **STM32CubeProgrammer** to upload the `.hex` file directly over USB.

1. **Enter Bootloader Mode:**

   * Hold the `BOOT0` button on the Black Pill.

   * Press and release the `NRST` (Reset) button.

   * Release the `BOOT0` button.

2. **Connect via USB:**

   * Open STM32CubeProgrammer.

   * Set Connection to **USB** (top right dropdown).

   * Click **Connect**. The STM32F401 should be detected automatically.

3. **Upload the Code:**

   * Go to the **Erasing & Programming** tab.

   * Click **Browse** and locate your `Debug/YourProjectName.hex` file.

   * Click **Download**.

4. **Boot the Application:**

   * Disconnect from STM32CubeProgrammer.

   * Ensure `BOOT0` is unpressed.

   * Press the `NRST` button once to reset the board into your newly flashed code.

## 7. Operation & Expected Output

### System Boot & Calibration

When you power on the board or press the physical reset button:

1. The ESC will receive a `1000us` (0%) signal for 2 seconds to arm safely.

2. The MPU6050 will take 200 static readings over 1 second to establish a baseline "zero" offset for pitch and roll. **Do not move the submarine during this time.**

3. The BLDC motor will automatically spin up to `30%` background throttle.

### Real-Time Pitch and Roll Control

* **Pitch Correction:** If you tilt the nose of the submarine down (negative pitch), the PID loop calculates an immediate counteracting force. Both servos will simultaneously deflect upwards past 90° to push the nose back level.

* **Roll Correction:** If you roll the submarine to the left, the Elevon mixing logic pushes the left servo down and the right servo up, creating a twisting force to stabilize it horizontally.

* **Damping (Kd Term):** As the submarine approaches level flight (0°), the derivative term gently backs off the servos to prevent overshoot or wobble.

### Remote Software Reset via Bluetooth

If you need to re-calibrate the submarine while it's in the water without opening the hull to press the physical reset button:

1. Connect your phone to the HC-05 module via the **Serial Bluetooth Terminal** app.

2. Type `r` or `R` and press Send.

3. Your terminal will display `>> System Resetting...`

4. The ARM Cortex-M core triggers a `NVIC_SystemReset()`.

5. The system immediately re-arms the ESC, zeroes out accumulated integral error, and re-calibrates the MPU6050 offsets to whatever its current resting orientation is.
