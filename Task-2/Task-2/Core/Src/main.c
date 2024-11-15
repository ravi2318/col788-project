#include "stm32l0xx_hal.h"
#include "MAX30102.h"
#include "filter.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include "main.h"

// UART and I2C Handles
UART_HandleTypeDef huart2;
I2C_HandleTypeDef hi2c1;

// Sensor and Sampling Configuration
const float kSamplingFrequency = 400.0;  // Sampling frequency in Hz
const unsigned long kFingerThreshold = 10000;  // Finger detection threshold
const unsigned int kFingerCooldownMs = 100;    // Cooldown time between readings in milliseconds
const float kEdgeThreshold = -2000.0;  // Edge detection threshold for heartbeat

// Filter Configuration
const float kLowPassCutoff = 3.0;  // Low pass filter cutoff frequency
const float kHighPassCutoff = 0.5; // High pass filter cutoff frequency
#define K_AVERAGING_SAMPLES 50  // Number of samples for moving average
const bool kEnableAveraging = true;
const int kSampleThreshold = 10;  // Minimum number of samples for valid output

// SpO2 Calibration Factors
float kSpO2_A = 1.5958422;
float kSpO2_B = -34.6596622;
float kSpO2_C = 112.6898759;

// Filter Instances
HighPassFilter high_pass_filter;
LowPassFilter low_pass_filter_red, low_pass_filter_ir;
Differentiator differentiator;
MovingAverageFilter averager_bpm, averager_spo2;

// Variables for heartbeat detection
uint32_t last_heartbeat = 0;
uint32_t finger_timestamp = 0;
bool finger_detected = false;
float last_diff = NAN;
bool crossed = false;
uint32_t crossed_time = 0;

// SpO2 statistics
MinMaxAvgStatistic stat_red, stat_ir;

// Function Prototypes
void Error_Handler(void);
void reset_filters(void);
void detect_heartbeat(uint32_t current_time, float current_diff, float current_value_ir);
void send_uart(char *message);
static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
void init_filters(void);

// UART transmit helper function
void send_uart(char *message) {
    HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
}

// Main application loop
int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USART2_UART_Init();

    // Initialize I2C and MAX30102 sensor
    MAX30102_Init(&hi2c1);

    // Initialize filters for processing
    init_filters();

    // Variables for periodic temperature reading
    uint32_t last_temp_read_time = 0;
    const uint32_t temp_read_interval_ms = 5000;  // Read temperature every 5 seconds

    // Main loop
    while (1) {
        // Read data from the MAX30102 sensor
        MAX30102_Data sample;
        MAX30102_ReadFifo(&hi2c1, &sample);
        float current_value_red = sample.red;  // Red LED value
        float current_value_ir = sample.ir;    // IR LED value

        // Get the current system time
        uint32_t current_time = HAL_GetTick();

        // Finger detection logic
        if (current_value_red > kFingerThreshold) {
            if (current_time - finger_timestamp > kFingerCooldownMs) {
                finger_detected = true;
            }
        } else {
            // If no finger is detected, reset filters
            reset_filters();
            finger_detected = false;
            finger_timestamp = current_time;
            send_uart("No finger detected, resetting filters\n\r");
        }

        // Periodic temperature reading
        if (current_time - last_temp_read_time >= temp_read_interval_ms) {
            last_temp_read_time = current_time;  // Update last temperature read time
//            float temperature = MAX30102_ReadTemperature(&hi2c1);  // Read sensor temperature
//            char temp_message[64];
//            snprintf(temp_message, sizeof(temp_message), "Temperature: %.2fC\n\r", temperature);
//            send_uart(temp_message);  // Send temperature reading over UART
        }

        // If finger is detected, process data for heartbeat and SpO2 calculation
        if (finger_detected) {
            // Apply low-pass filters to the red and IR values
            current_value_red = LowPassFilter_Process(&low_pass_filter_red, current_value_red);
            current_value_ir = LowPassFilter_Process(&low_pass_filter_ir, current_value_ir);
            float current_diff = Differentiator_Process(&differentiator, current_value_red);

            // Update SpO2 statistics
            MinMaxAvgStatistic_Process(&stat_red, current_value_red);
            MinMaxAvgStatistic_Process(&stat_ir, current_value_ir);

            // Perform heartbeat detection and SpO2 calculation
            detect_heartbeat(current_time, current_diff, current_value_ir);

            last_diff = current_diff;  // Update last difference for heartbeat detection
        }
    }
}

// Filter initialization
void init_filters(void) {
    // Initialize filters with respective cutoff frequencies
    HighPassFilter_InitWithCutoff(&high_pass_filter, kHighPassCutoff, kSamplingFrequency);
    LowPassFilter_InitWithCutoff(&low_pass_filter_red, kLowPassCutoff, kSamplingFrequency);
    LowPassFilter_InitWithCutoff(&low_pass_filter_ir, kLowPassCutoff, kSamplingFrequency);
    Differentiator_Init(&differentiator, kSamplingFrequency);

    // Initialize moving average filters for BPM and SpO2
    static float average_buffer_bpm[K_AVERAGING_SAMPLES];
    static float average_buffer_spo2[K_AVERAGING_SAMPLES];
    MovingAverageFilter_Init(&averager_bpm, average_buffer_bpm, K_AVERAGING_SAMPLES);
    MovingAverageFilter_Init(&averager_spo2, average_buffer_spo2, K_AVERAGING_SAMPLES);

    // Initialize statistics for SpO2 calculation
    MinMaxAvgStatistic_Init(&stat_red);
    MinMaxAvgStatistic_Init(&stat_ir);
}

// Reset filters when finger is removed
void reset_filters(void) {
    HighPassFilter_Reset(&high_pass_filter);
    LowPassFilter_Reset(&low_pass_filter_red);
    LowPassFilter_Reset(&low_pass_filter_ir);
    Differentiator_Reset(&differentiator);
    MovingAverageFilter_Reset(&averager_bpm);
    MovingAverageFilter_Reset(&averager_spo2);
    MinMaxAvgStatistic_Reset(&stat_red);
    MinMaxAvgStatistic_Reset(&stat_ir);
}

// Heartbeat detection and SpO2 calculation
void detect_heartbeat(uint32_t current_time, float current_diff, float current_value_ir) {
    if (!isnan(current_diff) && !isnan(last_diff)) {
        // Detect edge crossing for heartbeat detection
        if (last_diff > 0 && current_diff < 0) {
            crossed = true;
            crossed_time = current_time;
        }

        // Reset crossing flag
        if (current_diff > 0) {
            crossed = false;
        }

        // If a crossing occurred and meets threshold criteria
        if (crossed && current_diff < kEdgeThreshold) {
            if (last_heartbeat != 0 && (crossed_time - last_heartbeat) > 500) {
                uint32_t bpm = 60000 / (crossed_time - last_heartbeat);  // Calculate BPM

                // Only consider valid BPM ranges
                if (bpm > 50 && bpm < 150) {
                    // Calculate SpO2 ratio
                    float r = (MinMaxAvgStatistic_Maximum(&stat_red) - MinMaxAvgStatistic_Minimum(&stat_red)) /
                              MinMaxAvgStatistic_Average(&stat_red);
                    r /= (MinMaxAvgStatistic_Maximum(&stat_ir) - MinMaxAvgStatistic_Minimum(&stat_ir)) /
                         MinMaxAvgStatistic_Average(&stat_ir);

                    // Calculate SpO2 value
                    float spo2 = kSpO2_A * r * r + kSpO2_B * r + kSpO2_C;
                    spo2 = fmaxf(0, fminf(100, spo2));  // Constrain SpO2 between 0 and 100

                    // Process averaging if enabled
                    if (kEnableAveraging) {
                        int average_bpm = MovingAverageFilter_Process(&averager_bpm, bpm);
                        float average_spo2 = MovingAverageFilter_Process(&averager_spo2, spo2);

                        // Print if sufficient samples are collected
                        if (averager_bpm.count >= kSampleThreshold) {
                            char message[64];
                            snprintf(message, sizeof(message), "Heart Rate (avg, bpm): %d\n\r", average_bpm);
                            send_uart(message);
                            snprintf(message, sizeof(message), "SpO2 (avg, %%): %.2f\n\r", average_spo2);
                            send_uart(message);
                        }
                    } else {
                        // Print current readings if averaging is disabled
                        char message[64];
                        snprintf(message, sizeof(message), "Heart Rate (current, bpm): %lu\n\r", bpm);
                        send_uart(message);
                        snprintf(message, sizeof(message), "SpO2 (current, %%): %.2f\n\r", spo2);
                        send_uart(message);
                    }
                }
            }

            // Update last heartbeat timestamp and reset statistics
            last_heartbeat = crossed_time;
            crossed = false;
            MinMaxAvgStatistic_Reset(&stat_red);
            MinMaxAvgStatistic_Reset(&stat_ir);
        }
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == temp_Pin) {
        // Temperature interrupt detected, read and print temperature
        float temperature = MAX30102_ReadTemperature(&hi2c1);
        char temp_message[64];
        snprintf(temp_message, sizeof(temp_message), "Temperature: %.2f°C\r\n", temperature);
        send_uart(temp_message);
    }
}
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hi2c1.Init.Timing = 0x00000608;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : temp_Pin */
  GPIO_InitStruct.Pin = temp_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(temp_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

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
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
