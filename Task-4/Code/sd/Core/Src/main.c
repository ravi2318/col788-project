#include "stm32l0xx_hal.h"
#include "MAX30102.h"
#include "filter.h"
#include "fatfs.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include "main.h"

// UART, I2C, and SPI Handles
UART_HandleTypeDef huart2;
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi2;
RTC_HandleTypeDef hrtc;

// FatFs Variables
FATFS FatFs;
FIL fil;
FRESULT fres;
bool log_file_initialized = false;  // Flag to track if the log file is initialized

// Sensor and Sampling Configuration
const float kSamplingFrequency = 400.0f;
const unsigned long kFingerThreshold = 10000;
const unsigned int kFingerCooldownMs = 100;
const float kEdgeThreshold = -2000.0f;

// Filter Configuration
const float kLowPassCutoff = 3.0f;
const float kHighPassCutoff = 0.5f;
#define K_AVERAGING_SAMPLES 50
const bool kEnableAveraging = true;
const int kSampleThreshold = 10;

// SpO2 Calibration Factors
float kSpO2_A = 1.5958422;
float kSpO2_B = -34.6596622;
float kSpO2_C = 112.6898759;

// Filter Instances
HighPassFilter high_pass_filter;
LowPassFilter low_pass_filter_red, low_pass_filter_ir;
Differentiator differentiator;
MovingAverageFilter averager_bpm, averager_spo2;

// Heartbeat Detection Variables
uint32_t last_heartbeat = 0;
uint32_t finger_timestamp = 0;
bool finger_detected = false;
float last_diff = NAN;
bool crossed = false;
uint32_t crossed_time = 0;

// SpO2 Statistics
MinMaxAvgStatistic stat_red, stat_ir;

// Interrupt Flag
volatile bool temp_interrupt_flag = false;
volatile bool stop_logging_flag = false;

// Function Prototypes
void Error_Handler(void);
void reset_filters(void);
void detect_heartbeat(uint32_t current_time, float current_diff, float current_value_ir);
void send_uart(char *message);
void log_data_to_sd(char *message);
void initiate_temperature_read(void);
void log_data_to_sd(char *message);
void stop_operation(void);  // Function to stop calculations and logging
void Set_RTC_Time(void);
void Set_RTC_Date(void);

// Initialization Functions
static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
void init_filters(void);
static void MX_RTC_Init(void); // RTC initialization

// UART Transmit Helper Function
void send_uart(char *message) {
    HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
}

// SD Card Logging Function with Timestamp
void log_data_to_sd(char *message) {
    if (stop_logging_flag) return;  // Stop logging if flag is set

    RTC_DateTypeDef sDate;
    RTC_TimeTypeDef sTime;

    // Get the current date and time from RTC
    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);

    char log_entry[128];
    snprintf(log_entry, sizeof(log_entry), "[%02d-%02d-%02d %02d:%02d:%02d] %s",
             sDate.Year, sDate.Month, sDate.Date, sTime.Hours, sTime.Minutes, sTime.Seconds, message);

    // Open log file in append mode to avoid overwriting previous logs
    fres = f_open(&fil, "log.txt", log_file_initialized ? (FA_OPEN_APPEND | FA_WRITE) : (FA_CREATE_ALWAYS | FA_WRITE));
    if (fres == FR_OK) {
        UINT bytesWritten;
        f_write(&fil, log_entry, strlen(log_entry), &bytesWritten);
        f_close(&fil);

        // Set the log file initialized flag to true after the first write
        log_file_initialized = true;
    } else {
        send_uart("Failed to write to SD card.\n\r");
    }
}


// Main Application Loop
int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USART2_UART_Init();
    MX_SPI2_Init();
    MX_RTC_Init();  // Initialize RTC
    MX_FATFS_Init();

    // Mount SD card
    fres = f_mount(&FatFs, "", 1);
    if (fres != FR_OK) {
        send_uart("SD card mount error.\n\r");
    } else {
        send_uart("SD card mounted successfully.\n\r");
    }

    // Initialize I2C and MAX30102 sensor
    MAX30102_Init(&hi2c1);

    // Initialize filters for processing
    init_filters();

    // Variables for periodic temperature reading
    uint32_t last_temp_read_time = 0;
    const uint32_t temp_read_interval_ms = 5000;

    while (1) {
        if (stop_logging_flag) continue;  // Stop all processing if the flag is set
//        Set_RTC_Time();
//           Set_RTC_Date();
        if (temp_interrupt_flag) {
            temp_interrupt_flag = false;
            float temperature = MAX30102_ReadTemperature(&hi2c1);
            char temp_message[64];
            snprintf(temp_message, sizeof(temp_message), "Temperature: %.2f°C\r\n", temperature);
            send_uart(temp_message);
            log_data_to_sd(temp_message);
        }

        MAX30102_Data sample;
        MAX30102_ReadFifo(&hi2c1, &sample);
        float current_value_red = sample.red;
        float current_value_ir = sample.ir;
        uint32_t current_time = HAL_GetTick();

        if (current_value_red > kFingerThreshold) {
            if (current_time - finger_timestamp > kFingerCooldownMs) {
                finger_detected = true;
            }
        } else {
            if (finger_detected) {
                send_uart("No finger detected, resetting filters\n\r");
                log_data_to_sd("No finger detected, resetting filters\n\r");
                reset_filters();
            }
            finger_detected = false;
            finger_timestamp = current_time;
        }

        if (current_time - last_temp_read_time >= temp_read_interval_ms) {
            last_temp_read_time = current_time;
            initiate_temperature_read();
        }

        if (finger_detected) {
            current_value_red = LowPassFilter_Process(&low_pass_filter_red, current_value_red);
            current_value_ir = LowPassFilter_Process(&low_pass_filter_ir, current_value_ir);
            float current_diff = Differentiator_Process(&differentiator, current_value_red);

            MinMaxAvgStatistic_Process(&stat_red, current_value_red);
            MinMaxAvgStatistic_Process(&stat_ir, current_value_ir);

            detect_heartbeat(current_time, current_diff, current_value_ir);
            last_diff = current_diff;
        }
    }
}



// Start Temperature Measurement
void initiate_temperature_read(void) {
    MAX30102_WriteRegister(&hi2c1, MAX30102_REG_TEMP_CONFIG, 0x01);
}

// Filter Initialization
void init_filters(void) {
    HighPassFilter_InitWithCutoff(&high_pass_filter, kHighPassCutoff, kSamplingFrequency);
    LowPassFilter_InitWithCutoff(&low_pass_filter_red, kLowPassCutoff, kSamplingFrequency);
    LowPassFilter_InitWithCutoff(&low_pass_filter_ir, kLowPassCutoff, kSamplingFrequency);
    Differentiator_Init(&differentiator, kSamplingFrequency);

    static float average_buffer_bpm[K_AVERAGING_SAMPLES];
    static float average_buffer_spo2[K_AVERAGING_SAMPLES];
    MovingAverageFilter_Init(&averager_bpm, average_buffer_bpm, K_AVERAGING_SAMPLES);
    MovingAverageFilter_Init(&averager_spo2, average_buffer_spo2, K_AVERAGING_SAMPLES);

    MinMaxAvgStatistic_Init(&stat_red);
    MinMaxAvgStatistic_Init(&stat_ir);
}

// Reset Filters When Finger is Removed
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

// Heartbeat Detection and SpO2 Calculation
void detect_heartbeat(uint32_t current_time, float current_diff, float current_value_ir) {
    if (!isnan(current_diff) && !isnan(last_diff)) {
        if (last_diff > 0 && current_diff < 0) {
            crossed = true;
            crossed_time = current_time;
        }

        if (current_diff > 0) {
            crossed = false;
        }

        if (crossed && current_diff < kEdgeThreshold) {
            if (last_heartbeat != 0 && (crossed_time - last_heartbeat) > 500) {
                uint32_t bpm = 60000 / (crossed_time - last_heartbeat);
                if (bpm > 50 && bpm < 150) {
                    float r = (MinMaxAvgStatistic_Maximum(&stat_red) - MinMaxAvgStatistic_Minimum(&stat_red)) /
                              MinMaxAvgStatistic_Average(&stat_red);
                    r /= (MinMaxAvgStatistic_Maximum(&stat_ir) - MinMaxAvgStatistic_Minimum(&stat_ir)) /
                         MinMaxAvgStatistic_Average(&stat_ir);

                    float spo2 = kSpO2_A * r * r + kSpO2_B * r + kSpO2_C;
                    spo2 = fmaxf(0, fminf(100, spo2));

                    char message[64];
                    if (kEnableAveraging) {
                        int average_bpm = MovingAverageFilter_Process(&averager_bpm, bpm);
                        float average_spo2 = MovingAverageFilter_Process(&averager_spo2, spo2);
                        if (averager_bpm.count >= kSampleThreshold) {
                            snprintf(message, sizeof(message), "Heart Rate: %d bpm, SpO2: %.2f%%\n\r", average_bpm, average_spo2);
                            send_uart(message);
                            log_data_to_sd(message);
                        }
                    } else {
                        snprintf(message, sizeof(message), "Heart Rate: %lu bpm, SpO2: %.2f%%\n\r", bpm, spo2);
                        send_uart(message);
                        log_data_to_sd(message);
                    }
                }
            }
            last_heartbeat = crossed_time;
            crossed = false;
            MinMaxAvgStatistic_Reset(&stat_red);
            MinMaxAvgStatistic_Reset(&stat_ir);
        }
    }
}

// GPIO Interrupt Callback
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_13) {
        stop_logging_flag = true;  // Set flag to stop logging and calculations
        send_uart("Logging and calculations stopped by button press.\n\r");
    } else if (GPIO_Pin == temp_Pin) {
        temp_interrupt_flag = true;
    }
}

void Error_Handler(void)
{
    while(1);
}
//void Set_RTC_Time(void) {
//    RTC_TimeTypeDef sTime = {0};
//
//    // Set the time (example: 10:30:45 AM)
//    sTime.Hours = 8;
//    sTime.Minutes = 30;
//    sTime.Seconds = 30;
//    sTime.TimeFormat = RTC_HOURFORMAT12_AM;
//
//    if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) {
//        Error_Handler();
//    }
//}

// Function to set the RTC date
//void Set_RTC_Date(void) {
//    RTC_DateTypeDef sDate = {0};
//
//    // Set the date (example: September 18, 2024)
//    sDate.WeekDay = RTC_WEEKDAY_THURSDAY;
//    sDate.Month = RTC_MONTH_NOVEMBER;
//    sDate.Date = 8;
//    sDate.Year = 24;  // Year in two-digit format (2024 -> 24)
//
//    if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) {
//        Error_Handler();
//    }
//}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_RTC;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
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
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */



  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }



}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : temp_Pin */
  GPIO_InitStruct.Pin = temp_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(temp_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
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

