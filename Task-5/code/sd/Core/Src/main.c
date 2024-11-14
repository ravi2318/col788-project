#include "cmsis_os.h"
#include "stm32l0xx_hal.h"
#include "MAX30102.h"
#include "filter.h"
#include "fatfs.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include "main.h"

FATFS FatFs;
FIL fil;
FRESULT fres;
bool log_file_initialized = false;
volatile bool temp_interrupt_flag = false;
volatile bool stop_logging_flag = false;

// CMSIS-RTOS Handles
osThreadId sensorTaskHandle;
osThreadId processingTaskHandle;
osThreadId loggingTaskHandle;
osSemaphoreId dataReadySemaphore;
osMutexId sdCardMutex;
osMessageQId dataQueue;

const float kSamplingFrequency = 400.0f;
const float kLowPassCutoff = 3.0f;
const float kHighPassCutoff = 0.5f;
const float kEdgeThreshold = -100.0f;
#define K_AVERAGING_SAMPLES 100
const float kSpO2_A = 1.5958422;
const float kSpO2_B = -34.6596622;
const float kSpO2_C = 112.6898759;
char rawMessage[64];
// Filter Instances
LowPassFilter low_pass_filter_red, low_pass_filter_ir;
Differentiator differentiator;
MovingAverageFilter averager_bpm, averager_spo2;
MinMaxAvgStatistic stat_red, stat_ir;
HighPassFilter high_pass_filter;
// Heartbeat Detection Variables
uint32_t last_heartbeat = 0;
bool crossed = false;
float last_diff = NAN;
MAX30102_Data sensorData;


UART_HandleTypeDef huart2;
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi2;
RTC_HandleTypeDef hrtc;
MAX30102_Data sensorData;
// Sensor Data Structure
typedef struct {
    float red;
    float ir;
    uint32_t timestamp;
} SensorData_t;

// Message Queue Size
#define QUEUE_SIZE 10

// Function Prototypes
void SensorTask(void const *argument);
void ProcessingTask(void const *argument);
void LoggingTask(void const *argument);
void Error_Handler(void);
void send_uart(char *message);
void log_data_to_sd(char *message);
void init_filters(void);
void reset_filters(void);
void detect_heartbeat(uint32_t current_time, float current_diff, float current_value_ir);


static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);

LowPassFilter low_pass_filter_red;
LowPassFilter low_pass_filter_ir;

void send_uart(char *message) {
    if (huart2.Instance != NULL) {
        HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
    }
}


// SD Card Logging Function
// SD Card Logging Function with Improved Mounting
void log_data_to_sd(char *message) {
    if (osMutexWait(sdCardMutex, osWaitForever) != osOK) {
        send_uart("Failed to acquire SD card mutex.\n\r");
        return;
    }

    // Initialize the log file if not done already
    if (!log_file_initialized) {
        send_uart("Mounting SD card and clearing log file...\n\r");
        fres = f_mount(&FatFs, "", 1);
        if (fres != FR_OK) {
            char errorMsg[64];
            snprintf(errorMsg, sizeof(errorMsg), "SD card mount failed: %d\n\r", fres);
            send_uart(errorMsg);
            osMutexRelease(sdCardMutex);
            return;
        }

        // Delete existing log file to start fresh
        fres = f_unlink("log.txt");
        if (fres != FR_OK && fres != FR_NO_FILE) {
            send_uart("Failed to delete old log file.\n\r");
            osMutexRelease(sdCardMutex);
            return;
        }

        // Create a new log file
        fres = f_open(&fil, "log.txt", FA_CREATE_ALWAYS | FA_WRITE);
        if (fres != FR_OK) {
            send_uart("Failed to create new log file.\n\r");
            osMutexRelease(sdCardMutex);
            return;
        }
        send_uart("Log file initialized.\n\r");
        f_close(&fil);
        log_file_initialized = true;
    }

    // Skip logging if message is empty
    if (message == NULL || strlen(message) == 0) {
        send_uart("Empty message, skipping logging.\n\r");
        osMutexRelease(sdCardMutex);
        return;
    }

    // Open the log file in append mode
    fres = f_open(&fil, "log.txt", FA_OPEN_APPEND | FA_WRITE);
    if (fres != FR_OK) {
        send_uart("Failed to open log file for appending.\n\r");
        osMutexRelease(sdCardMutex);
        return;
    }

    // Write the message to the log file
    UINT bytesWritten;
    fres = f_write(&fil, message, strlen(message), &bytesWritten);
    if (fres != FR_OK || bytesWritten == 0) {
        send_uart("Failed to write to log file.\n\r");
    }

    f_sync(&fil);
    f_close(&fil);
    osMutexRelease(sdCardMutex);
}






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





// Sensor Task
// Sensor Task
void SensorTask(void const *argument) {
    send_uart("SensorTask started\r\n");
    MAX30102_Init(&hi2c1);

    while (1) {
       // MAX30102_Data sensorData;
        MAX30102_ReadFifo(&hi2c1, &sensorData);

        // Print raw values directly over UART
//        char rawMessage[64];
//        snprintf(rawMessage, sizeof(rawMessage), "Raw Red: %d, Raw IR: %d\r\n", (int)sensorData.red, (int)sensorData.ir);
       // send_uart(rawMessage);

        // Send data to processing task
        if (osSemaphoreRelease(dataReadySemaphore) == osOK) {
            osMessagePut(dataQueue, (uint32_t)&sensorData, osWaitForever);
        }

        // No delay; let the RTOS handle task scheduling
        // Remove osDelay(1000 / kSamplingFrequency);
    }
}





// Processing Task
// Processing Task
void ProcessingTask(void const *argument) {
    send_uart("ProcessingTask started\r\n");

    while (1) {
        osSemaphoreWait(dataReadySemaphore, osWaitForever);

        // Check if temperature interrupt flag is set
        if (temp_interrupt_flag) {
            temp_interrupt_flag = false;
            HAL_Delay(30);
            float temperature = MAX30102_ReadTemperature(&hi2c1);
            char temp_message[64];
            snprintf(temp_message, sizeof(temp_message), "Temperature: %.2f°C\r\n", temperature);
            send_uart(temp_message);
            log_data_to_sd(temp_message);
        }

        // Apply low-pass filters
        float redFiltered = LowPassFilter_Process(&low_pass_filter_red, (float)sensorData.red);
        float irFiltered = LowPassFilter_Process(&low_pass_filter_ir, (float)sensorData.ir);

        // Debug filter outputs
        if (redFiltered <= 0.0f || irFiltered <= 0.0f) {
            send_uart("Invalid filtered values. Resetting filters.\r\n");
            reset_filters();
            continue;
        }

        float current_diff = Differentiator_Process(&differentiator, redFiltered);
        MinMaxAvgStatistic_Process(&stat_red, redFiltered);
        MinMaxAvgStatistic_Process(&stat_ir, irFiltered);

        // Heartbeat Detection
        detect_heartbeat(HAL_GetTick(), current_diff, irFiltered);

        // BPM Calculation
        uint32_t current_time = HAL_GetTick();
        uint32_t time_diff = current_time - last_heartbeat;
        uint32_t bpm = (time_diff > 500 && time_diff < 2000) ? 60000 / time_diff : 0;

        // Validate BPM
        if (bpm < 50 || bpm > 150) {
            bpm = 0;  // Discard unrealistic BPM values
        }

        // SpO2 Calculation
        float r = (MinMaxAvgStatistic_Maximum(&stat_red) - MinMaxAvgStatistic_Minimum(&stat_red)) /
                  MinMaxAvgStatistic_Average(&stat_red);
        r /= (MinMaxAvgStatistic_Maximum(&stat_ir) - MinMaxAvgStatistic_Minimum(&stat_ir)) /
             MinMaxAvgStatistic_Average(&stat_ir);

        float spo2 = kSpO2_A * r * r + kSpO2_B * r + kSpO2_C;

        // Validate SpO2
        if (isnan(spo2) || spo2 < 80.0f || spo2 > 100.0f) {
            spo2 = 0.0f;
        }

        // Log Valid Data
        if (bpm != 0 && spo2 != 0.0f) {
            char message[128];
            snprintf(message, sizeof(message), "BPM: %lu, SpO2: %.2f%%\r\n", bpm, spo2);
            send_uart(message);
            log_data_to_sd(message);
        } else {
            send_uart("Skipping invalid BPM or SpO2 log entry.\r\n");
        }
    }
}


char tbuff[100]={0};

void detect_heartbeat(uint32_t current_time, float current_diff, float current_value_ir) {
    if (!isnan(current_diff) && !isnan(last_diff)) {
    	if (last_diff > 0 && current_diff < 0) {
            crossed = true;
        }

        if (crossed && current_diff < kEdgeThreshold) {
            if (last_heartbeat != 0 && (current_time - last_heartbeat) > 500) {
                uint32_t bpm = 60000 / (current_time - last_heartbeat);
                sprintf(tbuff,"\n cal bpm=%ld\n",bpm);
                                 send_uart(tbuff);
                if (bpm > 50 && bpm < 150) {
                    float r = (MinMaxAvgStatistic_Maximum(&stat_red) - MinMaxAvgStatistic_Minimum(&stat_red)) /
                              MinMaxAvgStatistic_Average(&stat_red);
                    r /= (MinMaxAvgStatistic_Maximum(&stat_ir) - MinMaxAvgStatistic_Minimum(&stat_ir)) /
                         MinMaxAvgStatistic_Average(&stat_ir);

                    float spo2 = kSpO2_A * r * r + kSpO2_B * r + kSpO2_C;
                    spo2 = fmaxf(0, fminf(100, spo2));

                    char message[64];
                    int average_bpm = MovingAverageFilter_Process(&averager_bpm, bpm);
                    float average_spo2 = MovingAverageFilter_Process(&averager_spo2, spo2);

                    snprintf(message, sizeof(message), "Heart Rate: %d bpm, SpO2: %.2f%%\n\r", average_bpm, average_spo2);
                    send_uart(message);
                }
            }
            last_heartbeat = current_time;
            crossed = false;
            reset_filters();
        }
    }
    last_diff = current_diff;
}


// Logging Task
// Logging Task
void LoggingTask(void const *argument) {
    send_uart("LoggingTask started\r\n");

    while (1) {
        osEvent event = osMessageGet(dataQueue, osWaitForever);
        if (event.status == osEventMessage) {
            char *logMessage = (char *)event.value.p;
            log_data_to_sd(logMessage);
        }
    }
}

void initiate_temperature_read(void) {
    send_uart("Initiating temperature read.\n\r");

    // Call the function without assignment
    MAX30102_WriteRegister(&hi2c1, MAX30102_REG_TEMP_CONFIG, 0x01);

    // Delay to allow temperature conversion to complete
    HAL_Delay(30);

    // Set the interrupt flag after initiating the read
    temp_interrupt_flag = true;
}




// Main Function
int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USART2_UART_Init();
    MX_SPI2_Init();
   // MX_RTC_Init();
    MX_FATFS_Init();
    init_filters();




    // Initialize CMSIS-RTOS components
    osSemaphoreDef(dataReadySemaphore);
    dataReadySemaphore = osSemaphoreCreate(osSemaphore(dataReadySemaphore), 1);
    if (dataReadySemaphore == NULL) {
        send_uart("Failed to create dataReadySemaphore\r\n");
    } else {
        send_uart("dataReadySemaphore created successfully\r\n");
    }
    osMutexDef(sdCardMutex);
    sdCardMutex = osMutexCreate(osMutex(sdCardMutex));

    osMessageQDef(dataQueue, QUEUE_SIZE, uint32_t);
    dataQueue = osMessageCreate(osMessageQ(dataQueue), NULL);

    // Create tasks
    osThreadDef(SensorTask, SensorTask, osPriorityNormal, 0, 256);
    sensorTaskHandle = osThreadCreate(osThread(SensorTask), NULL);

    osThreadDef(ProcessingTask, ProcessingTask, osPriorityAboveNormal, 0, 512);
    processingTaskHandle = osThreadCreate(osThread(ProcessingTask), NULL);
    if (processingTaskHandle == NULL) {
        send_uart("Failed to create ProcessingTask\r\n");
    } else {
        send_uart("ProcessingTask created successfully\r\n");
    }
    osThreadDef(LoggingTask, LoggingTask, osPriorityLow, 0, 512);
    loggingTaskHandle = osThreadCreate(osThread(LoggingTask), NULL);

    // Start the scheduler
    osKernelStart();

    // Infinite loop (should never reach here)
    while (1) {}
}


//
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == temp_Pin) {
        send_uart("Temperature interrupt triggered.\n\r");
        temp_interrupt_flag = true;
        osSemaphoreRelease(dataReadySemaphore);
    } else if (GPIO_Pin == GPIO_PIN_13) {
        stop_logging_flag = true;
        send_uart("Logging and calculations stopped by button press.\n\r");
    }
}





// Filter Initialization
//void init_filters(void) {
//    HighPassFilter_InitWithCutoff(&high_pass_filter, kHighPassCutoff, kSamplingFrequency);
//    LowPassFilter_InitWithCutoff(&low_pass_filter_red, kLowPassCutoff, kSamplingFrequency);
//    LowPassFilter_InitWithCutoff(&low_pass_filter_ir, kLowPassCutoff, kSamplingFrequency);
//    Differentiator_Init(&differentiator, kSamplingFrequency);
//
//    static float average_buffer_bpm[K_AVERAGING_SAMPLES];
//    static float average_buffer_spo2[K_AVERAGING_SAMPLES];
//    MovingAverageFilter_Init(&averager_bpm, average_buffer_bpm, K_AVERAGING_SAMPLES);
//    MovingAverageFilter_Init(&averager_spo2, average_buffer_spo2, K_AVERAGING_SAMPLES);
//
//    MinMaxAvgStatistic_Init(&stat_red);
//    MinMaxAvgStatistic_Init(&stat_ir);
//}
//
//
//// Reset Filters
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
//static void MX_RTC_Init(void)
//{
//
//  /* USER CODE BEGIN RTC_Init 0 */
//
//  /* USER CODE END RTC_Init 0 */
//
//  RTC_TimeTypeDef sTime = {0};
//  RTC_DateTypeDef sDate = {0};
//
//  /* USER CODE BEGIN RTC_Init 1 */
//
//  /* USER CODE END RTC_Init 1 */
//
//  /** Initialize RTC Only
//  */
//  hrtc.Instance = RTC;
//  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
//  hrtc.Init.AsynchPrediv = 127;
//  hrtc.Init.SynchPrediv = 255;
//  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
//  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
//  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
//  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
//  if (HAL_RTC_Init(&hrtc) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /* USER CODE BEGIN Check_RTC_BKUP */
//
//  /* USER CODE END Check_RTC_BKUP */
//
//  /** Initialize RTC and set the Time and Date
//  */
//  sTime.Hours = 0;
//  sTime.Minutes = 0;
//  sTime.Seconds = 0;
//  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
//  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
//  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
//  sDate.Month = RTC_MONTH_JANUARY;
//  sDate.Date = 1;
//  sDate.Year = 0;
//
//  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN RTC_Init 2 */
//
//  /* USER CODE END RTC_Init 2 */
//
//}

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
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
    __disable_irq();
    while (1) {
        send_uart("System Error\r\n");
        HAL_Delay(1000);
    }
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
