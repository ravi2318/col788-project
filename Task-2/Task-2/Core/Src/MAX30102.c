#include "MAX30102.h"
#include "stm32l0xx_hal.h"  // Include your specific STM32 HAL

/* Private function prototypes */
void MAX30102_WriteRegister(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t value);
uint8_t MAX30102_ReadRegister(I2C_HandleTypeDef *hi2c, uint8_t reg);

/**
 * @brief  Initializes the MAX30102 sensor
 * @param  hi2c: I2C handle
 * @retval None
 */
void MAX30102_Init(I2C_HandleTypeDef *hi2c)
{
    // Reset the MAX30102
    MAX30102_Reset(hi2c);

    // Wait for the reset to complete
    HAL_Delay(100);

    // Set the mode to SpO2 mode
    MAX30102_SetMode(hi2c, MAX30102_MODE_SPO2);

    // Configure SpO2 sensor parameters
    MAX30102_SetSpO2Config(hi2c, MAX30102_SPO2_PW_411, MAX30102_SPO2_ADC_RGE_2048, MAX30102_SPO2_SR_400);
    MAX30102_WriteRegister(hi2c, MAX30102_REG_INTR_ENABLE_2, MAX30102_REG_INTR_ENABLE_2);





    // Set the LED pulse amplitudes (adjust according to your requirements)
    MAX30102_SetLedPulseAmplitude(hi2c, MAX30100_LED_CURRENT_11MA , MAX30100_LED_CURRENT_11MA );


    // Additional sensor configurations can go here
}

/**
 * @brief  Reads the FIFO data (Red and IR)
 * @param  hi2c: I2C handle
 * @param  data: pointer to MAX30102_Data structure to store the results
 * @retval None
 */
void MAX30102_ReadFifo(I2C_HandleTypeDef *hi2c, MAX30102_Data *data)
{
    uint8_t fifoData[6];

    // Read 6 bytes from the FIFO_DATA register
    HAL_I2C_Mem_Read(hi2c, MAX30102_ADDRESS << 1, MAX30102_REG_FIFO_DATA, I2C_MEMADD_SIZE_8BIT, fifoData, 6, HAL_MAX_DELAY);

    // Combine the bytes to form 16-bit values for Red and IR data
    data->red = (fifoData[0] << 16) | (fifoData[1] << 8) | fifoData[2];
    data->ir = (fifoData[3] << 16) | (fifoData[4] << 8) | fifoData[5];
}

/**
 * @brief  Resets the MAX30102 sensor
 * @param  hi2c: I2C handle
 * @retval None
 */
void MAX30102_Reset(I2C_HandleTypeDef *hi2c)
{
    // Write 0x40 to the MODE_CONFIG register to reset the device
    MAX30102_WriteRegister(hi2c, MAX30102_REG_MODE_CONFIG, 0x40);
}

/**
 * @brief  Sets the operating mode of the MAX30102 sensor
 * @param  hi2c: I2C handle
 * @param  mode: Desired mode (e.g., MAX30102_MODE_SPO2, MAX30102_MODE_HR_ONLY)
 * @retval None
 */
void MAX30102_SetMode(I2C_HandleTypeDef *hi2c, uint8_t mode)
{
    MAX30102_WriteRegister(hi2c, MAX30102_REG_MODE_CONFIG, mode);
}

/**
 * @brief  Configures the SpO2 sensor parameters
 * @param  hi2c: I2C handle
 * @param  led_pw: Pulse width (e.g., MAX30102_SPO2_PW_118)
 * @param  adc_rge: ADC range (e.g., MAX30102_SPO2_ADC_RGE_4096)
 * @param  sr: Sample rate (e.g., MAX30102_SPO2_SR_100)
 * @retval None
 */
void MAX30102_SetSpO2Config(I2C_HandleTypeDef *hi2c, uint8_t led_pw, uint8_t adc_rge, uint8_t sr)
{
    uint8_t config = (adc_rge << 5) | (sr << 2) | led_pw;
    MAX30102_WriteRegister(hi2c, MAX30102_REG_SPO2_CONFIG, config);
}

/**
 * @brief  Sets the pulse amplitude for the LEDs
 * @param  hi2c: I2C handle
 * @param  led1_pa: Pulse amplitude for LED1 (Red)
 * @param  led2_pa: Pulse amplitude for LED2 (IR)
 * @retval None
 */
void MAX30102_SetLedPulseAmplitude(I2C_HandleTypeDef *hi2c, uint8_t led1_pa, uint8_t led2_pa)
{
    MAX30102_WriteRegister(hi2c, MAX30102_REG_LED1_PA, led1_pa);
    MAX30102_WriteRegister(hi2c, MAX30102_REG_LED2_PA, led2_pa);
}

/**
 * @brief  Reads the die temperature from the MAX30102 sensor
 * @param  hi2c: I2C handle
 * @retval Temperature in degrees Celsius
 */
//float MAX30102_ReadTemperature(I2C_HandleTypeDef *hi2c)
//{
//    // Enable temperature conversion
//    MAX30102_WriteRegister(hi2c, MAX30102_REG_TEMP_CONFIG, 0x01);
//
//    // Wait for the temperature conversion to complete
//    HAL_Delay(30);
//
//    // Read the integer part of the temperature
//    uint8_t temp_int = MAX30102_ReadRegister(hi2c, MAX30102_REG_TEMP_INT);
//
//    // Read the fractional part of the temperature
//    uint8_t temp_frac = MAX30102_ReadRegister(hi2c, MAX30102_REG_TEMP_FRAC);
//
//    // Combine the integer and fractional parts
//    float temperature = temp_int + (temp_frac * 0.0625);
//
//    return temperature;
//}

/**
 * @brief  Writes a value to a register in the MAX30102 sensor
 * @param  hi2c: I2C handle
 * @param  reg: Register address
 * @param  value: Value to write
 * @retval None
 */
void MAX30102_WriteRegister(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t value)
{
    HAL_I2C_Mem_Write(hi2c, MAX30102_ADDRESS << 1, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, HAL_MAX_DELAY);
}

/**
 * @brief  Reads a value from a register in the MAX30102 sensor
 * @param  hi2c: I2C handle
 * @param  reg: Register address
 * @retval Register value
 */
uint8_t MAX30102_ReadRegister(I2C_HandleTypeDef *hi2c, uint8_t reg)
{
    uint8_t value = 0;
    HAL_I2C_Mem_Read(hi2c, MAX30102_ADDRESS << 1, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, HAL_MAX_DELAY);
    return value;
}

// Simple SpO2 calculation
float calculate_spo2(uint16_t red, uint16_t ir) {
    if (ir == 0) {
        return 0; // Avoid division by zero
    }
    float ratio = (float)red / ir;

    // Simple empirical formula for SpO2 (for demonstration purposes)
    // In a real application, use a more accurate formula or calibration data
    float spo2 = 110 - (25 * ratio);
    if (spo2 < 0) {
        spo2 = 0;
    }
    if (spo2 > 100) {
        spo2 = 100;
    }
    return spo2;
}

float MAX30102_ReadTemperature(I2C_HandleTypeDef *hi2c)
{
    // Enable temperature conversion
    MAX30102_WriteRegister(hi2c, MAX30102_REG_TEMP_CONFIG, 0x01);

    // Wait for the temperature conversion to complete
    HAL_Delay(30);

    // Read the integer part of the temperature
    uint8_t temp_int = MAX30102_ReadRegister(hi2c, MAX30102_REG_TEMP_INT);

    // Read the fractional part of the temperature
    uint8_t temp_frac = MAX30102_ReadRegister(hi2c, MAX30102_REG_TEMP_FRAC);

    // Combine the integer and fractional parts
    float temperature = temp_int + (temp_frac * 0.0625);

    return temperature;
}



