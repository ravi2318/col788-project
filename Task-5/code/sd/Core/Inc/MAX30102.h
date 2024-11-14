#ifndef MAX30102_H
#define MAX30102_H

#include "stm32l0xx_hal.h"  // Adjust according to your STM32 family

/* MAX30102 I2C address */
#define MAX30102_ADDRESS  0x57  // 7-bit address, adjust if necessary

/* MAX30102 Register addresses */
#define MAX30102_REG_INTR_STATUS_1    0x00
#define MAX30102_REG_INTR_STATUS_2    0x01
#define MAX30102_REG_INTR_ENABLE_1    0x02
#define MAX30102_REG_INTR_ENABLE_2    0x03
#define MAX30102_REG_FIFO_WR_PTR      0x04
#define MAX30102_REG_OVF_COUNTER      0x05
#define MAX30102_REG_FIFO_RD_PTR      0x06
#define MAX30102_REG_FIFO_DATA        0x07
#define MAX30102_REG_MODE_CONFIG      0x09
#define MAX30102_REG_SPO2_CONFIG      0x0A
#define MAX30102_REG_LED1_PA          0x0C
#define MAX30102_REG_LED2_PA          0x0D
#define MAX30102_REG_PILOT_PA         0x10
#define MAX30102_REG_MULTI_LED_CTRL1  0x11
#define MAX30102_REG_MULTI_LED_CTRL2  0x12
#define MAX30102_REG_TEMP_INT         0x1F
#define MAX30102_REG_TEMP_FRAC        0x20
#define MAX30102_REG_TEMP_CONFIG      0x21
#define MAX30102_REG_PROX_INT_THRESH  0x30
#define MAX30102_REG_REV_ID           0xFE
#define MAX30102_REG_PART_ID          0xFF
//#define MAX30102_REG_FIFO_OVF_CNT 0x02

/* MAX30102 Mode settings */
#define MAX30102_MODE_HR_ONLY         0x02
#define MAX30102_MODE_SPO2            0x03
#define MAX30102_MODE_MULTI_LED       0x07

/* Pulse width for SpO2 ADC */
#define MAX30102_SPO2_PW_69           0x00
#define MAX30102_SPO2_PW_118          0x01
#define MAX30102_SPO2_PW_215          0x02
#define MAX30102_SPO2_PW_411          0x03

/* ADC range */
#define MAX30102_SPO2_ADC_RGE_2048    0x00
#define MAX30102_SPO2_ADC_RGE_4096    0x01
#define MAX30102_SPO2_ADC_RGE_8192    0x02
#define MAX30102_SPO2_ADC_RGE_16384   0x03

/* Sample rate */
#define MAX30102_SPO2_SR_50           0x00
#define MAX30102_SPO2_SR_100          0x01
#define MAX30102_SPO2_SR_200          0x02
#define MAX30102_SPO2_SR_400          0x03
#define MAX30102_SPO2_SR_800          0x04
#define MAX30102_SPO2_SR_1000         0x05
#define MAX30102_SPO2_SR_1600         0x06
#define MAX30102_SPO2_SR_3200         0x07

/* LED Pulse Amplitude */


#define   MAX30100_LED_CURRENT_0MA              0x00
#define   MAX30100_LED_CURRENT_4_4MA            0x01
#define   MAX30100_LED_CURRENT_7_6MA            0x02
#define   MAX30100_LED_CURRENT_11MA             0x03
#define   MAX30100_LED_CURRENT_14_2MA           0x04
#define   MAX30100_LED_CURRENT_17_4MA           0x05
#define   MAX30100_LED_CURRENT_20_8MA           0x06
#define   MAX30100_LED_CURRENT_24MA             0x7F
#define   MAX30100_LED_CURRENT_27_1MA           0x08
#define   MAX30100_LED_CURRENT_30_6MA           0x09
#define   MAX30100_LED_CURRENT_33_8MA           0x0A
#define   MAX30100_LED_CURRENT_37MA             0x0B
#define   MAX30100_LED_CURRENT_40_2MA           0x0C
#define   MAX30100_LED_CURRENT_43_6MA           0x0D
#define   MAX30100_LED_CURRENT_46_8MA           0x0E
#define   MAX30100_LED_CURRENT_50MA             0xFF

/* Structure to hold sensor data */
typedef struct {
    uint16_t red;
    uint16_t ir;
} MAX30102_Data;

/* Function prototypes */
void MAX30102_Init(I2C_HandleTypeDef *hi2c);
void MAX30102_ReadFifo(I2C_HandleTypeDef *hi2c, MAX30102_Data *data);
void MAX30102_Reset(I2C_HandleTypeDef *hi2c);
void MAX30102_SetMode(I2C_HandleTypeDef *hi2c, uint8_t mode);
void MAX30102_SetSpO2Config(I2C_HandleTypeDef *hi2c, uint8_t led_pw, uint8_t adc_rge, uint8_t sr);
void MAX30102_SetLedPulseAmplitude(I2C_HandleTypeDef *hi2c, uint8_t led1_pa, uint8_t led2_pa);
float MAX30102_ReadTemperature(I2C_HandleTypeDef *hi2c);

/* Utility function */
uint8_t MAX30102_ReadRegister(I2C_HandleTypeDef *hi2c, uint8_t reg);
void MAX30102_WriteRegister(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t value);


#endif // MAX30102_H
