#ifndef MPU6500_
#define MPU6500_

#include "stm32f4xx_hal.h"

#define MPU6500_ADDR 0xD0
#define MPU6500_ID 0x70


typedef struct {
    I2C_HandleTypeDef *i2c;

} MPU6500_Typedef;


#define MPU6500_REG_WHOAMI 0x75
#define MPU6500_REG_PWR_1 0x6B
#define MPU6500_REG_PWR_2 0x6C

#define TEMP_OUT_H 0x41

#define MPU6500_PWR1_RST 1



uint8_t MPU6500_Init(I2C_HandleTypeDef *i2c);
void MPU650_Read_Gyro(I2C_HandleTypeDef *i2c, uint16_t *gyroXYZOut);
HAL_StatusTypeDef MPU6500_ReadTemp(I2C_HandleTypeDef *i2c, int16_t *temp);
#endif
