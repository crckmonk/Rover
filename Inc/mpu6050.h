#ifndef MPU6050_
#define MPU6050_

#include "stm32f4xx_hal.h"

#define MPU6050_ADDR 0xD0

typedef struct {
    I2C_HandleTypeDef *i2c;

} MPU6050_Typedef;


uint8_t MPU6050_Init(I2C_HandleTypeDef *i2c);
#endif
