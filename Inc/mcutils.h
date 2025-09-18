#ifndef MCUTILS_H
#define MCUTILS_H

#include "stm32f4xx_hal.h"

void I2C_Scan(I2C_HandleTypeDef *hi2c);

void print_float(float value, uint16_t decimal_places);

#endif // MCUTILS_H