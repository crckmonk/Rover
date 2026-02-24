#ifndef MCUTILS_H
#define MCUTILS_H

#include "stm32f4xx_hal.h"
#include "global_config.h"
#include <math.h>
#include "printf.h"
#include <stdarg.h>

void I2C_Scan(I2C_HandleTypeDef *hi2c);

void DEBUG_PRINTF(DEBUG_LEVEL_t level, const char *format, ...);

void print_float(float value, uint16_t decimal_places);

#endif // MCUTILS_H