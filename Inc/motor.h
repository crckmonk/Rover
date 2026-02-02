#ifndef MOTOR_H
#define MOTOR_H

#include "stm32f4xx_hal.h"


#define LEFT_FORWARD TIM_CHANNEL_1
#define LEFT_REVERSE TIM_CHANNEL_2
#define RIGHT_FORWARD TIM_CHANNEL_3
#define RIGHT_REVERSE TIM_CHANNEL_4

typedef enum  {
    STOP = 0,
    DEAD_SLOW = 30,
    SLOW = 60,
    HALF = 80,
    FULL = 100
} speed_e;

typedef enum  {
    FORWARD,
    REVERSE,
    BRAKE
} direction;

typedef enum side_e {
    LEFT,
    RIGHT,
    BOTH
} side_e;


void motor_Init(TIM_HandleTypeDef *htim);

void motor_SetSpeed(TIM_HandleTypeDef *htim, side_e side_e, direction dir, uint8_t speed);

void motor_FullStop(TIM_HandleTypeDef *htim);

void Test_Speed_Settings(TIM_HandleTypeDef *htim, direction dir);


#endif /* MOTOR_H */
