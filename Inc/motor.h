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
    HALT = 0,
    FORWARD,
    REVERSE
} motor_Direction_t;

typedef enum side_e {
    LEFT,
    RIGHT,
    BOTH
} side_e;


void Motor_Init(TIM_HandleTypeDef *htim);

void Motor_SetSpeed(side_e side_e, motor_Direction_t dir, uint8_t speed);

void Motor_FullStop();

void Test_Speed_Settings(motor_Direction_t dir);

void Motor_TestChannels(void);

void Motor_TestTurning(void);

#endif /* MOTOR_H */
