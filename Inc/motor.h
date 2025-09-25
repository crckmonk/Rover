#ifndef MOTOR_H
#define MOTOR_H

#include "stm32f4xx_hal.h"


#define PORT_FORWARD TIM_CHANNEL_1
#define PORT_REVERSE TIM_CHANNEL_2
#define STB_FORWARD TIM_CHANNEL_3
#define STB_REVERSE TIM_CHANNEL_4

typedef enum  {
    STOP = 0,
    DEAD_SLOW = 30,
    SLOW = 60,
    HALF = 80,
    FULL = 100
} speed_mode;

typedef enum  {
    FORWARD,
    REVERSE,
    BRAKE
} direction;

typedef enum side {
    PORT,
    STARBOARD,
    BOTH
} side;


void Motor_Init(TIM_HandleTypeDef *htim);

void Set_Motor_Speed(TIM_HandleTypeDef *htim, side side, direction dir, speed_mode speed);

void Motor_full_stop(TIM_HandleTypeDef *htim);

void Test_Speed_Settings(TIM_HandleTypeDef *htim, direction dir);

void Test_Rotation(TIM_HandleTypeDef *htim, speed_mode speed);

#endif /* MOTOR_H */
