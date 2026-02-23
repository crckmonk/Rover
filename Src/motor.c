#include "motor.h"

void motor_Init(TIM_HandleTypeDef *htim)
{
    HAL_TIM_PWM_Start(htim, LEFT_FORWARD);
    HAL_TIM_PWM_Start(htim, LEFT_REVERSE);
    HAL_TIM_PWM_Start(htim, RIGHT_FORWARD);
    HAL_TIM_PWM_Start(htim, RIGHT_REVERSE);
  __HAL_TIM_SET_COMPARE(htim, LEFT_FORWARD, 0); 
  __HAL_TIM_SET_COMPARE(htim, LEFT_REVERSE, 0); 
  __HAL_TIM_SET_COMPARE(htim, RIGHT_FORWARD, 0); 
  __HAL_TIM_SET_COMPARE(htim, RIGHT_REVERSE, 0);  
}

void motor_SetSpeed(TIM_HandleTypeDef *htim,  side_e side, motor_Direction_t dir, uint8_t speed)
{
  if (side == LEFT || side == BOTH){
    switch(dir){
      case FORWARD:
        __HAL_TIM_SET_COMPARE(htim, LEFT_FORWARD, speed);
        break;
      case REVERSE: 
        __HAL_TIM_SET_COMPARE(htim, LEFT_REVERSE, speed);
        break;
      default:
        __HAL_TIM_SET_COMPARE(htim, LEFT_FORWARD, STOP);
        __HAL_TIM_SET_COMPARE(htim, LEFT_REVERSE, STOP);
        break;
    }
  }
  if (side == RIGHT || side == BOTH){
    switch(dir){
      case FORWARD:
        __HAL_TIM_SET_COMPARE(htim, RIGHT_FORWARD, speed);
        break;
      case REVERSE: 
        __HAL_TIM_SET_COMPARE(htim, RIGHT_REVERSE, speed);
        break;
      default:
        __HAL_TIM_SET_COMPARE(htim, RIGHT_FORWARD, STOP);
        __HAL_TIM_SET_COMPARE(htim, RIGHT_REVERSE, STOP);
        break;
    }
  }
}

void motor_FullStop(TIM_HandleTypeDef *htim){
  motor_SetSpeed(htim, BOTH, BRAKE, STOP);
}

void Test_Speed_Settings(TIM_HandleTypeDef *htim, motor_Direction_t dir){
    motor_SetSpeed(htim, BOTH, dir , DEAD_SLOW);
  HAL_Delay(1000);
  motor_SetSpeed(htim, BOTH, dir, SLOW);
  HAL_Delay(1000);
  motor_SetSpeed(htim, BOTH, dir, HALF);
  HAL_Delay(1000);
  motor_SetSpeed(htim, BOTH, dir, FULL);
  HAL_Delay(1000);
  motor_SetSpeed(htim, BOTH, dir, STOP);
  HAL_Delay(1000);
}