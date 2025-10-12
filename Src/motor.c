#include "motor.h"

void Motor_Init(TIM_HandleTypeDef *htim)
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

void Set_Motor_Speed(TIM_HandleTypeDef *htim,  side_e side, direction dir, uint8_t speed)
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

void Motor_full_stop(TIM_HandleTypeDef *htim){
  Set_Motor_Speed(htim, BOTH, BRAKE, STOP);
}

void Test_Speed_Settings(TIM_HandleTypeDef *htim, direction dir){
    Set_Motor_Speed(htim, BOTH, dir , DEAD_SLOW);
  HAL_Delay(1000);
  Set_Motor_Speed(htim, BOTH, dir, SLOW);
  HAL_Delay(1000);
  Set_Motor_Speed(htim, BOTH, dir, HALF);
  HAL_Delay(1000);
  Set_Motor_Speed(htim, BOTH, dir, FULL);
  HAL_Delay(1000);
  Set_Motor_Speed(htim, BOTH, dir, STOP);
  HAL_Delay(1000);
}