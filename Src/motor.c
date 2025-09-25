#include "motor.h"

void Motor_Init(TIM_HandleTypeDef *htim)
{
    HAL_TIM_PWM_Start(htim, PORT_FORWARD);
    HAL_TIM_PWM_Start(htim, PORT_REVERSE);
    HAL_TIM_PWM_Start(htim, STB_FORWARD);
    HAL_TIM_PWM_Start(htim, STB_REVERSE);
  __HAL_TIM_SET_COMPARE(htim, PORT_FORWARD, 0); 
  __HAL_TIM_SET_COMPARE(htim, PORT_REVERSE, 0); 
  __HAL_TIM_SET_COMPARE(htim, STB_FORWARD, 0); 
  __HAL_TIM_SET_COMPARE(htim, STB_REVERSE, 0);  
}

void Set_Motor_Speed(TIM_HandleTypeDef *htim,  side side, direction dir, speed_mode speed)
{
  if (side == PORT || side == BOTH){
    switch(dir){
      case FORWARD:
        __HAL_TIM_SET_COMPARE(htim, PORT_FORWARD, speed);
        break;
      case REVERSE: 
        __HAL_TIM_SET_COMPARE(htim, PORT_REVERSE, speed);
        break;
      default:
        __HAL_TIM_SET_COMPARE(htim, PORT_FORWARD, STOP);
        __HAL_TIM_SET_COMPARE(htim, PORT_REVERSE, STOP);
        break;
    }
  }
  if (side == STARBOARD || side == BOTH){
    switch(dir){
      case FORWARD:
        __HAL_TIM_SET_COMPARE(htim, STB_FORWARD, speed);
        break;
      case REVERSE: 
        __HAL_TIM_SET_COMPARE(htim, STB_REVERSE, speed);
        break;
      default:
        __HAL_TIM_SET_COMPARE(htim, STB_FORWARD, STOP);
        __HAL_TIM_SET_COMPARE(htim, STB_REVERSE, STOP);
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

void Test_Rotation(TIM_HandleTypeDef *htim, speed_mode speed){
  Set_Motor_Speed(htim, STARBOARD, FORWARD, speed);
  Set_Motor_Speed(htim, PORT, REVERSE, speed);
}