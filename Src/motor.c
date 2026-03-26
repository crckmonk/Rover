#include "motor.h"

static TIM_HandleTypeDef *motor_timer;

void motor_Init(TIM_HandleTypeDef *htim)
{
  motor_timer = htim;
    HAL_TIM_PWM_Start(motor_timer, LEFT_FORWARD);
    HAL_TIM_PWM_Start(motor_timer, LEFT_REVERSE);
    HAL_TIM_PWM_Start(motor_timer, RIGHT_FORWARD);
    HAL_TIM_PWM_Start(motor_timer, RIGHT_REVERSE);
  __HAL_TIM_SET_COMPARE(motor_timer, LEFT_FORWARD, 0); 
  __HAL_TIM_SET_COMPARE(motor_timer, LEFT_REVERSE, 0); 
  __HAL_TIM_SET_COMPARE(motor_timer, RIGHT_FORWARD, 0); 
  __HAL_TIM_SET_COMPARE(motor_timer, RIGHT_REVERSE, 0);  
}

void motor_SetSpeed(side_e side, motor_Direction_t dir, uint8_t speed)
{
  if (side == LEFT || side == BOTH){
    switch(dir){
      case FORWARD:
        __HAL_TIM_SET_COMPARE(motor_timer, LEFT_FORWARD, speed);
        break;
      case REVERSE: 
        __HAL_TIM_SET_COMPARE(motor_timer, LEFT_REVERSE, speed);
        break;
      default:
        __HAL_TIM_SET_COMPARE(motor_timer, LEFT_FORWARD, STOP);
        __HAL_TIM_SET_COMPARE(motor_timer, LEFT_REVERSE, STOP);
        break;
    }
  }
  if (side == RIGHT || side == BOTH){
    switch(dir){
      case FORWARD:
        __HAL_TIM_SET_COMPARE(motor_timer, RIGHT_FORWARD, speed);
        break;
      case REVERSE: 
        __HAL_TIM_SET_COMPARE(motor_timer, RIGHT_REVERSE, speed);
        break;
      default:
        __HAL_TIM_SET_COMPARE(motor_timer, RIGHT_FORWARD, STOP);
        __HAL_TIM_SET_COMPARE(motor_timer, RIGHT_REVERSE, STOP);
        break;
    }
  }
}

void motor_FullStop(){
  motor_SetSpeed(BOTH, HALT, STOP);
}




void Test_Speed_Settings(motor_Direction_t dir){
    motor_SetSpeed(BOTH, dir , DEAD_SLOW);
  HAL_Delay(1000);
  motor_SetSpeed(BOTH, dir, SLOW);
  HAL_Delay(1000);
  motor_SetSpeed(BOTH, dir, HALF);
  HAL_Delay(1000);
  motor_SetSpeed(BOTH, dir, FULL);
  HAL_Delay(1000);
  motor_SetSpeed(BOTH, dir, STOP);
  HAL_Delay(1000);
}