#include "motor.h"
#include "mcutils.h"

static TIM_HandleTypeDef *motor_timer;

void Motor_Init(TIM_HandleTypeDef *htim)
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

void Motor_SetSpeed(side_e side, motor_Direction_t dir, uint8_t speed)
{
  if(speed > 100){
    speed = 100;
  }
  if (side == LEFT || side == BOTH){
    switch(dir){
      case FORWARD:
        __HAL_TIM_SET_COMPARE(motor_timer, LEFT_REVERSE, STOP);  
        __HAL_TIM_SET_COMPARE(motor_timer, LEFT_FORWARD, speed);
        break;
      case REVERSE: 
        __HAL_TIM_SET_COMPARE(motor_timer, LEFT_FORWARD, STOP);  
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
        __HAL_TIM_SET_COMPARE(motor_timer, RIGHT_REVERSE, STOP); 
        __HAL_TIM_SET_COMPARE(motor_timer, RIGHT_FORWARD, speed);
        break;
      case REVERSE: 
        __HAL_TIM_SET_COMPARE(motor_timer, RIGHT_FORWARD, STOP);  
        __HAL_TIM_SET_COMPARE(motor_timer, RIGHT_REVERSE, speed);
        break;
      default:
        __HAL_TIM_SET_COMPARE(motor_timer, RIGHT_FORWARD, STOP);
        __HAL_TIM_SET_COMPARE(motor_timer, RIGHT_REVERSE, STOP);
        break;
    }
  }
}

void Motor_FullStop(){
  Motor_SetSpeed(BOTH, HALT, STOP);
}




void Test_Speed_Settings(motor_Direction_t dir){
    Motor_SetSpeed(BOTH, dir , DEAD_SLOW);
  HAL_Delay(1000);
  Motor_SetSpeed(BOTH, dir, SLOW);
  HAL_Delay(1000);
  Motor_SetSpeed(BOTH, dir, HALF);
  HAL_Delay(1000);
  Motor_SetSpeed(BOTH, dir, FULL);
  HAL_Delay(1000);
  Motor_SetSpeed(BOTH, dir, STOP);
  HAL_Delay(1000);
}

void Motor_TestChannels(void){
        DEBUG_PRINTF(DBG_INFO, "Testing LEFT_FORWARD\r\n");
    for (uint8_t i = 0; i< 100;i++){
        __HAL_TIM_SET_COMPARE(motor_timer, LEFT_FORWARD, i);
        HAL_Delay(50);
    }
    __HAL_TIM_SET_COMPARE(motor_timer, LEFT_FORWARD, 0);
    HAL_Delay(200);

    DEBUG_PRINTF(DBG_INFO, "Testing LEFT_REVERSE\r\n");
    for (uint8_t i = 0; i< 100;i++){
        
        __HAL_TIM_SET_COMPARE(motor_timer, LEFT_REVERSE, i);
        HAL_Delay(50);
    }
    __HAL_TIM_SET_COMPARE(motor_timer, LEFT_REVERSE, 0);
    HAL_Delay(200);

    DEBUG_PRINTF(DBG_INFO, "Testing RIGHT_FORWARD\r\n");
    for (uint8_t i = 0; i< 100;i++){
        
        __HAL_TIM_SET_COMPARE(motor_timer, RIGHT_FORWARD, i);
        HAL_Delay(50);
    }
    __HAL_TIM_SET_COMPARE(motor_timer, RIGHT_FORWARD, 0);
    HAL_Delay(200);

    DEBUG_PRINTF(DBG_INFO, "Testing RIGHT_REVERSE\r\n");
    for (uint8_t i = 0; i< 100;i++){
        
        __HAL_TIM_SET_COMPARE(motor_timer, RIGHT_REVERSE, i);
        HAL_Delay(50);
    }
    Motor_FullStop();
}


void Motor_TestTurning(void){
        DEBUG_PRINTF(DBG_INFO, "Testing right turn\r\n");
    for (uint8_t i = 0; i< 100;i++){
        __HAL_TIM_SET_COMPARE(motor_timer, LEFT_FORWARD, i);
        __HAL_TIM_SET_COMPARE(motor_timer, RIGHT_REVERSE, i);
        HAL_Delay(100);
    }

    Motor_FullStop();

    HAL_Delay(200);

    DEBUG_PRINTF(DBG_INFO, "Testing LEFT_REVERSE\r\n");
    for (uint8_t i = 0; i< 100;i++){
        
        __HAL_TIM_SET_COMPARE(motor_timer, LEFT_REVERSE, i);
        __HAL_TIM_SET_COMPARE(motor_timer, RIGHT_FORWARD, i);
        HAL_Delay(100);
    }
    Motor_FullStop();
}