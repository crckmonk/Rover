#include "assert_handler.h"
#include "motor.h"

extern TIM_HandleTypeDef htim2;

void assert_handler(void){
    // You can add logging or other actions here
    Motor_full_stop(&htim2);
    while(1){
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        HAL_Delay(350);
    }
}