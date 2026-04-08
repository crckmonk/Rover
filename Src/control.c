#include "control.h"


static Rover_t RoverState = {
    .SYSTEM_ID = 1,
    .mode = MAV_MODE_MANUAL_DISARMED,
    .status = MAV_STATE_STANDBY,
    .left_motor = 0,
    .right_motor = 0,
    .vbat = 0
};


void Rover_ResetState(){
    RoverState.mode = MAV_MODE_MANUAL_DISARMED;
    RoverState.left_motor = 0;
    RoverState.right_motor = 0;
    RoverState.status = MAV_STATE_STANDBY;
    RoverState.last_direction = HALT;
    RoverState.last_control_time = 0;
    RoverState.turning = 0;
}

void Rover_SetState(MAV_STATE state){
    RoverState.status = state;
}


MAV_STATE Rover_GetState(){
    return RoverState.status;
}


uint8_t Rover_CheckArmed(){
    return RoverState.mode & MAV_MODE_FLAG_SAFETY_ARMED ? 0x1:0;
}

uint8_t Rover_Disarm(){
    RoverState.mode = (RoverState.mode & ~MAV_MODE_FLAG_SAFETY_ARMED);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
    Rover_FullStopHard();
    return RoverState.mode;
}

uint8_t Rover_Arm(){
    /* TODO: Add some indication if armed (LED)*/
    RoverState.mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
    return RoverState.mode;
}

uint32_t Rover_GetId(){
    return RoverState.SYSTEM_ID;
}

MAV_MODE Rover_GetMode(){
    return RoverState.mode;
}

uint8_t Rover_ControlPending(){
    return RoverState.control_pending;
}

uint8_t Rover_SetControlPending(uint8_t value){
    RoverState.control_pending = value>0?1:0;
}

uint16_t Rover_SetVBat(uint16_t vbat){
    RoverState.vbat = vbat;
    return RoverState.vbat;
}

uint16_t Rover_GetVBat(){
    return RoverState.vbat;
}

/**
 * @brief Queue full stop
 */
void Rover_FullStopSoft(){
    RoverState.direction = HALT;
    RoverState.left_motor = 0;
    RoverState.right_motor = 0;
    RoverState.control_pending = 1;
}

/**
 * @brief Force full stop
 * 
 */
void Rover_FullStopHard(){
    RoverState.direction = HALT;
    RoverState.left_motor = 0;
    RoverState.right_motor = 0;
    Motor_FullStop();
}

void Rover_ProcessManualCtrl(mavlink_manual_control_t *control_msg) {
  /* * Incoming XYZ values are in [-1000, 1000] range; Motor values are in [0-100]*/
  if (RoverState.mode & MAV_MODE_FLAG_SAFETY_ARMED) {
    DEBUG_PRINTF(DBG_VERBOSE, "[ROVER] Processing manual control\r\n");
    uint16_t throttle_base, throttle_left, throttle_right;

    int16_t throttle = control_msg->x;
    int16_t steer = control_msg->r;

    if (throttle > 20) {
      RoverState.direction = FORWARD;
    } else if (throttle < -20) {
      RoverState.direction = REVERSE;
    } else {
      RoverState.direction = HALT;
    }
    int16_t left = throttle + steer;
    int16_t right = throttle - steer;
    left = left > 1000 ? 1000 : left < -1000 ? -1000 : left;
    right = right > 1000 ? 1000 : right < -1000 ? -1000 : right;

    RoverState.direction_left = left >= 0 ? FORWARD : REVERSE;
    RoverState.direction_right = right >= 0 ? FORWARD : REVERSE;
    RoverState.left_motor = (uint8_t)(abs(left) / 10);
    RoverState.right_motor = (uint8_t)(abs(right) / 10);
    DEBUG_PRINTF(DBG_INFO,
                 "[ROVER] Right motor: %d\r\nLeft motor: %d\r\nDirection: "
                 "%s\r\nDirection left: %s\r\nDirection right %s\r\n ",
                 RoverState.left_motor, RoverState.right_motor,
                 RoverState.direction == FORWARD   ? "Forward"
                 : RoverState.direction == REVERSE ? "Reverse"
                                                   : "Halt",
                 RoverState.direction_left == FORWARD ? "Forward" : "Reverse",
                 RoverState.direction_right == FORWARD ? "Forward" : "Reverse");

    RoverState.control_pending = 1;
  }
}

void Rover_ApplyControlState(){
    if(RoverState.mode & MAV_MODE_FLAG_SAFETY_ARMED){ 
        if(!RoverState.control_pending){
            return;
        }
        if (RoverState.direction != RoverState.last_direction){
            Motor_SetSpeed(BOTH, HALT, STOP);
            RoverState.last_direction = RoverState.direction;
        } 
        if (RoverState.direction_left != RoverState.direction_right){
            if (!RoverState.turning){
                Motor_SetSpeed(BOTH, HALT, STOP);
                RoverState.turning = 1;
            }
        } else { 
            if (RoverState.turning){
                Motor_SetSpeed(BOTH, HALT, STOP);
                RoverState.turning = 0;
            }

        }
    Motor_SetSpeed(LEFT, RoverState.direction_left, RoverState.left_motor);
    Motor_SetSpeed(RIGHT, RoverState.direction_right, RoverState.right_motor);
    RoverState.control_pending = 0;
    RoverState.last_control_time = HAL_GetTick();
  }
}