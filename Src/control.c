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
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
    return RoverState.mode;
}

uint8_t Rover_Arm(){
    /* TODO: Add some indication if armed (LED)*/
    RoverState.mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
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

void Rover_ProcessManualCtrl(mavlink_manual_control_t *control_msg){
    /* * Incoming XYZ values are in [-1000, 1000] range; Motor values are in [0-100]*/
    if (RoverState.mode & MAV_MODE_FLAG_SAFETY_ARMED ){
        DEBUG_PRINTF(DEBUG_VERBOSE, "[ROVER] Processing manual control\r\n");
        uint16_t throttle_base,throttle_left, throttle_right;
        RoverState.direction = control_msg->x >=0 ? control_msg->x > 0 ?FORWARD : HALT : REVERSE;
        throttle_base = control_msg->x >=0 ? control_msg->x : -(control_msg->x); /* Get absolute value of x*/
        throttle_left = (uint16_t)(throttle_base + (control_msg->y));
        throttle_right = (uint16_t)(throttle_base - (control_msg->y));
        RoverState.left_motor = (uint8_t)(throttle_left <= 1000? (throttle_left / 10): 100);
        RoverState.right_motor = (uint8_t)( throttle_right <= 1000? (throttle_right / 10): 100);
        DEBUG_PRINTF(DEBUG_VERBOSE, "[ROVER] Right motor: %d\r\nLeft motor: %d\r\nDirection: %s\r\n", RoverState.left_motor, RoverState.right_motor, RoverState.direction == FORWARD? "Forward":"Reverse");

        RoverState.control_pending = 1;
    }
}

void Rover_ApplyControlState(){
    if(RoverState.mode & MAV_MODE_FLAG_SAFETY_ARMED){
        if(!RoverState.control_pending){
            return;
        }
        if (RoverState.direction != RoverState.last_direction){
            motor_SetSpeed(BOTH, HALT, STOP);
            RoverState.last_direction = RoverState.direction;
        }
        switch (RoverState.direction) {
        case FORWARD: // GO
            motor_SetSpeed(LEFT, FORWARD, RoverState.left_motor);
            motor_SetSpeed(RIGHT, FORWARD, RoverState.right_motor);
        break;
        case REVERSE: // RVS
            motor_SetSpeed(LEFT, REVERSE, RoverState.left_motor);
            motor_SetSpeed(RIGHT, REVERSE, RoverState.right_motor);
        break;
        default:
            motor_SetSpeed(BOTH, HALT, STOP);

    }
    RoverState.control_pending = 0;
    RoverState.last_control_time = HAL_GetTick();
  }
}