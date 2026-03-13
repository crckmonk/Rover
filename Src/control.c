#include "control.h"


static Rover_t RoverState = {
    .SYSTEM_ID = 1,
    .mode = MAV_MODE_MANUAL_DISARMED,
    .status = MAV_STATE_STANDBY,
    .left_motor = 0,
    .right_motor = 0,
};


void Rover_ResetState(){
    RoverState.mode = MAV_MODE_MANUAL_DISARMED;
    RoverState.left_motor = 0;
    RoverState.right_motor = 0;
    RoverState.status = MAV_STATE_STANDBY;
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
    return RoverState.mode;
}

uint8_t Rover_Arm(){
    /* TODO: Add some indication if armed (LED)*/
    RoverState.mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    return RoverState.mode;
}

uint32_t Rover_GetId(){
    return RoverState.SYSTEM_ID;
}

MAV_MODE Rover_GetMode(){
    return RoverState.mode;
}

void Rover_ApplyManualControl(mavlink_manual_control_t *control_msg){
    if (RoverState.mode & MAV_MODE_FLAG_SAFETY_ARMED ){
        DEBUG_PRINTF(DEBUG_VERBOSE, "[ROVER] Apllying manual control");
        RoverState.last_control_time = HAL_GetTick();
    }
}