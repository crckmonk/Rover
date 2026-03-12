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

void Rover_SetStatus(MAV_STATE state){
    RoverState.status = state;
}

uint8_t Rover_CheckArmed(){
    return RoverState.mode & MAV_MODE_FLAG_SAFETY_ARMED ? 0x1:0;
}

uint8_t Rover_Disarm(){
    RoverState.mode = (RoverState.mode & ~MAV_MODE_FLAG_SAFETY_ARMED);
    return RoverState.mode;
}

uint8_t Rover_Arm(){
    RoverState.mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    return RoverState.mode;
}

