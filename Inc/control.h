/* Structures and functions for remote control communication protocol
*/
#ifndef __CONTROL_H
#define __CONTROL_H

#include "stm32f4xx.h"
#include "common/mavlink.h"
#include "mcutils.h"
#include "motor.h"


typedef struct  command_packet{
    /**/
    uint8_t direction;    // 0x01 = FWD; 0x02 = STOP; 0x03 = REV;
    uint8_t left_motors_speed;        // rightside motors speed
    uint8_t right_motors_speed;         // left side motors speed
    uint8_t buttons;         // Buttons as a bitmap
    uint8_t reserved1;
    uint8_t reserved2;
    uint8_t reserved3;
    uint8_t reserved4;
} command_packet;

typedef struct Rover_t{
    uint32_t SYSTEM_ID;
    MAV_MODE mode;
    MAV_STATE status;
    uint32_t last_control_time;
    uint8_t control_pending;
    motor_Direction_t direction;
    uint8_t left_motor;
    uint8_t right_motor;
} Rover_t;




#endif

void Rover_ResetState();

void Rover_SetState(MAV_STATE state);

MAV_STATE Rover_GetState();

uint8_t Rover_CheckArmed();

uint8_t Rover_Disarm();

uint8_t Rover_Arm();

uint32_t Rover_GetId();

MAV_MODE Rover_GetMode();

uint8_t Rover_ControlPending();

uint8_t Rover_SetControlPending(uint8_t value);

void Rover_ProcessManualCtrl(mavlink_manual_control_t *control_msg);

void Rover_ApplyControlState();
