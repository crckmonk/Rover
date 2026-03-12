/* Structures and functions for remote control communication protocol
*/
#ifndef __CONTROL_H
#define __CONTROL_H

#include "stm32f4xx.h"
#include "common/mavlink.h"

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
    uint8_t left_motor;
    uint8_t right_motor;
} Rover_t;




#endif