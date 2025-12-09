/* Structures and functions for remote control communication protocol
*/
#ifndef __CONTROL_H
#define __CONTROL_H

#include "main.h"

typedef struct  {
    uint8_t packet_type;    // 0x01 = FWD; 0x02 = STOP; 0x03 = REV; 0x04 = KEEP
    uint8_t left_motors_speed;        // rightside motors speed
    uint8_t right_motors_speed;         // left side motors speed
    uint8_t buttons;         // Buttons as a bitmap
    uint8_t reserved1;
    uint8_t reserved2;
    uint8_t reserved3;
    uint8_t reserved4;
}command_packet;


typedef struct {
    uint8_t packet_type;    // 0x11 = TELEMETRY
    uint8_t motor_dir;
    uint8_t left_motor_speed;
    uint8_t right_motor_speed;
    uint8_t battery_voltage; // in millivolts
    uint8_t systems_enabled; // Bitmap of systems on/off status
    uint8_t reserved1;
    uint8_t reserved2;
} telemetry_packet;


#endif