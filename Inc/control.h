/* Structures and functions for remote control communication protocol
*/
#ifndef __CONTROL_H
#define __CONTROL_H

#include "main.h"

struct command_packet {
    uint8_t packet_type;    // 0x02 = command
    uint8_t command;        // Motor commands, etc.
    int16_t param1;         // Speed, direction
    int16_t param2;         // Additional parameter
    uint8_t checksum;       // 1 byte
};



#endif