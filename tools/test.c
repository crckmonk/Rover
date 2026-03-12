#include <stdio.h>
#include "common/mavlink.h"

void print_binary(unsigned int num) {
    for (int i = 32; i >= 1; i--) {
        if (i%8 == 0){
            printf(" ");
        }
        printf("%d", (num >> i) & 1);
    }
    printf("\n");
}

int main(int argc, char** argv){
    uint8_t test = MAV_MODE_MANUAL_ARMED;
    printf("MANUAL_ARMED:    ");
    print_binary(test);
    printf("MANUAL_DISARMED: ");
    print_binary(MAV_MODE_MANUAL_DISARMED);
    printf("\r\nFLAG_ARMED: ");
    print_binary(MAV_MODE_FLAG_SAFETY_ARMED);
    printf("\r\nMAV_MODE_MANUAL_ARMED & MAV_MODE_FLAG_SAFETY_ARMED:");
    test &= MAV_MODE_FLAG_SAFETY_ARMED;
    print_binary(test);
    printf("CHECKING ARMED WITH &:    %x\r\n",MAV_MODE_MANUAL_ARMED & MAV_MODE_FLAG_SAFETY_ARMED);
    printf("CHECKING DISARMED WITH &: %x\r\n",MAV_MODE_MANUAL_DISARMED & MAV_MODE_FLAG_SAFETY_ARMED);
    printf("SETTING ARMED WITH |:    ");
    print_binary(MAV_MODE_MANUAL_DISARMED | MAV_MODE_FLAG_SAFETY_ARMED);
    printf("RESETTING ARMED WITH &~: ");
    print_binary(MAV_MODE_MANUAL_ARMED & ~MAV_MODE_FLAG_SAFETY_ARMED);
    return 0;
}