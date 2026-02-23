#include "mcutils.h"
#include <math.h>

extern UART_HandleTypeDef huart1;

/*
TODO: Replace with proper printf implementation 
*/


void I2C_Scan(I2C_HandleTypeDef *hi2c){
  uint8_t StartMSG[] = "Starting I2C Scanning: \r\n";
  uint8_t EndMSG[] = "Done! \r\n\r\n";
  uint16_t i, ret =0;
    HAL_Delay(1000);
 
    /*-[ I2C Bus Scanning ]-*/
    printf("%s", StartMSG);
    for(i=1; i<128; i++)
    {
        ret = HAL_I2C_IsDeviceReady(hi2c, (uint16_t)(i<<1), 3, 5);
        if (ret != HAL_OK) /* No ACK Received At That Address */
        {
          printf(" - ");
          }
        else if(ret == HAL_OK)
        {
          printf("0x%X",i);
        }
    }
    printf("%s",EndMSG);
}


void DEBUG_PRINTF(DEBUG_LEVEL_t level, const char* format, ...){
    if (level > DEBUG_LEVEL) return;

    switch(level){
        case DEBUG_ERROR:   printf("[ERROR] "); break;
        case DEBUG_INFO:    printf("[INFO] "); break;
        case DEBUG_VERBOSE: printf("[VERBOSE] "); break;
        default: return;
    }

    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
}



void print_float(float value, uint16_t decimal_places){
    int whole = (int)value;
    int decimal = (int)((value - whole) * pow(10, decimal_places));
    if(decimal < 0) decimal = -decimal;  // Handle negative numbers
    printf("%d.%0*d", whole, decimal_places, decimal);
}