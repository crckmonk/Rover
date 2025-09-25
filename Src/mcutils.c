#include "mcutils.h"
#include <stdio.h>
#include <math.h>


extern UART_HandleTypeDef huart1;

int __io_putchar(int ch)
{
 uint8_t c[1];
 c[0] = ch & 0x00FF;
 HAL_UART_Transmit_DMA(&huart1, &*c, 1);
 return ch;
}

int _write(int file,char *ptr, int len)
{
 int DataIdx;
 for(DataIdx= 0; DataIdx < len; DataIdx++)
 {
 __io_putchar(*ptr++);
 }
return len;
}

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


void print_float(float value, uint16_t decimal_places){
    int whole = (int)value;
    int decimal = (int)((value - whole) * pow(10, decimal_places));
    if(decimal < 0) decimal = -decimal;  // Handle negative numbers
    printf("%d.%0*d", whole, decimal_places, decimal);
}