#include "mpu6050.h"


uint8_t MPU6050_Init (I2C_HandleTypeDef *i2c)
{
  uint8_t check;
  uint8_t Data;

  HAL_I2C_Mem_Read (i2c, MPU6050_ADDR, 0x75, 1, &check, 1, 1000);  // read WHO_AM_I
  if (check == 0x68)  // 0x68 will be returned by the sensor if everything goes well
  {
  }
  return check;
}