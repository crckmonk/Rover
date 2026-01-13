#include "mpu6050.h"


uint8_t MPU6500_Init (I2C_HandleTypeDef *i2c)
{
  uint8_t check;
  uint8_t Data;
  HAL_I2C_Mem_Read(i2c, MPU6500_ADDR, MPU6500_REG_WHOAMI, 1, &check, 1, 1000);  // read WHO_AM_I
  if (check == MPU6500_ID){
    Data = 0;
    HAL_I2C_Mem_Write(i2c, MPU6500_ADDR, MPU6500_REG_PWR_1,1,&Data,1,1000);
    Data = 0x7; // Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
    HAL_I2C_Mem_Write(i2c, MPU6500_ADDR, 0x19, 1, &Data, 1, 1000);
  }
  return check;
} 

void MPU650_Read_Gyro (I2C_HandleTypeDef *i2c, uint16_t *gyroXYZOut)
{
	uint8_t Rec_Data[6];
  uint16_t Gyro_X_RAW, Gyro_Y_RAW, Gyro_Z_RAW;
	// Read 6 BYTES of data starting from GYRO_XOUT_H register
	HAL_I2C_Mem_Read (i2c, MPU6500_ADDR, 0x43, 1, Rec_Data, 6, 1000);

	Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

  gyroXYZOut[0] = Gyro_X_RAW;
  gyroXYZOut[1] = Gyro_Y_RAW;
  gyroXYZOut[2] = Gyro_Y_RAW;
  return;
}

HAL_StatusTypeDef MPU6500_ReadTemp(I2C_HandleTypeDef *i2c, int16_t *temp){
    HAL_StatusTypeDef status;
    uint8_t buffer[2];

    // Read 2 bytes starting from TEMP_OUT_H
    status = HAL_I2C_Mem_Read(i2c, MPU6500_ADDR, TEMP_OUT_H, I2C_MEMADD_SIZE_8BIT, buffer, 2, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;

    // Combine bytes into signed 16-bit integer
    *temp = (int16_t)((buffer[0] << 8) | buffer[1]);

    return HAL_OK;
}