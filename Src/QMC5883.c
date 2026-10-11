#include "QMC5883.h"
#include <math.h>

HAL_StatusTypeDef QMC5883_Init(QMC_Handle_t *qmc, I2C_HandleTypeDef *i2c, QMC_DataRate data_rate)
{
    uint8_t array[2];
    qmc->i2c = i2c;
    qmc->ctl_reg = 0x11; //0001 0001 8G field range, Continuos Mode
    array[0] = 1;
    array[1] = qmc->ctl_reg;
    qmc->ctl_reg |= data_rate;

    if(HAL_I2C_Mem_Write(qmc->i2c, QMC_I2C_Address, 0x0B, 1, &array[0], 1, 100) != HAL_OK) return HAL_ERROR;
    if(HAL_I2C_Mem_Write(qmc->i2c, QMC_I2C_Address, 0x09, 1, &array[1], 1, 100) != HAL_OK) return HAL_ERROR;

    return HAL_OK;
}


uint16_t QMC5883_ReadTemp(QMC_Handle_t *qmc){
    uint8_t temp_low, temp_high;
    HAL_I2C_Mem_Read(qmc->i2c, QMC_I2C_Address, 0x07, 1, &temp_low, 1, 100);
    HAL_I2C_Mem_Read(qmc->i2c, QMC_I2C_Address, 0x08, 1, &temp_high, 1, 100);
    return (((uint16_t)temp_high << 8 | temp_low) - QMC5883_TEMP_OFFSET);
}

HAL_StatusTypeDef QMC5883_Read(QMC_Handle_t *qmc)
{
    qmc->data[0] = 0; 	//clear data index 0
    if (HAL_I2C_Mem_Read(qmc->i2c, QMC_I2C_Address, 0x06, 1, qmc->data, 1, 100) != HAL_OK) {
        return HAL_ERROR;
    }

    /*data ready for read =========*/
    if((qmc->data[0] & 0x01) == 1)
    {
        if (HAL_I2C_Mem_Read(qmc->i2c, QMC_I2C_Address, 0x00, 1, qmc->data, 6, 100) != HAL_OK) {
            return HAL_ERROR;
        }
        qmc->xaxis = (qmc->data[1]<<8) | qmc->data[0];
        qmc->yaxis = (qmc->data[3]<<8) | qmc->data[2];
        qmc->zaxis = (qmc->data[5]<<8) | qmc->data[4];

        qmc->compass = atan2f(qmc->yaxis, qmc->xaxis) * 180.00f / M_PI;
        if(qmc->compass > 0)
        {
            qmc->heading = qmc->compass;
        }
        else
        {
            qmc->heading = 360 + qmc->compass;
        }
        return HAL_OK;
    }
    
    return HAL_BUSY;  // Data not ready
}


HAL_StatusTypeDef QMC5883_ReadAverage(QMC_Handle_t *qmc, uint32_t maxAvrage, uint32_t timePerAvg)
{
    int i;
    float avg = 0.00f;
    avg = (float)maxAvrage;
    float avgH = 0.00f;
    float avgC = 0.00f;

    for (i = 0; i < maxAvrage; i++)
    {
        QMC5883_ReadHeading(qmc);
        avgC += qmc->compass;
        avgH += qmc->heading;

        HAL_Delay(timePerAvg);
    }
    qmc->avg_heading = avgH / avg;
    qmc->avg_compass = avgC / avg;

    qmc->avg_heading_whole = (int32_t)qmc->avg_heading;
    qmc->avg_heading_decimal = (uint32_t)((qmc->avg_heading - qmc->avg_heading_whole) * 1000);
    qmc->avg_compass_whole = (int32_t)qmc->avg_compass;
    qmc->avg_compass_decimal = (uint32_t)((qmc->avg_compass - qmc->avg_compass_whole) * 1000);

    return HAL_OK;
}

HAL_StatusTypeDef QMC5883_Standby(QMC_Handle_t *qmc)
{
    uint8_t array[1] = {0};
    return HAL_I2C_Mem_Read(qmc->i2c, QMC_I2C_Address, 0x09, 1, &array[0], 1, 100);
}


HAL_StatusTypeDef QMC5883_Reset(QMC_Handle_t *qmc)
{
    uint8_t array[1] = {0x80};  // Reset command
    return HAL_I2C_Mem_Write(qmc->i2c, QMC_I2C_Address, 0x0A, 1, &array[0], 1, 100);
}