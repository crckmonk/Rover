#ifndef SRC_QMC5883L_QMC5883_H_
#define SRC_QMC5883L_QMC5883_H_

#include "stm32f4xx.h"
#define QMC_I2C_Address		0x1A

#define qmc_stanby		0
#define qmc_continus	1
#define QMC5883_TEMP_OFFSET  50000

#ifndef M_PI
#define M_PI 3.14
#endif

typedef enum
{
	QMC_Rate_200 = 0b00001100,
	QMC_Rate_100 = 0b00001000,
	QMC_Rate_50 =  0b00000100,
	QMC_Rate_10	=  0b00000000,
}QMC_DataRate;


/*Hardware abstract ======*/
typedef struct
{
	I2C_HandleTypeDef *i2c;
	uint8_t 		  ctl_reg;
	uint8_t			  data[6];
	int16_t			  xaxis;
	int16_t		 	  yaxis;
	int16_t			  zaxis;
	float 			  heading;
	float 			  compass;

	float 			  avg_heading;
	int32_t			  avg_heading_whole;
	uint32_t		  avg_heading_decimal;
	float 		      avg_compass;
	int32_t			  avg_compass_whole;
	uint32_t		  avg_compass_decimal;
}QMC_Handle_t;


HAL_StatusTypeDef QMC5883_Init(QMC_Handle_t *qmc, I2C_HandleTypeDef *i2c, QMC_DataRate data_rate);
HAL_StatusTypeDef QMC5883_ReadAverage(QMC_Handle_t *qmc, uint32_t maxAvrage, uint32_t timePerAvg);
uint16_t QMC5883_ReadTemp(QMC_Handle_t *qmc);
HAL_StatusTypeDef QMC5883_Read(QMC_Handle_t *qmc);
HAL_StatusTypeDef QMC5883_Standby(QMC_Handle_t *qmc);
HAL_StatusTypeDef QMC5883_Reset(QMC_Handle_t *qmc);

#endif /* SRC_QMC5883L_QMC5883_H_ */
