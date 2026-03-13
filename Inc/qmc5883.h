/*
 * QMC5883.h
 *
 *  Created on: Dec 9, 2024
 *
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge,
 * publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
 * AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * This File Part Of QMC5883 3-Axis Compass Sensor Library.
 * Port For STM32 MicroController With HAL Library.
 *
 *	Author: Mohammad Hosein Taheri
 *	gmail: etatosel@gmail.com
 * 
 * Versoin: 0.9
 */

 /*
	TODO: Calibration Function
		
 */


#ifndef SRC_QMC5883L_QMC5883_H_
#define SRC_QMC5883L_QMC5883_H_

#include "stm32f4xx.h"
#define QMC_I2C_Address		0x1A

#define QMC_OK			1
#define QMC_ERR			0

#define qmc_stanby		0
#define qmc_continus	1
#define QMC5883_TEMP_OFFSET  50000

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


uint8_t QMC5883_Init(QMC_Handle_t *qmc, I2C_HandleTypeDef *i2c, QMC_DataRate data_rate);
uint8_t QMC5883_ReadAverage(QMC_Handle_t *qmc, uint32_t maxAvrage, uint32_t timePerAvg);
uint16_t QMC5883_ReadTemp(QMC_Handle_t *qmc);
uint8_t QMC5883_Read(QMC_Handle_t *qmc);
float   QMC5883_ReadHeading(QMC_Handle_t *qmc);
uint8_t QMC5883_Standby(QMC_Handle_t *qmc);
uint8_t QMC5883_Reset(QMC_Handle_t *qmc);




#endif /* SRC_QMC5883L_QMC5883_H_ */
