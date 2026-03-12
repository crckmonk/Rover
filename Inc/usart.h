/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#define UART_TIMEOUT 1000
/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;

extern UART_HandleTypeDef huart6;

/* USER CODE BEGIN Private defines */


#define UART_TX_BUFFER_SIZE 512
#define UART_RX_BUFFER_SIZE 2048

typedef struct {
     UART_HandleTypeDef* huart;
     uint8_t TxBuffer[UART_TX_BUFFER_SIZE];
     uint16_t TxWrite;
     uint16_t TxRead;
     uint8_t TxBusy;
     uint8_t RxBuffer[UART_RX_BUFFER_SIZE];
     uint16_t RxWrite;
     uint16_t RxRead;
     uint8_t RxByte;  // Single byte for HAL_UART_Receive_IT
} UART_Buffers_t;

typedef enum {
  UART_ERROR = 0,
  UART_OK,
  UART_EMPTY
} UART_Status_t;


/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);
void MX_USART6_UART_Init(void);

/* USER CODE BEGIN Prototypes */
void _putchar(char character);
UART_Status_t UART_SendByte(volatile UART_Buffers_t* uart, uint8_t byte);
UART_Status_t UART_SendData(volatile UART_Buffers_t* uart, uint8_t* data, uint32_t size);
uint8_t UART_GetByte(volatile UART_Buffers_t *uart);
uint8_t UART_RxDataAvailable(volatile UART_Buffers_t *uart);
void UART_FlushTx(volatile UART_Buffers_t* uart);
void UART_FlushRx(volatile UART_Buffers_t *uart);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

