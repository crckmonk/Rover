/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
 * TODO: Add RX/TX buffers and interrupt handlers for USART1 and USART6
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */


volatile UART_Buffers_t UART1_Buffer = {
  .huart = &huart1,
  .TxWrite = 0,
  .TxRead = 0,
  .TxBusy = 0,
  .RxHead = 0,
  .RxTail = 0,
  .RxByte = 0
};

volatile UART_Buffers_t UART6_Buffer = {
  .huart = &huart6,
  .TxWrite = 0,
  .TxRead = 0,
  .TxBusy = 0,
  .RxHead = 0,
  .RxTail = 0,
  .RxByte = 0
};


 
uint8_t UART_SendByte(volatile UART_Buffers_t* uart, uint8_t byte)
{
    uint16_t nextHead = (uart->TxWrite + 1) % UART_BUFFER_SIZE;
    uint32_t time_start = HAL_GetTick();
    while (nextHead == uart->TxRead && (HAL_GetTick() - time_start) < UART_TIMEOUT) {}

    if (nextHead == UART1_Buffer.TxRead){
      return 0; 
    }
    
    uart->TxBuffer[uart->TxWrite] = byte;
    uart->TxWrite = nextHead;
    if (!uart->TxBusy){
      uart->TxBusy = 1;
      HAL_UART_Transmit_IT(uart->huart,(uint8_t*)&uart->TxBuffer[uart->TxRead],1);
    } 
    
    return 1; 
}

uint8_t UART_SendData(volatile UART_Buffers_t* uart, uint8_t* data, uint32_t size){
    for (uint32_t i = 0; i < size; i++){
        if (!UART_SendByte(uart, data[i])){
            return 0;
        }
    }
    return 1; 
}


void UART_Flush(volatile UART_Buffers_t* uart){
    uart->TxWrite = 0;
    uart->TxRead = 0;
    uart->TxBusy = 0;
    uart->RxHead = 0;
    uart->RxTail = 0;
    uart->RxByte = 0;
    memset(uart->TxBuffer, 0, UART_BUFFER_SIZE);
    memset(uart->RxBuffer, 0, UART_BUFFER_SIZE);
}




void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    volatile UART_Buffers_t* uart = NULL;
    
    if (huart->Instance == USART1) {
        uart = &UART1_Buffer;
    } else if (huart->Instance == USART6) {
        uart = &UART6_Buffer;
    }
    if(uart){
      uint16_t nextHead = (uart->RxHead + 1) % UART_BUFFER_SIZE;
      if(nextHead != uart->RxTail){ 
        uart->RxBuffer[uart->RxHead] = uart->RxByte;
        uart->RxHead = nextHead;
      }
      HAL_UART_Receive_IT(uart->huart, &uart->RxByte, 1);
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{

    volatile UART_Buffers_t* uart = NULL;
    
    if (huart->Instance == USART1) {
        uart = &UART1_Buffer;
    } else if (huart->Instance == USART6) {
        uart = &UART6_Buffer;
    }
    if(uart){
      uart->TxRead = (uart->TxRead + 1) % UART_BUFFER_SIZE;
      if(uart->TxWrite != uart->TxRead){
        HAL_UART_Transmit_IT(uart->huart,(uint8_t*)&uart->TxBuffer[uart->TxRead],1);
      } else {
        uart->TxBusy = 0;
      }
  }
}
/* USER CODE END 0 */

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart6;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {

    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}
/* USART6 init function */

void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART6)
  {
  /* USER CODE BEGIN USART6_MspInit 0 */

  /* USER CODE END USART6_MspInit 0 */
    /* USART6 clock enable */
    __HAL_RCC_USART6_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART6 GPIO Configuration
    PA11     ------> USART6_TX
    PA12     ------> USART6_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART6 interrupt Init */
    HAL_NVIC_SetPriority(USART6_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART6_IRQn);
  /* USER CODE BEGIN USART6_MspInit 1 */

  /* USER CODE END USART6_MspInit 1 */
  }
}

/**
 * @brief 
 * @param uartHandle 
 */
void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART6)
  {
  /* USER CODE BEGIN USART6_MspDeInit 0 */

  /* USER CODE END USART6_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART6_CLK_DISABLE();

    /**USART6 GPIO Configuration
    PA11     ------> USART6_TX
    PA12     ------> USART6_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* USART6 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART6_IRQn);
  /* USER CODE BEGIN USART6_MspDeInit 1 */

  /* USER CODE END USART6_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void _putchar(char character)
{

    UART_SendByte(&UART1_Buffer, (uint8_t)character);
}
/* USER CODE END 1 */
