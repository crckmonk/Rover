/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* User Configurations */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t TxBuffer[NRF24L01_PAYLOAD_LENGTH] = {0};
uint8_t RxBuffer[NRF24L01_PAYLOAD_LENGTH] = {0};
uint8_t RxAddr[NRF24_ADDR_WIDTH] = {0xD7,0xD7,0xD7,0xD7,0xD7};
uint8_t TxAddr[NRF24_ADDR_WIDTH] = {0xD7,0xD7,0xD7,0xD7,0xD7};
volatile uint8_t RxFlag = 0;
NRF24L01 nrf;
volatile uint8_t TxErrFlag = 0;
volatile uint8_t TxSuccess = 0;
volatile uint8_t TxReady = 0;
uint8_t ackCounter = 0;
uint8_t ackData[8] = {0};
uint8_t regTmp = 0;
telemetry_packet TxPacket = {0};
command_packet currentCommand = {0};
/* USER CODE END PV */

void executeCommand(command_packet *cmd);

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
QMC_HandleTypedef	qmc_sensor;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  //IMPORTANT: in launch.json set "device": "stm32f411xe" without .s
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  nrf.spi = &hspi1;
  nrf.DATA_RATE = NRF_DATA_RATE_1MBPS;
  nrf.RF_CHANNEL = NRF24_CHANNEL;
  nrf.PayloadLength = NRF24L01_PAYLOAD_LENGTH;
  nrf.RetransmitCount = 3;
  nrf.RetransmitDelay = 5;
  nrf.TX_POWER = NRF_TX_PWR_0dBm;
  nrf.RX_ADDRESS = RxAddr;
  nrf.TX_ADDRESS = TxAddr;
  nrf.CRC_WIDTH = NRF_CRC_WIDTH_1B;
  nrf.ADDR_WIDTH = NRF_ADDR_WIDTH_5;
  nrf.RX_BUFFER = RxBuffer;
  nrf.TX_BUFFER = TxBuffer;
  nrf.NRF_CSN_GPIOx = NRF24_CSN_GPIO_Port;
  nrf.NRF_CSN_GPIO_PIN = NRF24_CSN_Pin;
  nrf.NRF_CE_GPIOx = NRF24_CE_GPIO_Port;
  nrf.NRF_CE_GPIO_PIN = NRF24_CE_Pin;
  nrf.NRF_IRQ_GPIOx = NRF24_IRQ_GPIO_Port;
  nrf.NRF_IRQ_GPIO_PIN = NRF24_IRQ_Pin;
  nrf.NRF_IRQn = EXTI1_IRQn;
  nrf.NRF_IRQ_preempt_priority = 5;
  nrf.NRF_IRQ_sub_priority = 0;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  printf("TEST");
  
  MX_I2C1_Init();
  MX_SPI1_Init(); 
  /* USER CODE BEGIN 2 */
  Motor_Init(&htim2);
  /*Initialization*/
  nrf.STATE = NRF_STATE_TX;
  nrf.BUSY_FLAG = 0;
   if(NRF_Init(&nrf) == NRF_OK){
    
  } else {
    printf("TX NRF Init failed\r\n");
    Error_Handler();
  }
  NRF_SetRXAddress_P0(&nrf, TxAddr);
  NRF_EnableRXPipe(&nrf, 0);
  NRF_SetDynamicPayloadLength(&nrf, 1);
  NRF_EnableDynamicPayloadPipes(&nrf);
  NRF_EnableAckPayload(&nrf, 1);
  NRF_FlushRX(&nrf);
  NRF_FlushTX(&nrf);
  NRF_ClearInterrupts(&nrf);
  TxReady =1;
  printf("TX ready with ACK Payload...\r\n");
  /*Initialization DONE*/
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t lastTxTime = 0;
  while (1)
  {
    if(TxReady){
        TxSuccess = 0;
        TxErrFlag = 0;
        TxBuffer[0] = 0x11;
        TxBuffer[1] = TxPacket.motor_dir;
        TxBuffer[2] = TxPacket.left_motor_speed;
        TxBuffer[3] = TxPacket.right_motor_speed;
        TxBuffer[4] = 0; // battery voltage placeholder
        TxBuffer[5] = 0; // systems enabled placeholder
        TxBuffer[6] = 0;
        TxBuffer[7] = 0;
        lastTxTime = HAL_GetTick();  // ADD: Record time
        NRF_PushPacket(&nrf, TxBuffer);
                TxReady = 0;
                    }
      if (TxSuccess && nrf.BUSY_FLAG == 0)
      {
      //HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
      if (nrf.RX_BUFFER[0] != 0) {
        currentCommand.packet_type = nrf.RX_BUFFER[0];
        currentCommand.left_motors_speed = nrf.RX_BUFFER[1];
        currentCommand.right_motors_speed = nrf.RX_BUFFER[2];
        currentCommand.buttons = nrf.RX_BUFFER[3];
        currentCommand.reserved1 = nrf.RX_BUFFER[4];
        currentCommand.reserved2 = nrf.RX_BUFFER[5];
        currentCommand.reserved3 = nrf.RX_BUFFER[6];
        currentCommand.reserved4 = nrf.RX_BUFFER[7];
        for(int i=0;i<8;i++){
          printf("ACK Payload Byte %d: 0x%02X\r\n", i, nrf.RX_BUFFER[i]);
        }
        printf("\r\n");
        executeCommand(&currentCommand);
        // Process other reserved bytes if needed
        NRF_FlushRX(&nrf);
        TxReady = 1;
      } else {
        printf("TX OK (ACK), no ACK payload\r\n");
      }
    } else if ((HAL_GetTick() - lastTxTime) > 500 && nrf.BUSY_FLAG == 1)
    {
        printf("TX Timeout - resetting\r\n");
        NRF_FlushTX(&nrf);
        NRF_ClearInterrupts(&nrf);
        nrf.BUSY_FLAG = 0;
        TxReady = 1;
        TxErrFlag = 0;
        TxSuccess = 0;
    }
    else if (TxErrFlag)
    {
      printf("TX FAILED (no ACK after retries)\r\n");
      TxErrFlag = 0;
      TxReady = 1;
    }
    HAL_Delay(80);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

void executeCommand(command_packet* cmd) {
        switch(cmd->packet_type) {
          case 0x41: // GO
            Set_Motor_Speed(&htim2, LEFT, FORWARD, cmd->left_motors_speed);
            Set_Motor_Speed(&htim2, RIGHT, FORWARD, cmd->right_motors_speed);
            HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
            TxPacket.motor_dir = 1;
            TxPacket.left_motor_speed = cmd->left_motors_speed;
            TxPacket.right_motor_speed = cmd->right_motors_speed;
            break;
          case 0x42: // STOP
            Set_Motor_Speed(&htim2, BOTH, BRAKE, STOP);
            HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
            TxPacket.motor_dir = 0;
            break;
          case 0x43: // RVS
            Set_Motor_Speed(&htim2, LEFT, REVERSE, cmd->left_motors_speed);
            Set_Motor_Speed(&htim2, RIGHT, REVERSE, cmd->right_motors_speed);
            HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
            TxPacket.motor_dir = 2;
            TxPacket.left_motor_speed = cmd->left_motors_speed; 
            TxPacket.right_motor_speed = cmd->right_motors_speed;
            break;
          case 0x44: // KEEP
            HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET); // Blue LED ON
            break;
        }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
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

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_CSN_GPIO_Port, SPI1_CSN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NRF24_CE_GPIO_Port, NRF24_CE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_CSN_Pin */
  GPIO_InitStruct.Pin = SPI1_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI1_CSN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : NRF24_CE_Pin */
  GPIO_InitStruct.Pin = NRF24_CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(NRF24_CE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : NRF24_IRQ_Pin */
  GPIO_InitStruct.Pin = NRF24_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(NRF24_IRQ_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == nrf.NRF_IRQ_GPIO_PIN) {
    uint8_t status = 0;
    NRF_ReadRegister(&nrf, NRF_STATUS, &status);

    if (status & (1 << 5)) {  // TX_DS = ACK received
      TxSuccess = 1;
    }
    if (status & (1 << 4)) {  // MAX_RT = no ACK after retries
      TxErrFlag = 1;
    }
    regTmp = status;
    NRF_IRQ_Handler(&nrf);
  }
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    HAL_Delay(300);
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
