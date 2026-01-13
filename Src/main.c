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
#define SCANNER
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
uint8_t RxTxAddress[5] = {0xD7,0xD7,0xD7,0xD7,0xD7};
uint8_t txBuffer[NRF24_PAYLOAD_LENGTH] = {0};
uint8_t rxBuffer[NRF24_PAYLOAD_LENGTH] = {0};
NRF24L01 nrf;
command_packet currentCommand = {0};
command_packet rxCmdPacket;
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

void NRF24_PrintConfig(NRF24L01* dev) {
    /* TODO: Move to nrf24.c or other comms lib*/
    uint8_t reg;
    uint8_t addr[5];
    
    printf("\r\n=== NRF24L01 Configuration ===\r\n");
    
    // CONFIG register
    NRF24_ReadRegister(dev, NRF24_CONFIG, &reg);
    printf("CONFIG (0x00): 0x%02X\r\n", reg);
    printf("  - PWR_UP: %d\r\n", (reg >> 1) & 1);
    printf("  - PRIM_RX: %d (%s)\r\n", reg & 1, (reg & 1) ? "RX" : "TX");
    printf("  - CRC: %s, Width: %dB\r\n", (reg >> 3) & 1 ? "ON" : "OFF", ((reg >> 2) & 1) + 1);
    
    // EN_AA register
    NRF24_ReadRegister(dev, NRF24_EN_AA, &reg);
    printf("EN_AA (0x01): 0x%02X\r\n", reg);
    
    // EN_RXADDR register
    NRF24_ReadRegister(dev, NRF24_EN_RXADDR, &reg);
    printf("EN_RXADDR (0x02): 0x%02X\r\n", reg);
    
    // SETUP_AW register
    NRF24_ReadRegister(dev, NRF24_SETUP_AW, &reg);
    printf("SETUP_AW (0x03): 0x%02X (%d bytes)\r\n", reg, reg + 2);
    
    // SETUP_RETR register
    NRF24_ReadRegister(dev, NRF24_SETUP_RETR, &reg);
    printf("SETUP_RETR (0x04): 0x%02X\r\n", reg);
    printf("  - ARD: %d (%d us)\r\n", (reg >> 4) & 0x0F, ((reg >> 4) & 0x0F) * 250 + 250);
    printf("  - ARC: %d retransmits\r\n", reg & 0x0F);
    
    // RF_CH register
    NRF24_ReadRegister(dev, NRF24_RF_CH, &reg);
    printf("RF_CH (0x05): %d (%.0f MHz)\r\n", reg, 2400.0 + reg);
    
    // RF_SETUP register
    NRF24_ReadRegister(dev, NRF24_RF_SETUP, &reg);
    printf("RF_SETUP (0x06): 0x%02X\r\n", reg);
    printf("  - Data Rate: %s\r\n", 
           ((reg >> 3) & 1) ? "2Mbps" : (((reg >> 5) & 1) ? "250kbps" : "1Mbps"));
    printf("  - TX Power: %d dBm\r\n", -18 + ((reg >> 1) & 3) * 6);
    
    // STATUS register
    NRF24_ReadRegister(dev, NRF24_STATUS, &reg);
    printf("STATUS (0x07): 0x%02X\r\n", reg);
    printf("  - RX_DR: %d, TX_DS: %d, MAX_RT: %d\r\n", 
           (reg >> 6) & 1, (reg >> 5) & 1, (reg >> 4) & 1);
    printf("  - RX_P_NO: %d, TX_FULL: %d\r\n", (reg >> 1) & 7, reg & 1);
    
    // OBSERVE_TX register
    NRF24_ReadRegister(dev, NRF24_OBSERVE_TX, &reg);
    printf("OBSERVE_TX (0x08): 0x%02X (Lost: %d, Retrans: %d)\r\n", 
           reg, (reg >> 4) & 0x0F, reg & 0x0F);
    
    // RX_ADDR_P0
    uint8_t tx[5] = {0};
    NRF24_SendCommand(dev, NRF24_CMD_R_REGISTER | NRF24_RX_ADDR_P0, tx, addr, 5);
    printf("RX_ADDR_P0: %02X:%02X:%02X:%02X:%02X\r\n", 
           addr[0], addr[1], addr[2], addr[3], addr[4]);
    
    // TX_ADDR
    NRF24_SendCommand(dev, NRF24_CMD_R_REGISTER | NRF24_TX_ADDR, tx, addr, 5);
    printf("TX_ADDR:    %02X:%02X:%02X:%02X:%02X\r\n", 
           addr[0], addr[1], addr[2], addr[3], addr[4]);
    
    // RX_PW_P0
    NRF24_ReadRegister(dev, NRF24_RX_PW_P0, &reg);
    printf("RX_PW_P0 (0x11): %d bytes\r\n", reg);
    
    // FIFO_STATUS register
    NRF24_ReadRegister(dev, NRF24_FIFO_STATUS, &reg);
    printf("FIFO_STATUS (0x17): 0x%02X\r\n", reg);
    printf("  - TX: %s, RX: %s\r\n",
           (reg & 0x10) ? "Empty" : ((reg & 0x20) ? "Full" : "Data"),
           (reg & 0x01) ? "Empty" : ((reg & 0x02) ? "Full" : "Data"));
    
    // DYNPD register
    NRF24_ReadRegister(dev, NRF24_DYNPD, &reg);
    printf("DYNPD (0x1C): 0x%02X 0b%08b\r\n", reg,reg);
    
    // FEATURE register
    NRF24_ReadRegister(dev, NRF24_FEATURE, &reg);
    printf("FEATURE (0x1D): 0x%02X\r\n", reg);
    printf("  - EN_DPL: %d, EN_ACK_PAY: %d, EN_DYN_ACK: %d\r\n",
           (reg >> 2) & 1, (reg >> 1) & 1, reg & 1);
    
    printf("==============================\r\n\r\n");
}

void NRF24_PrintState(NRF24L01* dev){
  uint8_t reg;
  uint8_t str[32];
  NRF24_ReadRegister(dev, NRF24_CONFIG, &reg);
  printf("PWR_UP: %b PRIM_RX: %b CE: %b", (reg >> 1) & 1,  reg & 1, HAL_GPIO_ReadPin(dev->NRF24_CE_GPIOx, dev->NRF24_CE_GPIO_PIN));
      NRF24_ReadRegister(dev, NRF24_FIFO_STATUS, &reg);
    printf("FIFO_STATUS (0x17): 0x%02X\r\n", reg);
    printf("  - TX: %s, RX: %s\r\n", (reg & 0x10) ? "Empty" : ((reg & 0x20) ? "Full" : "Data"), (reg & 0x01) ? "Empty" : ((reg & 0x02) ? "Full" : "Data"));
    NRF24_ReadRegister(dev, NRF24_STATUS, &reg);
    printf("STATUS (0x07): 0x%02X\r\n", reg);
    printf("  - RX_DR: %d, TX_DS: %d, MAX_RT: %d\r\n", (reg >> 6) & 1, (reg >> 5) & 1, (reg >> 4) & 1);
    printf("  - RX_P_NO: %d, TX_FULL: %d\r\n", (reg >> 1) & 7, reg & 1);
    printf("IRQFlag: %d\r\n",nrf.IRQ_FLAG);
}


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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  
  MX_I2C1_Init();
  MX_SPI1_Init(); 
  /* USER CODE BEGIN 2 */
  uint8_t reg[6];
  uint16_t tmp[3];
  int16_t temp;
  printf("MPU6050 Init: %02x\r\n",MPU6500_Init(&hi2c1));
  MPU650_Read_Gyro(&hi2c1, tmp);
if(MPU6500_ReadTemp(&hi2c1, &temp) != HAL_OK){
    Error_Handler();
}

  printf("GYRO X: %04x Y: %04x Z: %04x\r\n",tmp[0],tmp[1],tmp[2]);
  printf("TEMP RAW: %d\r\n",temp);



  Motor_Init(&htim2);
  nrf.RX_BUFFER = rxBuffer;
  nrf.TX_BUFFER = txBuffer;
  Radio_InitNRF24(&nrf, &hspi1,RxTxAddress,NRF24_STATE_RX);
  /*Initialization*/
  
  NRF24_PrintConfig(&nrf);
  
  


  NRF24_CE_ENABLE(&nrf);
  NRF24_PrintState(&nrf);

  //printf("TX ready with ACK Payload...\r\n");
  /*Initialization DONE*/
  /* USER CODE END 2 */  

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint8_t ackCounter = 0;
  txBuffer[0] = 0x11;
  txBuffer[1] = 0xAA;
  txBuffer[2] = 0xAA;
  txBuffer[3] = ackCounter;


  NRF24_WriteAckPayload(&nrf,0,txBuffer,NRF24_PAYLOAD_LENGTH);
  while (1)
  {
    /*
    TODO: Implement RX with Ack payloads - Done
    */
    
   //Radio_NRF24RxMainLoop(&nrf, &telemetryPacket, &rxCmdPacket);
   if(nrf.IRQ_FLAG & NRF24_IRQ_RX_DR){
      NRF24_PullPacket(&nrf,rxBuffer);
      NRF24_WriteAckPayload(&nrf,0,txBuffer,NRF24_PAYLOAD_LENGTH);
      nrf.IRQ_FLAG = 0;
      nrf.BUSY_FLAG =1;
      if(rxBuffer[0]!= 0){
        txBuffer[3] = ackCounter +=1;
        rxCmdPacket.packet_type = rxBuffer[0];
        rxCmdPacket.left_motors_speed = rxBuffer[1];
        rxCmdPacket.right_motors_speed = rxBuffer[2];
        rxCmdPacket.buttons = rxBuffer[3];
        printf("RX: Success. Executing %02x\r\n",rxCmdPacket.packet_type);
        executeCommand(&rxCmdPacket);
      }
    }

   

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
            break;
          case 0x42: // STOP
            Set_Motor_Speed(&htim2, BOTH, BRAKE, STOP);
            HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
            break;
          case 0x43: // RVS
            Set_Motor_Speed(&htim2, LEFT, REVERSE, cmd->left_motors_speed);
            Set_Motor_Speed(&htim2, RIGHT, REVERSE, cmd->right_motors_speed);
            HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
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
  if (GPIO_Pin == nrf.NRF24_IRQ_GPIO_PIN) {
    NRF24_IRQ_Handler(&nrf);
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
