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
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

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

HAL_StatusTypeDef status;

// Basic initialization
status = MPU6500_Init();
if(status != HAL_OK){
    Error_Handler();
}

// Optional: Read WHO_AM_I register to verify communication
uint8_t whoami;
status = MPU6500_ReadWhoAmI(&whoami);
if(status != HAL_OK || whoami != 0x70){
    Error_Handler();
}

int16_t accel_x, accel_y, accel_z;
int16_t gyro_x, gyro_y, gyro_z;
int16_t temperature;
float accel_g[3];    // Acceleration in g
float gyro_dps[3];   // Angular velocity in degrees per second
float temp_c;        // Temperature in Celsius

// Read raw sensor data
status = MPU6500_ReadAccel(&accel_x, &accel_y, &accel_z);
if(status != HAL_OK){
    Error_Handler();
}

status = MPU6500_ReadGyro(&gyro_x, &gyro_y, &gyro_z);
if(status != HAL_OK){
    Error_Handler();
}

status = MPU6500_ReadTemp(&temperature);
if(status != HAL_OK){
    Error_Handler();
}

// Convert raw data to physical units
// For ±16g range: 1g = 2048 LSB
accel_g[0] = (float)accel_x / 2048.0f;
accel_g[1] = (float)accel_y / 2048.0f;
accel_g[2] = (float)accel_z / 2048.0f;

// For ±2000°/s range: 1°/s = 16.4 LSB
gyro_dps[0] = (float)gyro_x / 16.4f;
gyro_dps[1] = (float)gyro_y / 16.4f;
gyro_dps[2] = (float)gyro_z / 16.4f;

// Temperature conversion: T(°C) = (TEMP_OUT / 340) + 36.53
// temp_c = ((float)temperature / 340.0f) + 36.53f;

// Temperature conversion: T(°C) = (TEMP_OUT / 333.87) + 21
temp_c = ((float)temperature) / 333.87f + 21.0f;

printf("Accel: X=%.2fg Y=%.2fg Z=%.2fg | Gyro: X=%.1f Y=%.1f Z=%.1f dps | Temp: %.1fC\r\n",
       accel_g[0], accel_g[1], accel_g[2],
       gyro_dps[0], gyro_dps[1], gyro_dps[2],
       temp_c);

Motor_Init(&htim2);

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
  txBuffer[2] = (uint8_t)temp_c;
  txBuffer[3] = ackCounter;


  NRF24_WriteAckPayload(&nrf,0,txBuffer,NRF24_PAYLOAD_LENGTH);
  while (1)
  {
    /*
    TODO: Implement RX with Ack payloads - Done
    */
   if(MPU6500_ReadTemp(&temperature) != HAL_OK){
    printf("ERROR\r\n");
  }
  temp_c = ((float)temperature) / 333.87f + 21.0f;
  txBuffer[2] = (uint8_t)temp_c;
   //Radio_NRF24RxMainLoop(&nrf, &telemetryPacket, &rxCmdPacket);
   if(nrf.IRQ_FLAG & NRF24_IRQ_RX_DR){
      NRF24_PullPacket(&nrf,rxBuffer);
      NRF24_WriteAckPayload(&nrf,0,txBuffer,NRF24_PAYLOAD_LENGTH);
      printf("TX 0: %d 1: %d 2: %d 3: %d\r\n",txBuffer[0],txBuffer[1],txBuffer[2],txBuffer[3]);
      printf("RX 0: %02x 1: %d 2: %f 3: %d\r\n",rxBuffer[0],rxBuffer[1],rxBuffer[2],rxBuffer[3]);
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

/* USER CODE BEGIN 4 */
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
