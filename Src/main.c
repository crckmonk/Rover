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
#include "gpio.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "NRF24/esp8266.h"
#include "mcutils.h"
// #include "yyjson.h"
#include "NRF24/ESP8266.h"
#include "global_config.h"
#include "motor.h"


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
uint8_t RxTxAddress[5] = {0xD7, 0xD7, 0xD7, 0xD7, 0xD7};
uint8_t txBuffer[NRF24_PAYLOAD_LENGTH] = {0};
uint8_t rxBuffer[NRF24_PAYLOAD_LENGTH] = {0};
NRF24_Handler_t nrf;
command_packet currentCommand = {0};
command_packet rxCmdPacket;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
QMC_Handle_t qmc_sensor;

extern UART_Buffers_t UART1_Buffer;
extern UART_Buffers_t UART6_Buffer;
extern I2C_HandleTypeDef hi2c1;
LSM303_RawData_t accData_raw = {0};
LSM303_RawData_t magData_raw = {0};

LSM303_Data_t accData_grav = {0};
LSM303_Data_t lsm303dlhc_data_mag_conv = {0};

LSM303_RawData_t mag_horizontal = {0};
LSM303_RawData_t magData_cal = {0};
LSM303_RawData_t accData_cal = {0};
float heading = 0.0f;
volatile uint8_t send_heartbeat = 0;
volatile uint8_t tele_timer =0;


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM3) {
    send_heartbeat = 1;
    tele_timer = tele_timer < 3? tele_timer+=1:0;
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  }
}


/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
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
  MX_USART6_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  // LSM303_AccInit_t lsm303dlhc_acc_init = {0};
  // LSM303_MagInit_t lsm303dlhc_mag_init = {0};

  // lsm303dlhc_acc_init.ctrl_reg1_a =
  //     LSM303DLHC_ACR1A_XEN | LSM303DLHC_ACR1A_YEN | LSM303DLHC_ACR1A_ZEN |
  //     LSM303DLHC_ACR1A_ODR30_100_HZ;
  // lsm303dlhc_acc_init.ctrl_reg4_a = LSM303DLHC_ACR4A_FS10_1MG;

  // lsm303dlhc_mag_init.op = LSM303DLHC_MAGOP_CONT;
  // lsm303dlhc_mag_init.rate = LSM303DLHC_MAGRATE_15;
  // lsm303dlhc_mag_init.gain = LSM303DLHC_MAGGAIN_1_3;
  // lsm303dlhc_mag_init.auto_range = false;

  DEBUG_PRINTF(DEBUG_INFO, "Testing ESP8266\r\n");
  DEBUG_PRINTF(DEBUG_INFO, "Initializing motors with TIM2\r\n");
  motor_Init(&htim2);
  
  ESP8266_Handler_t esp_dev;



  ESP8266_Init(&esp_dev, &huart6, &UART6_Buffer);
  esp_dev.ssid = (uint8_t *)"RoverAP";
  esp_dev.password = (uint8_t *)"moronik88";

  HAL_UART_Receive_IT(&huart6, &esp_dev.uart_buffers->RxByte, 1);

  while (  ESP8266_UDPSoftAP(&esp_dev) != ESP8266_OK) {
    DEBUG_PRINTF(DEBUG_ERROR, "SoftAP Initialization failed, retrying...\r\n");
    HAL_Delay(1000);
  }



  DEBUG_PRINTF(DEBUG_INFO, "UDP SoftAP Initialized successfully\r\n");
  DEBUG_PRINTF(DEBUG_VERBOSE, "RX Buffer: %s\r\n", esp_dev.rx_buffer);
  DEBUG_PRINTF(DEBUG_INFO, "Transmitting heartbeat\r\n");
  HAL_TIM_Base_Start_IT(&htim3);
  

  while (1) {
    ESP8266_MainLoop(&esp_dev);
    if (send_heartbeat) {
      MavLink_SendHeartbeat(&esp_dev);
      send_heartbeat = 0;
    }
    if ((__HAL_TIM_GET_COUNTER(&htim3) % 16000) == 0){
      ESP8266_SendTelemetry(&esp_dev);
    }
    ESP8266_MainLoop(&esp_dev);
    Rover_ApplyControlState();
    if (Rover_GetMode() & MAV_MODE_FLAG_SAFETY_ARMED){
      
    }
    // if (esp_dev.uart_buffers->RxWrite != esp_dev.uart_buffers->RxRead)
    // {
    //     uint8_t ch =
    //     esp_dev.uart_buffers->RxBuffer[esp_dev.uart_buffers->RxRead];
    //     esp_dev.uart_buffers->RxRead = (esp_dev.uart_buffers->RxRead + 1) %
    //     UART_BUFFER_SIZE; if(ch == '{'){
    //       writing = 1;
    //     } else if(ch == '}'){
    //       writing = 0;
    //       msgBuffer[idx++] = '}';
    //       msgBuffer[idx] = '\0';
    //       printf("Received message: %s\r\n", msgBuffer);
    //       JSON_CommandParse(msgBuffer, &cmdPacket);
    //       printf("Parsed Command - Direction: %d, Left Speed: %d, Right
    //       Speed: %d\r\n", cmdPacket.direction, cmdPacket.left_motors_speed,
    //       cmdPacket.right_motors_speed); executeCommand(&cmdPacket);
    //       memset(msgBuffer, 0, sizeof(msgBuffer)); // Clear buffer for next
    //       message idx = 0;
    //     }
    //     if (writing){
    //       msgBuffer[idx++] = ch;
    //     }
    // } else {
    // }

    // if (LSM303_ReadAccRaw(&accData_raw) == LSM303DLHC_OK) {
    //   LSM303_ApplyAccCalibration(&accData_raw, &accData_cal);
    // 	LSM303_ConvertAcc(&accData_grav, &accData_cal);
    //   printf("ACC Conv X: %.3f, Y: %.3f, Z: %.3f\r\n", accData_grav.x,
    //   accData_grav.y, accData_grav.z);
    //   //printf("ACC Conv X: %.3f, Y: %.3f, Z: %.3f\r\n", accData_grav.x,
    //   accData_grav.y, accData_grav.z);
    // } else {
    // 	/* handle error */
    // }

    // if (LSM303_ReadMagRaw(&magData_raw) == LSM303DLHC_OK) {
    //   LSM303_MagCalibrationUpdateRange(&magData_raw);
    //   LSM303_MagCalibrationCompute();
    //   LSM303_ApplyMagCalibration(&magData_raw, &magData_cal);
    //   heading = LSM303_GetHeadingDegrees(&magData_cal);
    //   printf("Calibrated heading: %0.2f deg\r\n", heading);

    // } else {
    // 	/* handle error */
    // }
    // QMC5883_ReadAverage(&qmc_sensor, 10, 50);
    // printf("Heading: %d.%03d deg, Compass: %d.%03d deg\r\n",
    //        qmc_sensor.avg_heading_whole,
    //        qmc_sensor.avg_heading_decimal,
    //        qmc_sensor.avg_compass_whole,
    //        qmc_sensor.avg_compass_decimal);
    // printf("TEMP: %d \r\n",QMC5883_ReadTemp(&qmc_sensor));
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == nrf.NRF24_IRQ_GPIO_PIN) {
    NRF24_IRQ_Handler(&nrf);
  }
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
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
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
