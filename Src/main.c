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

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
QMC_HandleTypedef	qmc_sensor;

extern I2C_HandleTypeDef hi2c1;
lsm303dlhc_data_raw_t lsm303dlhc_data_acc = { 0 };
lsm303dlhc_data_raw_t lsm303dlhc_data_mag = { 0 };
lsm303dlhc_data_t lsm303dlhc_data_acc_conv = { 0 };
lsm303dlhc_data_t lsm303dlhc_data_mag_conv = { 0 };

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

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
lsm303dlhc_acc_init_t lsm303dlhc_acc_init = { 0 };
	lsm303dlhc_mag_init_t lsm303dlhc_mag_init = { 0 };
	
	lsm303dlhc_acc_init.ctrl_reg1_a = LSM303DLHC_ACR1A_XEN | LSM303DLHC_ACR1A_YEN | LSM303DLHC_ACR1A_ZEN | LSM303DLHC_ACR1A_ODR30_100_HZ;
	lsm303dlhc_acc_init.ctrl_reg4_a = LSM303DLHC_ACR4A_FS10_1MG;
	
	lsm303dlhc_mag_init.op = LSM303DLHC_MAGOP_CONT;
	lsm303dlhc_mag_init.rate = LSM303DLHC_MAGRATE_15;
	lsm303dlhc_mag_init.gain = LSM303DLHC_MAGGAIN_1_3;
	lsm303dlhc_mag_init.auto_range = false;


  if (lsm303dlhc_init_acc(&hi2c1, &lsm303dlhc_acc_init) != LSM303DLHC_OK) {
    printf("LSM303DLHC Accel Init Error\r\n");
    }

	if (lsm303dlhc_init_mag(&hi2c1, &lsm303dlhc_mag_init) != LSM303DLHC_OK) {
		printf("LSM303DLHC Mag Init Error\r\n");
	}
  // if(  QMC5883_Init(&qmc_sensor, &hi2c1, qmc_continus) == QMC_OK )
  // {
  //     printf("QMC5883L Initialization Successful\r\n");
  // }
  // else
  // {
  //     printf("QMC5883L Initialization Failed\r\n");
  // }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    if (lsm303dlhc_read_acc_raw(&lsm303dlhc_data_acc) == LSM303DLHC_OK) {
			/* raw data in lsm303dlhc_data_acc */
			/* if conversion is needed: */
      printf("ACC X: %d, Y: %d, Z: %d\r\n", lsm303dlhc_data_acc.x, lsm303dlhc_data_acc.y, lsm303dlhc_data_acc.z);
			lsm303dlhc_convert_acc(&lsm303dlhc_data_acc_conv, &lsm303dlhc_data_acc);
      printf("ACC Conv X: %.3f, Y: %.3f, Z: %.3f\r\n", lsm303dlhc_data_acc_conv.x, lsm303dlhc_data_acc_conv.y, lsm303dlhc_data_acc_conv.z);
		} else {
			/* handle error */
		}
		
		if (lsm303dlhc_read_mag_raw(&lsm303dlhc_data_mag) == LSM303DLHC_OK) {
			/* raw data in lsm303dlhc_data_mag */

      printf("Compass Heading: %0.2f deg\r\n", LSM303_GetHeadingDegrees(&lsm303dlhc_data_mag));
		} else {
			/* handle error */
		}
      // QMC5883_ReadAverage(&qmc_sensor, 10, 50);
      // printf("Heading: %d.%03d deg, Compass: %d.%03d deg\r\n",
      //        qmc_sensor.avg_heading_whole,
      //        qmc_sensor.avg_heading_decimal,
      //        qmc_sensor.avg_compass_whole,
      //        qmc_sensor.avg_compass_decimal);
      // printf("TEMP: %d \r\n",QMC5883_ReadTemp(&qmc_sensor));
    HAL_Delay(50);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
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
