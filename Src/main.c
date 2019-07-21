/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
 *  Created on: 21.07.2019
 *      Author: Mateusz Salamon
 *		mateusz@msalamon.pl
 *
 *      Website: https://msalamon.pl/w-stm32-nie-ma-eepromu-ale-na-szczescie-jest-emulacja-eeprom-w-oparciu-o-f1-f4/
 *      GitHub:  https://github.com/lamik/EEPROM-emulation-STM32F4-HAL
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "eeprom.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* Virtual address defined by the user: 0xFFFF value is prohibited */
uint16_t VirtAddVarTab[NB_OF_VAR];
uint16_t VarDataTab[NB_OF_VAR] = {'M', 'a', 't', 'e', 'u', 's', 'z', ' ', 'S', 'a', 'l', 'a', 'm', 'o', 'n',
									' ', 'm', 's', 'a', 'l', 'a', 'm', 'o', 'n', '.', 'p', 'l'};
uint8_t VarDataTabRead[NB_OF_VAR];
uint16_t VarIndex,VarDataTmp = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* Unlock the Flash Program Erase controller */
  HAL_FLASH_Unlock();

  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /* EEPROM Init */
  if( EE_Init() != EE_OK)
  {
    Error_Handler();
  }

  // Fill EEPROM variables addresses
  for(VarIndex = 1; VarIndex <= NB_OF_VAR; VarIndex++)
  {
	  VirtAddVarTab[VarIndex-1] = VarIndex;
  }


  // Store Values in EEPROM emulation
  HAL_UART_Transmit(&huart2, "Store values\n\r", 14, 100);

  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	for (VarIndex = 0; VarIndex < NB_OF_VAR; VarIndex++)
	{
	  /* Sequence 1 */
	  if((EE_WriteVariable(VirtAddVarTab[VarIndex],  VarDataTab[VarIndex])) != HAL_OK)
	  {
		Error_Handler();
	  }
	}
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  // Read values
  HAL_UART_Transmit(&huart2, "Read values\n\r", 13, 100);
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
  for (VarIndex = 0; VarIndex < NB_OF_VAR; VarIndex++)
  {
	  if((EE_ReadVariable(VirtAddVarTab[VarIndex],  &VarDataTabRead[VarIndex])) != HAL_OK)
	  {
		  Error_Handler();
	  }
  }
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  HAL_UART_Transmit(&huart2, "Read table: ", 12, 100);
  HAL_UART_Transmit(&huart2, VarDataTabRead, NB_OF_VAR, 1000);
  HAL_UART_Transmit(&huart2, "\n\r", 2, 100);


  // Store revert Values in EEPROM emulation
  HAL_UART_Transmit(&huart2, "\n\r", 2, 100);
  HAL_UART_Transmit(&huart2, "Store revert values\n\r", 21, 100);

  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	for (VarIndex = 0; VarIndex < NB_OF_VAR; VarIndex++)
	{
	  /* Sequence 1 */
	  if((EE_WriteVariable(VirtAddVarTab[VarIndex],  VarDataTab[NB_OF_VAR-VarIndex-1])) != HAL_OK)
	  {
		Error_Handler();
	  }
	}
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  // Read values
  HAL_UART_Transmit(&huart2, "Read revert values\n\r", 20, 100);
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
  for (VarIndex = 0; VarIndex < NB_OF_VAR; VarIndex++)
  {
	  if((EE_ReadVariable(VirtAddVarTab[VarIndex],  &VarDataTabRead[VarIndex])) != HAL_OK)
	  {
		  Error_Handler();
	  }
  }
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  HAL_UART_Transmit(&huart2, "Read revert table: ", 19, 100);
  HAL_UART_Transmit(&huart2, VarDataTabRead, NB_OF_VAR, 1000);
  HAL_UART_Transmit(&huart2, "\n\r", 2, 100);

  // Store Values in EEPROM emulation
  HAL_UART_Transmit(&huart2, "\n\r", 2, 100);
  HAL_UART_Transmit(&huart2, "Store values\n\r", 14, 100);

  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	for (VarIndex = 0; VarIndex < NB_OF_VAR; VarIndex++)
	{
	  /* Sequence 1 */
	  if((EE_WriteVariable(VirtAddVarTab[VarIndex],  VarDataTab[VarIndex])) != HAL_OK)
	  {
		Error_Handler();
	  }
	}
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  // Read values
  HAL_UART_Transmit(&huart2, "Read values\n\r", 13, 100);
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
  for (VarIndex = 0; VarIndex < NB_OF_VAR; VarIndex++)
  {
	  if((EE_ReadVariable(VirtAddVarTab[VarIndex],  &VarDataTabRead[VarIndex])) != HAL_OK)
	  {
		  Error_Handler();
	  }
  }
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  HAL_UART_Transmit(&huart2, "Read table: ", 12, 100);
  HAL_UART_Transmit(&huart2, VarDataTabRead, NB_OF_VAR, 1000);
  HAL_UART_Transmit(&huart2, "\n\r", 2, 100);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	  while(1)
	  {
	    /* Toggle LED2 fast */
	    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	    HAL_Delay(40);
	  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
