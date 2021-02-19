/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
TIM_HandleTypeDef htim2;
ADC_HandleTypeDef handle_adc;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t adcv;
uint16_t setpwm;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
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
  /* USER CODE BEGIN 2 */
  MX_TIM2_Init();
  MX_ADC1_Init();

  HAL_ADC_Start(&handle_adc);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		adcv = HAL_ADC_GetValue(&handle_adc);
		setpwm = (int)(adcv / 4.096);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, setpwm);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, setpwm);
		HAL_Delay(10);




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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); ///encoder's button pin

  /*Configure GPIO pins : PC13 PC14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


}

/* USER CODE BEGIN 4 */
static void MX_TIM2_Init(void)
{

	TIM_ClockConfigTypeDef Clock_Source_Config = {0};
	TIM_MasterConfigTypeDef master_Config = {0};
	TIM_OC_InitTypeDef PWM_Output_Config_CH  = {0};

	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 72-1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 1000-1;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
		{Error_Handler();}

	Clock_Source_Config.ClockSource =TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &Clock_Source_Config) != HAL_OK)
			{Error_Handler();}

	 if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
		  {Error_Handler();}


	master_Config.MasterOutputTrigger = TIM_TRGO_RESET;
	master_Config.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &master_Config) != HAL_OK)
		{Error_Handler();}

	PWM_Output_Config_CH.OCMode = TIM_OCMODE_PWM1;
	PWM_Output_Config_CH.Pulse = 0;
	PWM_Output_Config_CH.OCPolarity = TIM_OCPOLARITY_HIGH;
	PWM_Output_Config_CH.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &PWM_Output_Config_CH, TIM_CHANNEL_1)!= HAL_OK)
		{Error_Handler();}

	if (HAL_TIM_PWM_ConfigChannel(&htim2, &PWM_Output_Config_CH, TIM_CHANNEL_2)!= HAL_OK)
		{Error_Handler();}

	HAL_TIM_OC_MspInit(&htim2);



}

void MX_ADC1_Init(void)
{
	ADC_ChannelConfTypeDef adc_ch_config = {0};

	handle_adc.Instance = ADC1;
	handle_adc.Init.ContinuousConvMode = ENABLE;
	handle_adc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	handle_adc.Init.DiscontinuousConvMode = DISABLE;
	handle_adc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	handle_adc.Init.NbrOfConversion=1;
	handle_adc.Init.NbrOfDiscConversion=0;
	handle_adc.Init.ScanConvMode=ADC_SCAN_DISABLE;
	if (HAL_ADC_Init(&handle_adc)!= HAL_OK)
	{Error_Handler();}

	adc_ch_config.Channel = ADC_CHANNEL_3;
	adc_ch_config.Rank = ADC_REGULAR_RANK_1;
	adc_ch_config.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
	if (HAL_ADC_ConfigChannel(&handle_adc, &adc_ch_config)!= HAL_OK)
	{Error_Handler();}


}

__weak void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){}
__weak void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim){}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
