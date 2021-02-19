/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : stm32f1xx_hal_msp.c
  * Description        : This file provides code for the MSP Initialization 
  *                      and de-Initialization codes.
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
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */
 
/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{
  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  /* System interrupt init*/

  /** NOJTAG: JTAG-DP Disabled and SW-DP Enabled 
  */
  __HAL_AFIO_REMAP_SWJ_NOJTAG();

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}

/* USER CODE BEGIN 1 */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim){

	 __HAL_RCC_TIM2_CLK_ENABLE();
	HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(TIM2_IRQn);
}
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *htim){

	__HAL_RCC_GPIOA_CLK_DISABLE();
	__HAL_RCC_TIM2_CLK_DISABLE();
	HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0 | GPIO_PIN_1);
	HAL_NVIC_DisableIRQ(TIM2_IRQn);
}


void HAL_TIM_OC_MspInit(TIM_HandleTypeDef *htim)
{
	GPIO_InitTypeDef sConfig = { 0 };

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_TIM2_CLK_ENABLE();

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);

	sConfig.Pin = GPIO_PIN_0 | GPIO_PIN_1;
	sConfig.Mode = GPIO_MODE_AF_PP;
	sConfig.Pull = GPIO_NOPULL;
	sConfig.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &sConfig);

	HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(TIM2_IRQn);
}


void HAL_TIM_OC_MspDeInit(TIM_HandleTypeDef *htim)
{
	__HAL_RCC_GPIOA_CLK_DISABLE();
	__HAL_RCC_TIM2_CLK_DISABLE();
	HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0 | GPIO_PIN_1);
	HAL_NVIC_DisableIRQ(TIM2_IRQn);
}

void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
	if(hadc->Instance == ADC1)
	{
		GPIO_InitTypeDef adc_pin = {0};

		__HAL_RCC_ADC1_CLK_ENABLE();
		__HAL_RCC_GPIOA_CLK_ENABLE();

		adc_pin.Mode = GPIO_MODE_ANALOG;
		adc_pin.Pin = GPIO_PIN_3;

		HAL_GPIO_Init(GPIOA,&adc_pin);


	}


}


void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc)
{
	if(hadc->Instance == ADC1)
		{
		__HAL_RCC_ADC1_CLK_DISABLE();
		__HAL_RCC_GPIOA_CLK_DISABLE();
		HAL_GPIO_DeInit(GPIOA,GPIO_PIN_3);

		}

}




__weak void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *htim){}
__weak void HAL_TIM_Encoder_MspDeInit(TIM_HandleTypeDef *htim){}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
