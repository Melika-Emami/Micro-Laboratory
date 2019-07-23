/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include "string.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#define APB1CLK   84000000
#define APB2CLK		168000000
extern int flag;
extern uint16_t buffer[1000];
extern int counter;
int counter = 0;
double Ts;
extern double T;
double T;
extern double tow;
double tow;
extern int N;
int N;
extern char line[128];
char line[128];

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart2;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles System tick timer.
  */
  /* USER CODE BEGIN SysTick_IRQn 0 */
double getch()
{
	char ch ;
	double val = 0;
	while(ch != '\n')
	{
		while(HAL_UART_Receive(&huart2,(unsigned char*)&ch,1,100) != HAL_OK){};
		if(ch != '\n')
		{
			if(ch != '.')			
				val	= (ch-'0')+ val * 10;
		}
	}
	return val;	
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
 
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line0 interrupt.
  */
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */

  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  /* USER CODE BEGIN EXTI0_IRQn 1 */
	
	//HAL_NVIC_ResetPendingIRQ(EXTI0_IRQn);
		HAL_UART_Transmit(&huart2,(uint8_t*)"please enter numbers:\n",strlen("please enter numbers:\n"),100);
	
	if(flag == 1){
			HAL_UART_Transmit(&huart2,(uint8_t*)"end\n",4,100);
		
			HAL_TIM_Base_Stop(&htim1);
			HAL_TIM_OC_Stop_IT(&htim1,TIM_CHANNEL_1);
	
			HAL_TIM_Base_Stop_IT(&htim2);
			HAL_ADC_Stop_DMA(&hadc1);
		  counter = 0;
			flag = 0;
		HAL_UART_Transmit(&huart2,(uint8_t*)"please enter numbers:\n",strlen("please enter numbers:\n"),100);
	}
		
		Ts = (getch()/100.00);
		T = (getch()/100.00);
		tow = (getch()/100.00);
		N = (int)getch();

		HAL_UART_Transmit(&huart2,(uint8_t*)"Thanks!\n",strlen("Thanks!\n"),100);
		//sprintf(line,"Ts: %f, T: %f, tow: %f N:%d",1000*Ts-1,1000*T-1,tow/T,N);
		//HAL_UART_Transmit(&huart2,(uint8_t*)line,50,100);		
		htim1.Init.Period= (1000*Ts)-1;
		htim1.Init.Prescaler=(APB2CLK/10000)-1;		
		HAL_TIM_Base_Init(&htim1);
		
		htim2.Init.Period= (10000*T)-1;
		htim2.Init.Prescaler=(APB1CLK/10000)-1;
		HAL_TIM_Base_Init(&htim2);

		HAL_TIM_Base_Start(&htim1);
		HAL_TIM_OC_Start_IT(&htim1,TIM_CHANNEL_1);

		HAL_TIM_Base_Start_IT(&htim2);
		
		HAL_ADC_Start_DMA(&hadc1,(uint32_t *)buffer,N);
		flag = 1;
		//if(HAL_NVIC_GetActive(EXTI0_IRQn))
		HAL_NVIC_ClearPendingIRQ(EXTI0_IRQn);
  /* USER CODE END EXTI0_IRQn 1 */
}

/**
  * @brief This function handles ADC1, ADC2 and ADC3 global interrupts.
  */
void ADC_IRQHandler(void)
{
  /* USER CODE BEGIN ADC_IRQn 0 */

  /* USER CODE END ADC_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  /* USER CODE BEGIN ADC_IRQn 1 */

  /* USER CODE END ADC_IRQn 1 */
}

/**
  * @brief This function handles TIM1 capture compare interrupt.
  */
void TIM1_CC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_CC_IRQn 0 */

  /* USER CODE END TIM1_CC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_CC_IRQn 1 */

  /* USER CODE END TIM1_CC_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

//	HAL_UART_Transmit(&huart2,(uint8_t*)"hus",3,100);
  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */
  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream0 global interrupt.
  */
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

  /* USER CODE END DMA2_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
