/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <stdio.h>
int info;
int newInfo;
void TransmitChar(char data);
void TransmitString(char* data);
void TransmitChar(char data) {
	while (!(USART3->ISR & (1 << 7))) {

	}
	//GPIOC->ODR = (0 << 9);
	USART3->TDR = data;
}

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
	RCC->AHBENR |= (1 << 18) | (1 << 19);
	RCC->APB1ENR |= (1 << 18); // Enable USART


	GPIOB->MODER = (2 << 20) | (2 << 22);
	GPIOC->MODER = (1 << 12) | (1 << 14) | (1 << 16) | (1 << 18);
	GPIOB->OTYPER = (0 << 8) | (0 << 10);
	GPIOB->OSPEEDR = (0 << 8) | (0 << 10);
	GPIOB->PUPDR = (0 << 8) | (0 << 10);
	GPIOB->ODR = 0;
	//GPIOC->ODR = (1 << 9);

	GPIOB->AFR[1] = (4 << 8) | (4 << 12);

	//RCC->APB1RSTR |= (1 << 18); // Reset USART

	USART3->BRR = HAL_RCC_GetHCLKFreq() / 115200;				// Get system clock frequency
	//USART3->CR1 |= (1 << 0) |(1 << 2);
	//USART3->CR1 |= (1 << 3);
	//USART3->CR1 = USART_CR1_TE | USART_CR1_UE; /* (2) */
	USART3->CR1 = USART_CR1_TE | USART_CR1_RXNEIE | USART_CR1_RE | USART_CR1_UE; /* (2) */
	//USART3->CR1 |= (1 << 5);
	//USART3->ISR &= (1 << 5);

	//EXTI->IMR |= 28;
	//EXTI->RTSR |= 28;	
	//SYSCFG->EXTICR[0] &= ~1;

	NVIC_EnableIRQ(USART3_4_IRQn);
	NVIC_SetPriority(USART3_4_IRQn, 0);


	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	/* USER CODE BEGIN 2 */
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		//GPIOC->ODR ^= (1 << 6);
	/* USER CODE END WHILE */
		//TransmitChar('s');
		if (newInfo) {
			//TransmitChar('s');
		}

		uint16_t string;
		if (USART3->ISR & (1 << 5)) {
			string = USART3->RDR;
			if ((char)string == 'r') {
				GPIOC->ODR ^= (1 << 6);
			}
			if ((char)string == 'b') {
				GPIOC->ODR ^= (1 << 7);
			}
			if ((char)string == 'o') {
				GPIOC->ODR ^= (1 << 8);
			}
			if ((char)string == 'g') {
				GPIOC->ODR ^= (1 << 9);
			}
			TransmitChar((char)info);
		}

		//HAL_Delay(0);
	/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}



void TransmitString(char* data) {
	int i = 0;
	char letter = *(data + i);
	while (letter != 0) {
		TransmitChar(letter);
		i++;
		letter = *(data + i);
	}
}

void USART3_4_IRQHandler() {
	if (USART3->ISR &= (1 << 5)) {
		if (!newInfo) {
			info = USART3->RDR;
			newInfo = 1;
		}
		else {
			int correct = 0;
			char* message = "Sorry, your command was not recognized!";
			uint16_t string = USART3->RDR;;
			if ((char)info == 'r') {
				if ((char)string == '0') {
					GPIOC->ODR &= ~(1 << 6);
					correct = 1;
					message = "Red turned off";
				}
				else if ((char)string == '1') {
					GPIOC->ODR |= (1 << 6);
					correct = 1;
					message = "Red turned on";
				}
				else if ((char)string == '2') {
					GPIOC->ODR ^= (1 << 6);
					correct = 1;
					message = "Red toggled";
				}
			}
			if ((char)info == 'b') {
				if ((char)string == '0') {
					GPIOC->ODR &= ~(1 << 7);
					correct = 1;
					message = "Blue turned off";
				}
				else if ((char)string == '1') {
					GPIOC->ODR |= (1 << 7);
					correct = 1;
					message = "Blue turned on";
				}
				else if ((char)string == '2') {
					GPIOC->ODR ^= (1 << 7);
					correct = 1;
					message = "Blue toggled";
				}
			}
			if ((char)info == 'o') {
				if ((char)string == '0') {
					GPIOC->ODR &= ~(1 << 8);
					correct = 1;
					message = "Orange turned off";
				}
				else if ((char)string == '1') {
					GPIOC->ODR |= (1 << 8);
					correct = 1;
					message = "Orange turned on";
				}
				else if ((char)string == '2') {
					GPIOC->ODR ^= (1 << 8);
					correct = 1;
					message = "Orange toggled";
				}
			}
			if ((char)info == 'g') {
				if ((char)string == '0') {
					GPIOC->ODR &= ~(1 << 9);
					correct = 1;
					message = "Green turned off";
				}
				else if ((char)string == '1') {
					GPIOC->ODR |= (1 << 9);
					correct = 1;
					message = "Green turned on";
				}
				else if ((char)string == '2') {
					GPIOC->ODR ^= (1 << 9);
					correct = 1;
					message = "Green toggled";
				}
			}
			char text[] = { (char)info, (char)string, '\r', '\n' };
			TransmitString(text);
			TransmitString(message);
			TransmitString("\r\n");
			//if(!correct) {
			//	TransmitString("Sorry, your command was not recognized!\r\n");
			//}
			TransmitString("CMD?\r\n");
			newInfo = 0;
		}
	}
}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	* in the RCC_OscInitTypeDef structure.
	*/
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	*/
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
		| RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
	{
		Error_Handler();
	}
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
	__disable_irq();
	while (1)
	{
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
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	   ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	   /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
