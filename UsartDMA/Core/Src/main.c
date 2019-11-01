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
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
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
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
#define DMA_RX_BUFFER_SIZE          64
uint8_t DMA_RX_Buffer[2][DMA_RX_BUFFER_SIZE];
bool Usart1CurrentBuffer=false;

#define UART_BUFFER_SIZE            256
uint8_t UART_Buffer[UART_BUFFER_SIZE];
size_t Write;
size_t len, tocopy;
uint8_t* ptr;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
int _write(int file, char *ptr, int len)
{
	/* Implement your write code here, this is used by puts and printf for example */

	//	int i=0;
	//  for(i=0 ; i<len ; i++)
	//    ITM_SendChar((*ptr++));
	//  return len;

	//
	HAL_UART_Transmit(&huart1, ptr, len,5);
	return  len;

}
void USART_IrqHandler (UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma)
{
	if (huart->Instance->SR & UART_FLAG_IDLE)           /* if Idle flag is set */
	{
		volatile uint32_t tmp;                  /* Must be volatile to prevent optimizations */

		//__HAL_DMA_DISABLE (hdma);       /* Disabling DMA will force transfer complete interrupt if enabled */
		printf("id\n");
		//DMA_IrqHandler (hdma, huart);
		tmp = huart->Instance->SR;                       /* Read status register */
		tmp = huart->Instance->DR;                       /* Read data register */
	}
}

void DMA_IrqHandler (DMA_HandleTypeDef *hdma, UART_HandleTypeDef *huart)
{
//	//if(__HAL_DMA_GET_IT_SOURCE(hdma, DMA_IT_HT) != RESET) return;
//	typedef struct
//	{
//		__IO uint32_t ISR;   /*!< DMA interrupt status register */
//		__IO uint32_t Reserved0;
//		__IO uint32_t IFCR;  /*!< DMA interrupt flag clear register */
//	} DMA_Base_Registers;
//
//	DMA_Base_Registers *regs = (DMA_Base_Registers *)hdma->DmaBaseAddress;

	if(__HAL_DMA_GET_IT_SOURCE(hdma, DMA_IT_TC) != RESET)   // if the source is TC
	{
		/* Clear the transfer complete flag */
		/* Important! DMA stream won't start if all flags are not cleared first */
		__HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_TC_FLAG_INDEX(hdma));
		/* Get the length of the data */
		len =  hdma->Instance->CNDTR;
		/* UNCOMMENT BELOW TO transmit the data via uart */

		HAL_UART_Transmit(&huart1, DMA_RX_Buffer[0], len, 10);

		Usart1CurrentBuffer=!Usart1CurrentBuffer;
		hdma->Instance->CMAR = (uint32_t)DMA_RX_Buffer[Usart1CurrentBuffer];
		/* Prepare DMA for next transfer no need in dma circular mode */

		//HAL_UART_Receive_DMA (&huart1, DMA_RX_Buffer, 64);
		// regs->IFCR = 0x3FU << hdma->ChannelIndex; // clear all interrupts
		//		hdma->Instance->CMAR = (uint32_t)DMA_RX_Buffer;   /* Set memory address for DMA again */
		//        hdma->Instance->CNDTR = DMA_RX_BUFFER_SIZE;    /* Set number of bytes to receive */
		//        hdma->Instance->CCR |= DMA_CCR_EN;            /* Start DMA transfer */
	}
}
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
	MX_DMA_Init();
	MX_USART1_UART_Init();
	/* USER CODE BEGIN 2 */


	//__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);   // enable idle line interrupt


	//	hdma_usart1_rx.Instance->CCR &= ~DMA_SxCR_HTIE;
	HAL_UART_Receive_DMA (&huart1, DMA_RX_Buffer[0], 64);
	__HAL_DMA_ENABLE_IT(&hdma_usart1_rx,DMA_IT_TC);
	__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT); 	// discable half complete interrupt
	__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_TE); 	// discable error transfer interrupt
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		//  printf("DMA CCR=%x\n",DMA1_Channel5->CCR);
		HAL_Delay(1000);
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
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) 
{

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

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
