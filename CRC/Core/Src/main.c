/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

/* USER CODE BEGIN PV */
uint8_t TestBuffer[]={1,2,3,4,5,6,7,8};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//****************************************************************************
int _write(int file, char *ptr, int len)
{
	/* Implement your write code here, this is used by puts and printf for example */

	int i=0;
	for(i=0 ; i<len ; i++)
		ITM_SendChar((*ptr++));
	return len;

	//
	//	HAL_UART_Transmit(&huart1, ptr, len,5);
	//	return  len;

}
//****************************************************************************
uint32_t Crc32(uint32_t Crc, uint32_t Data)
{
	int i;

	Crc = Crc ^ Data;

	for(i=0; i<32; i++)
		if (Crc & 0x80000000)
			Crc = (Crc << 1) ^ 0x04C11DB7; // Polynomial used in STM32
		else
			Crc = (Crc << 1);

	return(Crc);
}
//****************************************************************************
uint32_t Crc32Fast(uint32_t Crc, uint32_t Data)
{
	static const uint32_t CrcTable[16] = { // Nibble lookup table for 0x04C11DB7 polynomial
			0x00000000,0x04C11DB7,0x09823B6E,0x0D4326D9,0x130476DC,0x17C56B6B,0x1A864DB2,0x1E475005,
			0x2608EDB8,0x22C9F00F,0x2F8AD6D6,0x2B4BCB61,0x350C9B64,0x31CD86D3,0x3C8EA00A,0x384FBDBD };

	Crc = Crc ^ Data; // Apply all 32-bits

	// Process 32-bits, 4 at a time, or 8 rounds

	Crc = (Crc << 4) ^ CrcTable[Crc >> 28]; // Assumes 32-bit reg, masking index to 4-bits
	Crc = (Crc << 4) ^ CrcTable[Crc >> 28]; //  0x04C11DB7 Polynomial used in STM32
	Crc = (Crc << 4) ^ CrcTable[Crc >> 28];
	Crc = (Crc << 4) ^ CrcTable[Crc >> 28];
	Crc = (Crc << 4) ^ CrcTable[Crc >> 28];
	Crc = (Crc << 4) ^ CrcTable[Crc >> 28];
	Crc = (Crc << 4) ^ CrcTable[Crc >> 28];
	Crc = (Crc << 4) ^ CrcTable[Crc >> 28];

	return(Crc);
}
//****************************************************************************
uint32_t Crc32SlowBlock(uint32_t Crc, uint32_t Size, uint32_t *Data)
{
	while(Size--)
		Crc = Crc32(Crc, *Data++); // 32-bit at a time

	return(Crc);
}
//****************************************************************************
uint32_t Crc32FastBlock(uint32_t Crc, uint32_t Size, uint32_t *Data)
{
	while(Size--)
		Crc = Crc32Fast(Crc, *Data++); // 32-bit at a time

	return(Crc);
}
//****************************************************************************
void ALLCRCTypeTest(CRC_HandleTypeDef *hcrc)
{
	int myDataBufferCount=sizeof(TestBuffer)/4;
	uint32_t myCRCValue1=0xffffffff;
	int i=0;
	uint32_t uwCRCValue=HAL_CRC_Calculate(hcrc, TestBuffer,  sizeof(TestBuffer)/4);
	printf("hardware crc= %x\n",uwCRCValue);
	printf("soft crc slow=%x\n", Crc32SlowBlock(0xFFFFFFFF, sizeof(TestBuffer)/sizeof(uint32_t), TestBuffer) );
	printf("soft crc fast=%x\n", Crc32FastBlock(0xFFFFFFFF, sizeof(TestBuffer)/sizeof(uint32_t), TestBuffer) );

}
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
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	 ALLCRCTypeTest(&hcrc);
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
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

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
