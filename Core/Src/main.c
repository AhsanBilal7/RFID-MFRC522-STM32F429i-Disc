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
#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "rc522.h"
#include "Motor.h"

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
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart1;
/* USER CODE BEGIN PV */
static void MX_USART1_UART_Init(void);
/* USER CODE END PV */

// variable RFID
uint8_t dataUDP_permit[]="Ma the hop le ";
uint8_t dataUDP_deny[]="Ma the khong hop le ";
int  CardID[5];
int MyCardSerNum[];
unsigned char MyID1[5] =  {0x96, 0x46, 0x38, 0x12, 0xfa};	//My card 1on my keys
unsigned char MyID2[5] =  {0x88, 0x04, 0xfa, 0xef, 0x99};	//My card 2on my keys
unsigned char MyID3[5] =  {0xea, 0x2b, 0x26, 0xa3, 0x44};	//My card 3on my keys
unsigned char MyID[5] =  {0x3b, 0x48, 0x36, 0xcc, 0x89};	//My card 4on my keys


// RC522
uint8_t MFRC522_Check(uint8_t* id);
uint8_t MFRC522_Compare(uint8_t* CardID, uint8_t* CompareID);
void MFRC522_WriteRegister(uint8_t addr, uint8_t val);
uint8_t MFRC522_ReadRegister(uint8_t addr);
void MFRC522_SetBitMask(uint8_t reg, uint8_t mask);
void MFRC522_ClearBitMask(uint8_t reg, uint8_t mask);
uint8_t MFRC522_Request(uint8_t reqMode, uint8_t* TagType);
uint8_t MFRC522_ToCard(uint8_t command, uint8_t* sendData, uint8_t sendLen, uint8_t* backData, uint16_t* backLen);
uint8_t MFRC522_Anticoll(uint8_t* serNum);
void MFRC522_CalulateCRC(uint8_t* pIndata, uint8_t len, uint8_t* pOutData);
uint8_t MFRC522_SelectTag(uint8_t* serNum);
uint8_t MFRC522_Auth(uint8_t authMode, uint8_t BlockAddr, uint8_t* Sectorkey, uint8_t* serNum);
uint8_t MFRC522_Read(uint8_t blockAddr, uint8_t* recvData);
uint8_t MFRC522_Write(uint8_t blockAddr, uint8_t* writeData);
void MFRC522_Init(void);
void MFRC522_Reset(void);
void MFRC522_AntennaOn(void);
void MFRC522_AntennaOff(void);
void MFRC522_Halt(void);

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  MX_USART1_UART_Init();
  /* USER CODE END 2 */
char msg[1] ;
msg[0] = 'c' ;
char string[8];
  MFRC522_Init();
uint8_t test[] = "Hello! This is Ahsan Bilal\r\n";
  uint8_t newline[] = "\n";

	HAL_Delay(1000);
	// while(1) { HAL_UART_Transmit(&huart1, &msg, 1, 1000);
//	HAL_Delay(1000);}
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	    {
//		MyCardID=MFRC522_Check(CardID);
		  HAL_GPIO_WritePin(GPIOG,GPIO_PIN_13,GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOG,GPIO_PIN_14,GPIO_PIN_RESET);
	//	 HAL_UART_Transmit(&huart1,&CardID, sizeof(CardID), 1000);
		 // if (MFRC522_Check(CardID) == MI_OK);
		//  HAL_UART_Transmit(&huart1,&CardID, sizeof(CardID), 1000);
		  if(MFRC522_Check(CardID)== MI_OK) {
			//  MFRC522_SelectTag(MyCardSerNum);
			  HAL_GPIO_WritePin(GPIOG,GPIO_PIN_14,GPIO_PIN_SET);
			//  CardID = CardID%16;
			  int newstring[9];
			  itoa (CardID[0] , string , 16);
			  // The UID is sored in the firt index of the card number and it is a 4 bit number
			  // We used the Itoa function to convert  the 4 byte integer to 4 byte a hexadecimal number
			  HAL_UART_Transmit(&huart1,&string[0], sizeof(string[0]), 1000);
			  HAL_UART_Transmit(&huart1,&string[1], sizeof(string[1]), 1000);
			  HAL_UART_Transmit(&huart1,&string[2], sizeof(string[1]), 1000);
			  HAL_UART_Transmit(&huart1,&string[3], sizeof(string[1]), 1000);
			  HAL_UART_Transmit(&huart1,&string[4], sizeof(string[1]), 1000);
			  HAL_UART_Transmit(&huart1,&string[5], sizeof(string[1]), 1000);
			  HAL_UART_Transmit(&huart1,&string[6], sizeof(string[1]), 1000);
			  HAL_UART_Transmit(&huart1,&string[7], sizeof(string[1]), 1000);
			  HAL_UART_Transmit(&huart1,&string[8], sizeof(string[1]), 1000);
//			  HAL_UART_Transmit(&huart1,&string[10], sizeof(string[1]), 1000);

//			  HAL_UART_Transmit(&huart1,&string, sizeof(string), 1000);

		 //     HAL_UART_Transmit(&huart1,&CardID[0], sizeof(CardID[0]), 1000);
			  HAL_UART_Transmit(&huart1, &newline, sizeof(newline), 1000);

/*print
		  HAL_UART_Transmit(&huart1,&CardID[0], 1, 1000);
		  HAL_UART_Transmit(&huart1, &newline, sizeof(newline), 1000);
		  HAL_UART_Transmit(&huart1,&CardID[1], 1, 1000);
	  HAL_UART_Transmit(&huart1, &newline, sizeof(newline), 1000);
		  HAL_UART_Transmit(&huart1,&CardID[2], 1, 1000);
          HAL_UART_Transmit(&huart1, &newline, sizeof(newline), 1000);
		  HAL_UART_Transmit(&huart1,&CardID[3], 1, 1000);
		  HAL_UART_Transmit(&huart1, &newline, sizeof(newline), 1000);
		  HAL_UART_Transmit(&huart1,&CardID[4], 1, 1000);
		  HAL_UART_Transmit(&huart1, &newline, sizeof(newline), 1000);
		  HAL_UART_Transmit(&huart1, &test, sizeof(test), 1000);
	  HAL_UART_Transmit(&huart1, &newline, sizeof(newline), 1000);

	  HAL_UART_Transmit(&huart1, &newline, sizeof(newline), 1000);
*/
			             // you do need space as I see from your example
			      }
			 //     free(keyHex);

}
		  //  HAL_Delay(1000);
	//	if(MFRC522_Check(CardID)== MI_OK) { HAL_UART_Transmit(&huart1, &CardID, sizeof(CardID), 1000);}
		   //       string[6] = '\0';
			//	  strncpy(string, MyID1, 6);
	///	  HAL_UART_Transmit(&huart1, &string, sizeof(string), 1000);
	///	  HAL_UART_Transmit(&huart1, &newline, sizeof(newline), 1000);

		//  HAL_Delay(1000);
	    /* USER CODE END WHILE */

	    /* USER CODE BEGIN 3 */
	/*  		if (MFRC522_Check(CardID) == MI_OK)
	  			{

	  			if (MFRC522_Compare(CardID, MyID) == MI_OK)
	  				{
	  			  HAL_GPIO_WritePin(GPIOG,GPIO_PIN_13,GPIO_PIN_RESET);
	  			  HAL_GPIO_WritePin(GPIOG,GPIO_PIN_14,GPIO_PIN_SET);

	  				Motor_move();
	  				}
	  				else if(MFRC522_Compare(CardID, MyID1) == MI_OK)
	  				{
	  	  			  HAL_GPIO_WritePin(GPIOG,GPIO_PIN_13,GPIO_PIN_RESET);
	  	  			  HAL_GPIO_WritePin(GPIOG,GPIO_PIN_14,GPIO_PIN_SET);
		  				Motor_move();

	  				}
	  				else if(MFRC522_Compare(CardID, MyID2) == MI_OK)
					{
					  HAL_GPIO_WritePin(GPIOG,GPIO_PIN_13,GPIO_PIN_RESET);
					  HAL_GPIO_WritePin(GPIOG,GPIO_PIN_14,GPIO_PIN_SET);
		  				Motor_move();

					}
	  				else if(MFRC522_Compare(CardID, MyID3) == MI_OK)
					{
					  HAL_GPIO_WritePin(GPIOG,GPIO_PIN_13,GPIO_PIN_RESET);
					  HAL_GPIO_WritePin(GPIOG,GPIO_PIN_14,GPIO_PIN_SET);
		  				Motor_move();

					}

				} */

	  		  HAL_Delay(10);

	    /* USER CODE END 3 */
	  		}
	//  }

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
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
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

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();



  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_3, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_3;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

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
