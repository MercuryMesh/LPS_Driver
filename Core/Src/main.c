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
#include "uart.h"
#include "timer.h"


/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

uint8_t sendbuff [128];
uint8_t receivebuff [128];


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
uint8_t SetupSendBuff(uint8_t send, uint8_t reg, uint16_t subreg);
void SPISend(uint8_t bytes);
void Send32At(uint8_t position, uint32_t bytes);
void SendAt(uint8_t position, uint8_t* bytes, uint8_t length);
void ReceiveAt(uint8_t position, uint8_t* write, uint8_t len);
void TestSend(void);
void TestReceive(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();


  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
	uart_init(115200);
	tim3_init(500, 1);

	
	uint8_t len = SetupSendBuff(0,0,0);
	SPISend(len+4);
	
	uint32_t val = (receivebuff[len]) | (receivebuff[len+1] << 8) | (receivebuff[len+2] << 16) | (receivebuff[len+3] << 24);
	
	if (val == 0xDECA0130) {
		// SUCCESS. Light green LED
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
        	transmit_string("looks good! Love that for you\n\r");
	} else {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
	}
	
	// TX_POWER: 0x1E -> 0x0E082848
	len = SetupSendBuff(1,0x1E,0);
	Send32At(len, 0x0E082848);
	SPISend(len+4);
	
	// set channels to 5 and set PCODES to 3
	// enable RXPRF
	len = SetupSendBuff(1, 0x1F, 0x00);
	Send32At(len, (0x05) | (0x05 << 4) | (0x03 << 27) | (0x03 << 22) | (0x01 << 18));
	SPISend(len + 4);
	
	//DWM "Default Configurations that should be modified"
	// AGC_TUNE1: 0x23:04 -> 0x8870;
	len = SetupSendBuff(1,0x23,0x04);
	Send32At(len, 0x8870);
	SPISend(len+2);
	
	// AGC_TUNE2: 0x23:0C -> 0x2502A907
	len = SetupSendBuff(1,0x23,0x0C);
	Send32At(len, 0x2502A907);
	SPISend(len+4);
	
	// AGC_TUNE3: 0x23:12 -> 0x0035
	len = SetupSendBuff(1,0x23,0x12);
	Send32At(len, 0x0035);
	SPISend(len+2);
	
	// DRX_TUNE0B: 0x27:02 -> 0x0001
	len = SetupSendBuff(1, 0x27, 0x02);
	Send32At(len, 0x0001);
	SPISend(len + 2);
	
	// DRX_TUNE1a: 0x27:04 -> 0x0087
	len = SetupSendBuff(1, 0x27, 0x04);
	Send32At(len, 0x0087);
	SPISend(len + 2);
	
	// DRX_TUNE1b: 0x27:06 -> 0x0010
	len = SetupSendBuff(1, 0x27, 0x06);
	Send32At(len, 0x0010);
	SPISend(len + 2);
	
	// DRX_TUNE2: 0x27:08 -> 0x311A002D
	len = SetupSendBuff(1,0x27,0x08);
	Send32At(len, 0x311A002D);
	SPISend(len+4);
	
	// DRX_TUNE4H: 0x27:26 -> 0x0010
	len = SetupSendBuff(1, 0x27, 0x26);
	Send32At(len, 0x0010);
	SPISend(len + 2);
	
	// RF_RXCTRLH: 0x28:0B -> 0xD8
	len = SetupSendBuff(1, 0x28, 0x0B);
	Send32At(len, 0xD8);
	SPISend(len + 1);
	
	// RF_TXCTRL: 0x28:0C -> 0x1E3FE3
	len = SetupSendBuff(1,0x28,0x0C);
	Send32At(len, 0x1E3FE3);
	SPISend(len+3);
	
	// TC_PGDELAY: 0x2A:0B -> 0xB5
	len = SetupSendBuff(1,0x2A,0x0B);
	Send32At(len, 0xB5);
	SPISend(len+1);
	
	// FS_PLLCFG -> 0x0800041D
	len = SetupSendBuff(1, 0x2B, 0x07);
	Send32At(len, 0x0800041D);
	SPISend(len + 4);
	
	// FS_PLLTUNE: 0x2B:0B -> 0xBE
	len = SetupSendBuff(1,0x2B,0x0B);
	Send32At(len, 0xBE);
	SPISend(len+1);
	
	// LDE_CFG2: 0x2E:1806 -> 0x1607;
	len = SetupSendBuff(1,0x2E,0x1806);
	Send32At(len, 0x1607);
	SPISend(len+2);
	
	// LDE_REPC: 0x2E:2804 -> 0x51EA
	len = SetupSendBuff(1, 0x2E, 0x2804);
	Send32At(len, 0x51EA);
	SPISend(len + 2);
	
	
	// LDELOAD
	// PMSC_CTRL0: 0x36:00 -> 0x0301
	len = SetupSendBuff(1,0x36,0);
	Send32At(len, 0x0301);
	SPISend(len+2);
	
	// OTP_CTRL: 0x2D:06 -> 0x8000
	len = SetupSendBuff(1,0x2D,0x06);
	Send32At(len, 0x8000);
	SPISend(len+2);
	
	HAL_Delay(1);
	
	// PMSC_CTRL0: 0x36:00 -> 0x0200
	len = SetupSendBuff(1,0x36,0);
	Send32At(len, 0x0200);
	SPISend(len+2);
	

  while (1)
  {
  }
}

void TestSend(void) {
	// set transmit data buffer
	// TX_BUFFER: 0x09:00 -> 'Hello, World!'
	uint8_t len = SetupSendBuff(1, 0x09, 0);
	char *d = "Hello, World!";
	SendAt(len, (uint8_t *) d, sizeof(d));
	SPISend(len + sizeof(d));
	
	
	// set transmit frame control
  // TFLEN = sizeof(d) + 2
	// TFLE = 0
	// R = 0
	// TXBR = 10
	// TR = 1
	// TXPRF = 01
	// TXPSR = 01
	// PE = 00
	// TXBOFFS = 0
	uint32_t config = (0xF) | (0x02 << 13) | (0x01 << 15) | (0x01 << 16) | (0x01 << 18);
	len = SetupSendBuff(1, 0x08, 0);
	SendAt(len, (uint8_t *) &config, 4);
	SPISend(len + 4);
	
	// Set the transmit start bit in the System Control Register
	// SYS_CTRL[0] = 0x02
	len = SetupSendBuff(1, 0x0D, 0);
	uint8_t c = (0x01 << 1);
	SendAt(len, &c, 1);
	SPISend(len + 1);
	
	len = SetupSendBuff(0, 0x0F, 0);
	while (1) {
		// HAL_Delay(100);
		SPISend(len + 1);
		
		uint8_t status = receivebuff[len];
		if (status & (0x01 << 4)) {
			transmit_string("transmit frame begins\n\r");
		}
		if (status & (0x01 << 5)) {
			transmit_string("transmit preamble sent\n\r");
		}
		if (status & (0x01 << 6)) {
			transmit_string("transmit PHY header sent\n\r");
		}
		if (status & (0x01 << 7)) {
			transmit_string("transmission complete!\n\r");
			break;
		} else {
			transmit_string("not done yet\n\r");
		}
	}
}

void TestReceive(void) {
	HAL_Delay(200);
	
	// SYS_CTRL: 0x0D:01 -> 0x01
	uint8_t len = SetupSendBuff(1,0x0D,0x01);
	Send32At(len, 0x01);
	SPISend(len+1);
	
	// SYS_STATUS: GET 0x0F:01
	len = SetupSendBuff(0,0x0F,0x01);
	while(1)
	{
		HAL_Delay(100);
		SPISend(len+2);
		if(receivebuff[len] & (1))
		{
			transmit_string("Preamble");
		}
		if(receivebuff[len] & (1<<1))
		{
			transmit_string("SFD");
		}
		if(receivebuff[len] & (1<<2))
		{
			transmit_string("LDE");
		}
		if(receivebuff[len] & (1<<3))
		{
			transmit_string("PHY");
		}
		if(receivebuff[len] & (1<<4))
		{
			transmit_string("PHYError");
		}
		if(receivebuff[len] & (1<<5))
		{
			transmit_string("FrameDone");
			break;
		}
		if(receivebuff[len] & (1<<6))
		{
			transmit_string("FCS");
		}
		if(receivebuff[len] & (1<<7))
		{
			transmit_string("FCSError");
		}
		if(receivebuff[len+1] & (1))
		{
			transmit_string("FrameSyncLoss");
		}
		if(receivebuff[len+1] & (1<<1))
		{
			transmit_string("FrameTimeout");
		}
		if(receivebuff[len+1] & (1<<2))
		{
			transmit_string("LeadingEdgeDetectionError");
		}
		if(receivebuff[len+1] & (1<<5))
		{
			transmit_string("PreambleDetectionTimeout");
		}
		transmit_string("Waiting\n\r");
	}
	transmit_string("We Got Something!\n\r");
	
	// SYS_STATUS: GET 0x11:01
	len = SetupSendBuff(0,0x11,0);
	SPISend(len+13);
	
	uint8_t stringrec [14] = {0};
	ReceiveAt(len, stringrec, 13);
	transmit_string((char*)stringrec);
	transmit_string("\n\r");
}

uint8_t SetupSendBuff(uint8_t send, uint8_t reg, uint16_t subreg)
{
	sendbuff[0] = reg;
	if (send)
	{
		sendbuff[0] |= (1<<7);
	}
	if (subreg)
	{
		sendbuff[0] |= (1<<6);
		sendbuff[1] = subreg & (0x007F);
		if (subreg > 127)
		{
			sendbuff[1] |= (1<<7);
			sendbuff[2] = (subreg >> 7);
			return 3;
		}
		return 2;
	}
	return 1;
}

void SPISend(uint8_t bytes)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1, sendbuff, receivebuff, bytes, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

void Send32At(uint8_t position, uint32_t bytes)
{
	sendbuff[position] = bytes;
	bytes >>= 8;
	sendbuff[position+1] = bytes;
	bytes >>= 8;
	sendbuff[position+2] = bytes;
	bytes >>= 8;
	sendbuff[position+3] = bytes;
	bytes >>= 8;
}

void SendAt(uint8_t position, uint8_t* bytes, uint8_t length) {
	for (int i = 0; i < length; i++) {
		sendbuff[position + i] = bytes[length - 1 - i];
	}
}

void ReceiveAt(uint8_t position, uint8_t* write, uint8_t len)
{
	for (int i = 0; i < len; i++) {
		write[len - 1 - i] = receivebuff[position + i];
	}
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

void TIM3_IRQHandler() {
	// do thing
	// transmit_string("timer time\n\r");
	TIM3->SR &= ~(0x01);
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
	__HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  // GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	
	GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
