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
#include "device_com.h"
#include "base_station.h"

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void TestSend(SPI_HandleTypeDef* spi_instance);
void TestReceive(SPI_HandleTypeDef* spi_instance);

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
  if (!MX_SPI1_Init()) {
		transmit_string("Failed to initialize hspi1\n\r");
		Error_Handler();
	}
	/*
	if (!MX_SPI2_Init()) {
		transmit_string("Failed to initialize hspi2\n\r");
		Error_Handler(); 
	}*/
	
	uart_init(115200);
	tim3_init(500, 1);
	
	if (!dw_init(&hspi1)) {
		transmit_string("Failed to initialize device on hspi1\n\r");
		Error_Handler();
	}
	
	base_station_main(1);
//	
//	if (!dw_init(&hspi2)) {
//		transmit_string("Failed to initialize device on hspi2\n\r");
//		Error_Handler();
//	}
//	
//	TestReceive(&hspi2);
//	HAL_Delay(1000);
//	uint16_t count = 0;
//	while(1)
//	{
//		// SYS_STATUS: GET 0x0F:01
//		uint8_t len = SetupSendBuff(0,0x0F,0x01);
//		SPISend(&hspi2, len+2);
//		/*
//		if(receivebuff[len] & (1))
//		{
//			transmit_string("Preamble");
//		}
//		if(receivebuff[len] & (1<<1))
//		{
//			transmit_string("SFD");
//		}
//		if(receivebuff[len] & (1<<2))
//		{
//			transmit_string("LDE");
//		}
//		if(receivebuff[len] & (1<<3))
//		{
//			transmit_string("PHY");
//		}
//		*/
//		if(receivebuff[len] & (1<<4))
//		{
//			transmit_string("PHYError");
//			count++;
//			if (count > 10) {
//				break;
//			}
//		}
//		if(receivebuff[len] & (1<<5))
//		{
//			// transmit_string("FrameDone");
//			break;
//		}
//		/*
//		if(receivebuff[len] & (1<<6))
//		{
//			transmit_string("FCS");
//		}*/
//		if(receivebuff[len] & (1<<7))
//		{
//			transmit_string("FCSError");
//		}
//		if(receivebuff[len+1] & (1))
//		{
//			transmit_string("FrameSyncLoss");
//		}
//		if(receivebuff[len+1] & (1<<1))
//		{
//			transmit_string("FrameTimeout");
//		}
//		if(receivebuff[len+1] & (1<<2))
//		{
//			transmit_string("LeadingEdgeDetectionError");
//		}
//		if(receivebuff[len+1] & (1<<5))
//		{
//			transmit_string("PreambleDetectionTimeout");
//		}
//		// transmit_string("Waiting\n\r");
//		HAL_Delay(333);
//	}
//	transmit_string("We Got Something!\n\r");
//	
//	// SYS_STATUS: GET 0x11:01
//	uint8_t len = SetupSendBuff(0,0x11,0);
//	SPISend(&hspi2, len+13);
//	
//	uint8_t stringrec [14] = {0};
//	ReceiveAt(len, stringrec, 13);
//	transmit_string((char*)stringrec);
//	transmit_string("\n\r");

  while (1)
  {
  }
}

void TestSend(SPI_HandleTypeDef* spi_instance) {
	// set transmit data buffer
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
	uint32_t config = (0x2) | (0x01 << 14) | (0x01 << 15) | (0x01 << 18) | (0x01 << 16);
	uint8_t len = SetupSendBuff(1, 0x08, 0);
	Send32At(len, config);
	SPISend(spi_instance, len + 4);
	
	// Set the transmit start bit in the System Control Register
	// SYS_CTRL[0] = 0x02
	len = SetupSendBuff(1, 0x0D, 0);
	Send32At(len, (0x01 << 1) | (0x01 << 7));
	SPISend(spi_instance, len + 1);
	
	transmit_string("Sending initial message!\n\r");
}

void TestReceive(SPI_HandleTypeDef* spi_instance) {	
	// SYS_CTRL: 0x0D:01 -> 0x01
	uint8_t len = SetupSendBuff(1,0x0D,0x01);
	Send32At(len, 0x01);
	SPISend(spi_instance, len+1);
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

void TIM3_IRQHandler() {
	// do thing
	static uint16_t state = 0;
	
	if (state < 11) {
		state++;
	}
	if (state == 10) {
		transmit_string("Starting send\n\r");
		TestSend(&hspi1);
	} else if (state == 11) {
		transmit_string("Checking status\n\r");
		uint8_t len = SetupSendBuff(0, 0x0F, 0x04);
		SPISend(&hspi1, 2);
		uint16_t status = 0;
		ReceiveAt(len, (uint8_t *) &status, 2);
		if (status & (0x01 << 13)) {
			transmit_string("Data received!\n\r");
			state = 12;
			
			// get that data
			len = SetupSendBuff(0, 0x11, 0x0);
			SPISend(&hspi1, len + 4);
			uint8_t delay[4];
			ReceiveAt(len, delay, 4);
			
			uint32_t delay_num = delay[0] << 24 | delay[1] << 16 | delay[2] << 8 | delay[3];
			char str[11];
			str[0] = '0';
			str[1] = 'x';
			to_ascii(delay_num, &(str[2]));
			str[10] = 0;
			transmit_string(str);
			transmit_string("\n\r");
			
			len = SetupSendBuff(0, 0x15, 0x0);
			SPISend(&hspi1, len + 5);
			uint8_t rx_time[5];
			ReceiveAt(len, rx_time, 5);
			uint64_t rx_num = rx_time[0];
			rx_num = rx_num << 32;
			rx_num |= rx_time[1] << 24 | rx_time[2] << 16 | rx_time[3] << 8 | rx_time[4];
			
			len = SetupSendBuff(0, 0x17, 0x0);
			SPISend(&hspi1, len + 5);
			uint8_t tx_time[5];
			ReceiveAt(len, tx_time, 5);
			uint64_t tx_num = tx_time[0];
			tx_num = tx_num << 32;
			tx_num |= tx_time[1] << 24 | tx_time[2] << 16 | tx_time[3] << 8 | tx_time[4];
			
			uint32_t tof = (rx_num - tx_num - delay_num) / 2;
			to_ascii(tof, &(str[2]));
			str[10] = 0;
			transmit_string(str);
			transmit_string("\n\r");
		} 
	}
	// transmit_string("timer time\n\r")
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
	__HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  // GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	/*Configure GPIO pin: PB12 */
	GPIO_InitStruct.Pin = GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
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
