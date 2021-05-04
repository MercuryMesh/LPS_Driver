#include "device_com.h"

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

uint8_t sendbuff [128];
uint8_t receivebuff [128];

uint8_t dw_init(SPI_HandleTypeDef* spi_instance) {
	uint8_t len = SetupSendBuff(0,0,0);
	SPISend(spi_instance, len+4);
	
	uint32_t val = (receivebuff[len]) | (receivebuff[len+1] << 8) | (receivebuff[len+2] << 16) | (receivebuff[len+3] << 24);
	
	if (val == 0xDECA0130) {
		// SUCCESS. Light green LED
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
	} else {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
		return 0;
	}
	
	len = SetupSendBuff(0x01, 0x2C, 0x00);
	Send32At(len, (0x01 << 11));
	SPISend(spi_instance, len + 2);
	
	// trigger a soft reset
	len = SetupSendBuff(0, 0x36, 0x00);
	SPISend(spi_instance, len + 1);
	
	// force clock to 19.2 MHz
	uint8_t temp = receivebuff[len];
	temp |= (0x01);
	temp &= ~(0x01 << 1);
	len = SetupSendBuff(1, 0x36, 0x00);
	Send32At(len, temp);
	SPISend(spi_instance, len + 1);
	
	// set all soft reset bits to 0000
	len = SetupSendBuff(1, 0x36, 0x03);
	Send32At(len, ~0xFF);
	SPISend(spi_instance, len + 1);
	
	HAL_Delay(10);
	
	// set all soft reset bits back to 1111
	len = SetupSendBuff(1, 0x36, 0x03);
	Send32At(len, 0xF0);
	SPISend(spi_instance, len + 1);
	
	// put clock back in auto mode
	temp &= ~((0x01 << 1) | 0x01);
	len = SetupSendBuff(1, 0x36, 0x00);
	Send32At(len, temp);
	SPISend(spi_instance, len + 1);
	
	// soft reset done
	
	HAL_Delay(100);
	
	// SYS_CFG: 0x04:3 -> 0x20 (Recieve Auto-Reenable (on frame failure))
	len = SetupSendBuff(1,0x04,0x3);
	Send32At(len, 0x20);
	SPISend(spi_instance, len+1);
	
	// SYS_MASK: 0x0E:1 -> 0x20 (Interrupt on Data Frame Ready)
	len = SetupSendBuff(1,0x0E,0x0);
	Send32At(len, 0x01 << 13);
	SPISend(spi_instance, len+2);
	
	// TX_POWER: 0x1E -> 0x15355575
	len = SetupSendBuff(1,0x1E,0);
	Send32At(len, 0x15355575);
	SPISend(spi_instance, len+4);
	
	// set channels to 1 and set PCODES to 1
	// enable RXPRF
	len = SetupSendBuff(1, 0x1F, 0x00);
	Send32At(len, (0x01) | (0x01 << 4) | (0x01 << 27) | (0x01 << 22) | (0x01 << 18));
	SPISend(spi_instance, len + 4);
	
	// AGC_TUNE1: 0x23:04 -> 0x8870;
	len = SetupSendBuff(1,0x23,0x04);
	Send32At(len, 0x8870);
	SPISend(spi_instance, len+2);
	
	// AGC_TUNE2: 0x23:0C -> 0x2502A907
	len = SetupSendBuff(1,0x23,0x0C);
	Send32At(len, 0x2502A907);
	SPISend(spi_instance, len+4);
	
	// AGC_TUNE3: 0x23:12 -> 0x0035
	len = SetupSendBuff(1,0x23,0x12);
	Send32At(len, 0x0035);
	SPISend(spi_instance, len+2);
	
	// GPIO_DIR: 0x26:08 -> (1<<20)
	len = SetupSendBuff(1,0x26,0x08);
	Send32At(len, (1 << 20));
	SPISend(spi_instance, len+3);
	
	// DRX_TUNE0B: 0x27:02 -> 0x0001
	len = SetupSendBuff(1, 0x27, 0x02);
	Send32At(len, 0x0001);
	SPISend(spi_instance, len + 2);
	
	// DRX_TUNE1a: 0x27:04 -> 0x0087
	len = SetupSendBuff(1, 0x27, 0x04);
	Send32At(len, 0x0087);
	SPISend(spi_instance, len + 2);
	
	// DRX_TUNE1b: 0x27:06 -> 0x0010
	len = SetupSendBuff(1, 0x27, 0x06);
	Send32At(len, 0x0010);
	SPISend(spi_instance, len + 2);
	
	// DRX_TUNE2: 0x27:08 -> 0x311A002D
	len = SetupSendBuff(1,0x27,0x08);
	Send32At(len, 0x311A002D);
	SPISend(spi_instance, len+4);
	
	// DRX_TUNE4H: 0x27:26 -> 0x0010
	len = SetupSendBuff(1, 0x27, 0x26);
	Send32At(len, 0x0010);
	SPISend(spi_instance, len + 2);
	
	// RF_RXCTRLH: 0x28:0B -> 0xD8
	len = SetupSendBuff(1, 0x28, 0x0B);
	Send32At(len, 0xD8);
	SPISend(spi_instance, len + 1);
	
	// RF_TXCTRL: 0x28:0C -> 0x00005C40
	len = SetupSendBuff(1,0x28,0x0C);
	Send32At(len, 0x00005C40);
	SPISend(spi_instance, len+3);
	
	// TC_PGDELAY: 0x2A:0B -> 0xC9
	len = SetupSendBuff(1,0x2A,0x0B);
	Send32At(len, 0xC9);
	SPISend(spi_instance, len+1);
	
	// FS_PLLCFG -> 0x09000407
	len = SetupSendBuff(1, 0x2B, 0x07);
	Send32At(len, 0x09000407);
	SPISend(spi_instance, len + 4);
	
	// FS_PLLTUNE: 0x2B:0B -> 0x1E
	len = SetupSendBuff(1,0x2B,0x0B);
	Send32At(len, 0x1E);
	SPISend(spi_instance, len+1);
	
	// LDE_CFG2: 0x2E:1806 -> 0x1607;
	len = SetupSendBuff(1,0x2E,0x1806);
	Send32At(len, 0x1607);
	SPISend(spi_instance, len+2);
	
	// LDE_REPC: 0x2E:2804 -> 0x5998
	len = SetupSendBuff(1, 0x2E, 0x2804);
	Send32At(len, 0x5998);
	SPISend(spi_instance, len + 2);
	
	// LDELOAD
	// PMSC_CTRL0: 0x36:00 -> 0x0301
	len = SetupSendBuff(1,0x36,0);
	Send32At(len, 0x0301);
	SPISend(spi_instance, len+2);
	
	// OTP_CTRL: 0x2D:06 -> 0x8000
	len = SetupSendBuff(1,0x2D,0x06);
	Send32At(len, 0x8000);
	SPISend(spi_instance, len+2);
	
	HAL_Delay(1);
	
	// PMSC_CTRL0: 0x36:00 -> 0x0200
	len = SetupSendBuff(1,0x36,0);
	Send32At(len, 0x0200);
	SPISend(spi_instance, len+2);
	return 1;
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
uint8_t MX_SPI1_Init(void)
{
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    return 0;
  }
	
	return 1;

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
uint8_t MX_SPI2_Init(void)
{
  /* SPI1 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    return 0;
  }
	return 1;
}

void SPISend(SPI_HandleTypeDef* instance, uint8_t bytes)
{
	if (instance->Instance == SPI1) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	} else {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	}
	HAL_SPI_TransmitReceive(instance, sendbuff, receivebuff, bytes, HAL_MAX_DELAY);
	if (instance->Instance == SPI1) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	}
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
