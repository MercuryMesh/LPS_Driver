#include "stm32f0xx_hal.h"
#include "device_com.h"
#include "uart.h"

void begin_receiving(void);
uint8_t await_receive(void);
void get_system_time(uint8_t* response);

SPI_HandleTypeDef spi;

uint8_t base_station_main(void) {
	if (!MX_SPI2_Init() || !dw_init(&hspi2)) {
		return 1;
	}
	spi = hspi2;
	begin_receiving();
	
	if (await_receive()) {
		// schedule the retransmission
		uint8_t sys_time[5];
		get_system_time(sys_time);
		
		// calculate the send time with a 1ms delay
		((uint32_t*) sys_time)[0] += 0x03D090;
		
	}
}

void get_system_time(uint8_t* response) {
	uint8_t len = SetupSendBuff(0, 0x06, 0x00);
	SPISend(&spi, len + 5);
	ReceiveAt(len, response, 5);
}

void begin_receiving(void) {
	// SYS_CTRL: 0x0D:01 -> 0x01
	uint8_t len = SetupSendBuff(1,0x0D,0x01);
	Send32At(len, 0x01);
	SPISend(&spi, len+1);
}

uint8_t await_receive(void) {
	uint8_t count = 0;
	while(1)
	{
		uint8_t len = SetupSendBuff(0,0x0F,0x01);
		SPISend(&spi, len+2);
		if(
			receivebuff[len] & (1<<4)			||
			receivebuff[len] & (1<<7) 		||
			receivebuff[len + 1] & 1 			||
			receivebuff[len + 1] & (1<<1) ||
			receivebuff[len + 1] & (1<<2) ||
			receivebuff[len + 1] & (1<<5)
		)
		{
			count++;
			if (count > 10) {
				return 0;
			}
		}
		if(receivebuff[len] & (1<<5))
		{
			return 1;
		}
		HAL_Delay(200);
	}
}

/**
  * @brief IRQ handler for SPI1 (via EXTI line 1, PC1)
	* (receive data frame ready).
  */
void EXTI0_1_IRQHandler(void)
{
	transmit_string("We Got Something on 1!\n\r");
	
//	// SYS_STATUS: GET 0x11:01
//	uint8_t len = SetupSendBuff(0,0x11,0);
//	SPISend(&hspi1, len+13);
//	
//	uint8_t stringrec [14] = {0};
//	ReceiveAt(len, stringrec, 13);
//	transmit_string((char*)stringrec);
//	transmit_string("\n\r");
	
	EXTI->PR |= (1 << 1);
}

/**
  * @brief IRQ handler for SPI1 (via EXTI line 1, PC1)
	* (receive data frame ready).
  */
void EXTI2_3_IRQHandler(void)
{
	transmit_string("We Got Something on 2!\n\r");
	
//	// SYS_STATUS: GET 0x11:01
//	uint8_t len = SetupSendBuff(0,0x11,0);
//	SPISend(&hspi2, len+13);
//	
//	uint8_t stringrec [14] = {0};
//	ReceiveAt(len, stringrec, 13);
//	transmit_string((char*)stringrec);
//	transmit_string("\n\r");
	
	EXTI->PR |= (1 << 2);
}