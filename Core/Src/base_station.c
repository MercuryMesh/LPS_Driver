#include "stm32f0xx_hal.h"
#include "device_com.h"

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