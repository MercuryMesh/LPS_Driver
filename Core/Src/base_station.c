#include "stm32f0xx_hal.h"
#include "device_com.h"
#include "uart.h"

void begin_receiving(void);
uint8_t await_receive(void);
void get_recv_time(uint8_t* response);
void to_ascii(uint32_t data, char* str);

SPI_HandleTypeDef spi;

void base_station_main(uint8_t initialize) {
	if (initialize && (!MX_SPI2_Init() || !dw_init(&hspi2))) {
		return;
	}
	spi = hspi2;
	begin_receiving();
	
	if (await_receive()) {
		transmit_string("got\n\r");
		// schedule the retransmission
		uint8_t sys_time[5];
		get_recv_time(sys_time);
		
		// calculate the send time with a 1ms delay
		uint32_t delay_time = sys_time[0] << 24 | sys_time[1] << 16 | sys_time[2] << 8 | sys_time[3];
		
		char str[11];
		str[0] = '0';
		str[1] = 'x';
		to_ascii(delay_time, &(str[2]));
		str[10] = 0;
		transmit_string(str);
		transmit_string("\n\r");
		
		uint32_t send_time = delay_time;
		send_time += 0x03D090;
		
		delay_time = (send_time-delay_time) << 8;
		
		to_ascii(send_time, &(str[2]));
		str[10] = 0;
		transmit_string(str);
		transmit_string("\n\r");
		
		to_ascii(delay_time, &(str[2]));
		str[10] = 0;
		transmit_string(str);
		transmit_string("\n\r");
		
		// Load the time to transmit
		uint8_t len = SetupSendBuff(1, 0x0A, 0x00);
		sendbuff[len] = 0;
		sendbuff[len+1] = send_time;
		sendbuff[len+2] = (send_time >> 8);
		sendbuff[len+3] = (send_time >> 16);
		sendbuff[len+4] = (send_time >> 24);
		SPISend(&spi, len + 5);
		
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
		uint32_t config = (0x6) | (0x01 << 14) | (0x01 << 15) | (0x01 << 18) | (0x01 << 16);
		len = SetupSendBuff(1, 0x08, 0);
		Send32At(len, config);
		SPISend(&spi, len + 4);
				
		// TX_BUFFER: 0x09:00 -> calculated delay time
		len = SetupSendBuff(1, 0x09, 0x00);
		sendbuff[len] = delay_time;
		sendbuff[len+1] = (delay_time >> 8);
		sendbuff[len+2] = (delay_time >> 16);
		sendbuff[len+3] = (delay_time >> 24);
		SPISend(&spi, len + 4);
		
		// schedule it
		len = SetupSendBuff(1, 0x0D, 0x00);
		Send32At(len, (0x01 << 2) | (0x01 << 1));
		SPISend(&spi, len + 1);

		transmit_string("setup transmit\n\r");
	}
}

static void get_recv_time(uint8_t* response) {
	uint8_t len = SetupSendBuff(0, 0x15, 0x00);
	SPISend(&spi, len + 5);
	ReceiveAt(len, response, 5);
}

static void begin_receiving(void) {
	// SYS_CTRL: 0x0D:01 -> 0x01
	uint8_t len = SetupSendBuff(1,0x0D,0x01);
	Send32At(len, 0x01);
	SPISend(&spi, len+1);
}

static uint8_t await_receive(void) {
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

void to_ascii(uint32_t data, char* str) {
	uint32_t d = data;
	for (int i = 0; i < 8; i++) {
		uint8_t c = d & 0xF;
		
		if (c > 9) {
			str[7 - i] = c + 55;
		} else {
			str[7 - i] = c + 48;
		}
		
		d = d >> 4;
	}
}
