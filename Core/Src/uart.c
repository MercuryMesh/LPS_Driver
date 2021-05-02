#include "stm32f0xx_hal.h"
#include "uart.h"

void uart_init(int baude_rate) {
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
	
	/**
		Setting transmit and receive registers
		PC4 ------> USART3_TX
		PC5 ------> USART3_RX
	*/
	
	// set each to af mode
	GPIOC->MODER |= (0x01 << 9) | (0x01 << 11);
	GPIOC->MODER &= ~((0x01 << 8) | (0x01 << 10));
	
	// set each to AF1 mode
	GPIOC->AFR[0] &= ~((0x03 <<17) | (0x03 << 21));
	GPIOC->AFR[0] |= (0x01 << 16) | (0x01 << 20);
	
	// set the USART baude rate to 115200
	uint16_t brr = HAL_RCC_GetHCLKFreq() / baude_rate;
	USART3->BRR = brr;
	
	// enable tx and rx
	USART3->CR1 |= (0x01 << 3) | (0x01 << 2);
	
	// enable the USART
	USART3->CR1 |= 0x01;
}

// returns the number of characters transmitted
int transmit_string(char* txs) {
	int i = 0;
	char curr = txs[i];
	// once curr is false (0) that means we're at the end.
	while (curr) {
		transmit_char(curr);
		curr = txs[++i];
	}
	return i;
}

void transmit_char(char tx) {
	while (!(USART3->ISR & (0x01 << 7))) {
	}
	
	USART3->TDR = tx;
}


