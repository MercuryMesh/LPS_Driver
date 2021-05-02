#include "timer.h"

/**
	initializes tim3 to generate periodic interupts 
	to send new location requests.
*/
void tim3_init(uint16_t interval_ms, uint8_t priority) {
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	
	TIM3->PSC = 7999;
	TIM3->ARR = interval_ms;
	
	// enable the update interrupt
	TIM3->DIER |= (0x01);
	
	// enable the timer
	TIM3->CR1 |= (0x01);
	
	NVIC_EnableIRQ(TIM3_IRQn);
	HAL_NVIC_SetPriority(TIM3_IRQn, priority, priority);
}
