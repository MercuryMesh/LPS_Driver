#include "stm32f0xx_hal.h"

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;

extern uint8_t sendbuff [128];
extern uint8_t receivebuff [128];

uint8_t dw_init(SPI_HandleTypeDef* spi_instance);
uint8_t MX_SPI1_Init(void);
uint8_t MX_SPI2_Init(void);
void SPISend(SPI_HandleTypeDef* instance, uint8_t bytes);
void Send32At(uint8_t position, uint32_t bytes);
void SendAt(uint8_t position, uint8_t* bytes, uint8_t length);
void ReceiveAt(uint8_t position, uint8_t* write, uint8_t len);
uint8_t SetupSendBuff(uint8_t send, uint8_t reg, uint16_t subreg);

