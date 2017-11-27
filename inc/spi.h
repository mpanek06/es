#ifndef SPI_H
#define SPI_H

#include "stm32f4xx.h"

void SPIConfiguration();
uint16_t SPI_TxRx(uint16_t data, uint8_t RW);

#endif
