#include "spi.h"
#include "stm32f4xx.h"

void SPIConfiguration()
{
	RCC->APB2ENR |= RCC_APB2ENR_SPI5EN;
	SPI5->CR1 = SPI_CR1_SPE | SPI_CR1_BR_0 | SPI_CR1_BR_1 | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_MSTR;
}

// RW: 1=R, 0=W
uint16_t SPI_TxRx(uint16_t data, uint8_t RW)
{
	data |= (uint16_t)RW<<15;

	while( !(SPI5->SR & (SPI_SR_TXE)) );
	SPI5->DR = data;
	while( !(SPI5->SR & SPI_SR_RXNE) );
	return SPI5->DR  & 0xFFFF;
}
