#include "stm32f4xx.h"
#include "gyro.h"
#include "spi.h"

Gyro_t gyroReadings;

void GYRO_IO_Write(uint16_t pBuffer, uint16_t WriteAddr)
{
  GYRO_CS_LOW();

  SPI_TxRx((WriteAddr<<8) | pBuffer, 0);

  GYRO_CS_HIGH();
}

uint16_t GYRO_IO_Read(uint8_t ReadAddr)
{
  uint16_t data=0;
  GYRO_CS_LOW();

  data = SPI_TxRx( (ReadAddr<<8), 1 );

  GYRO_CS_HIGH();

  return data & 0x00FF;
}

void Gyro_Init()
{
	uint16_t initVals = 0x0000;

	SPI5->CR1 |= SPI_CR1_DFF; //SPI in 16 bit mode

	Gyro_GPIO_Config();

	initVals = 1<<0 | 1<<1 | 0<<2 | 1<<3 | 1<<4 | 1<<5 | 1<<6 | 1<<7;
	/* Write value to MEMS CTRL_REG1 register */
	GYRO_IO_Write(initVals, L3GD20_CTRL_REG1_ADDR);

	initVals = 1<<4 | 1<<5 ;
	GYRO_IO_Write(initVals, L3GD20_CTRL_REG4_ADDR);
}

void Gyro_GPIO_Config()
{
	//CS - PC1
	GPIOC->MODER   |= GPIO_MODER_MODE1_0;
	GPIOC->OSPEEDR |= GPIO_OSPEEDR_OSPEED1_1;
}

void Gyro_ReadData()
{
	uint16_t tmp;

	tmp  = GYRO_IO_Read(0x28);
	tmp |= GYRO_IO_Read(0x29) << 8;

	gyroReadings.xAxisVel = (int16_t) tmp;

	tmp  = GYRO_IO_Read(0x2A);
	tmp |= GYRO_IO_Read(0x2B) << 8;

	gyroReadings.yAxisVel = (int16_t) tmp;

	tmp  = GYRO_IO_Read(0x2C);
	tmp |= GYRO_IO_Read(0x2D) << 8;

	gyroReadings.zAxisVel = (int16_t) tmp;
}

void Gyro_CalculatePosition()
{
	gyroReadings.xAxisPos += (double)gyroReadings.xAxisVel/20;
	gyroReadings.yAxisPos += (double)gyroReadings.yAxisVel/20;
	gyroReadings.zAxisPos += (double)gyroReadings.zAxisVel/20;

}
