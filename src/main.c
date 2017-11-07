#include "stm32f4xx.h"
#include "ili9341.h"

volatile uint32_t tmp = 0;

void SysClockConfiguration();
void SysTickConfiguration();
void GPIOConfiguration();
void TIMConfiguration();
void SPIConfiguration();

void SPI_Tx(uint16_t data);
uint16_t SPI_Rx(void);

int main(void)
{
	SysClockConfiguration();
	SysTickConfiguration();
	GPIOConfiguration();
	TIMConfiguration();
	SPIConfiguration();
    ili9341_Init();

	while (1);

	return 0;
} /* main */

void SysTick_Handler()
{

	tmp += 1;
	if(0 == tmp%10)
	{
		GPIOG->ODR ^= GPIO_ODR_ODR_13;
	}
	else
	{
	}
	GPIOG->ODR ^= GPIO_ODR_ODR_14;
}

void SysClockConfiguration()
{
	RCC->CR |= RCC_CR_HSEON;
	// M = 8,PLLN = 360, PLLP=b00 = 2, PLLQ=7
	RCC->PLLCFGR = 8ul<<0 | 360ul<<6 | 0ul<<16 | 1ul<<22 | 7ul<<24 | 1ul<<29;
	//PLLSAIN     = 192
	//PLLSAIR     = 4
	//PLLSAIDivR  = 8
	//Reset val: 0010 0100 0000 0000 0011 0000 0000 0000
	RCC->PLLSAICFGR = 192ul <<6 | 4ul<<28;
	RCC->DCKCFGR |= 2ul<<16;

	while (!(RCC->CR & RCC_CR_HSERDY));

	RCC->CR |= RCC_CR_PLLON;
	RCC->CFGR = RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV2;

	FLASH->ACR = FLASH_ACR_DCRST | FLASH_ACR_ICRST;
	FLASH->ACR = FLASH_ACR_DCEN | FLASH_ACR_ICEN | FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_5WS;
	while ((FLASH->ACR & FLASH_ACR_LATENCY) != FLASH_ACR_LATENCY_5WS);

	while (!(RCC->CR & RCC_CR_PLLRDY));
	RCC->CFGR |= RCC_CFGR_SW_PLL;
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN
				  | RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN
				  | RCC_AHB1ENR_GPIOGEN | RCC_AHB1ENR_GPIOFEN;
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN | RCC_APB2ENR_SPI5EN;
	__DSB();
}

void SysTickConfiguration()
{
	SysTick->LOAD  = (uint32_t)(4000000 - 1UL);                       /* set reload register */
	SysTick->VAL   = 0UL;                                             /* Load the SysTick Counter Value */
	SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk |
					 SysTick_CTRL_TICKINT_Msk   |
		             SysTick_CTRL_ENABLE_Msk;
}

void GPIOConfiguration()
{
	// PG13, PG14 -Blinking LEDs
	GPIOG->MODER |= (1<<28);
	GPIOG->MODER |= (1<<26);
	GPIOG->ODR |= (1<<13);
	GPIOG->ODR |= (1<<14);

	// PE9, PE11, PE13, PE14 - TIM1 PWM
	GPIOE->MODER |= GPIO_MODER_MODE9_1 | GPIO_MODER_MODE11_1 | GPIO_MODER_MODE13_1 | GPIO_MODER_MODE14_1;
	//pp mode
	GPIOE->OTYPER |= GPIO_OTYPER_OT_9 | GPIO_OTYPER_OT_11 | GPIO_OTYPER_OT_13 | GPIO_OTYPER_OT_14 ;
	GPIOE->PUPDR  |= GPIO_PUPDR_PUPD9_0 | GPIO_PUPDR_PUPD11_0 | GPIO_PUPDR_PUPD13_0 | GPIO_PUPDR_PUPD14_0;
	//hight speed
	GPIOE->OSPEEDR |= GPIO_OSPEEDR_OSPEED9_1  | GPIO_OSPEEDR_OSPEED11_1 | GPIO_OSPEEDR_OSPEED13_1 | GPIO_OSPEEDR_OSPEED14_1;

	GPIOE->AFR[1] |=  GPIO_AFRH_AFRH1_0 | GPIO_AFRH_AFRH3_0 | GPIO_AFRH_AFRH5_0 | GPIO_AFRH_AFRH6_0 ;



	//SPI
	// PF7, PF8, PF9
	GPIOF->MODER |= GPIO_MODER_MODE7_1 | GPIO_MODER_MODE8_1 | GPIO_MODER_MODE9_1 ;
	//hight speed
	GPIOF->OSPEEDR |= GPIO_OSPEEDR_OSPEED7_0  | GPIO_OSPEEDR_OSPEED8_0 | GPIO_OSPEEDR_OSPEED9_0;

	GPIOF->AFR[0] |=  GPIO_AFRH_AFRH7_0 | GPIO_AFRH_AFRH7_2 ;
	GPIOF->AFR[1] |=  GPIO_AFRH_AFRH0_0 | GPIO_AFRH_AFRH0_2 | GPIO_AFRH_AFRH1_0 | GPIO_AFRH_AFRH1_2 ;


	//LCD
	//WRX, RDX
	GPIOD->MODER |= GPIO_MODER_MODE13_0 | GPIO_MODER_MODE12_0;
	GPIOD->OSPEEDR |= GPIO_OSPEEDR_OSPEED12_1  | GPIO_OSPEEDR_OSPEED13_1;

	//NCS
	GPIOC->MODER |= GPIO_MODER_MODE2_0;
	GPIOC->OSPEEDR |= GPIO_OSPEEDR_OSPEED2_1;
}

void TIMConfiguration()
{
	TIM1->CCMR1 = TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;
	TIM1->CCMR1 |= TIM_CCMR1_OC2PE | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;
	TIM1->CCMR2 |= TIM_CCMR2_OC3PE | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1;
	TIM1->CCMR2 |= TIM_CCMR2_OC4PE | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1;
	TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
	TIM1->BDTR = TIM_BDTR_MOE;
	TIM1->PSC = 167;
	TIM1->ARR = 999;
	TIM1->CCR1 = 800;
	TIM1->CCR2 = 600;
	TIM1->CCR3 = 400;
	TIM1->CCR4 = 200;
	TIM1->EGR = TIM_EGR_UG;
	TIM1->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
}

void SPIConfiguration()
{
	SPI5->CR1 = SPI_CR1_SPE | SPI_CR1_BR_0 | SPI_CR1_BR_1 | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_MSTR;
}

void SPI_Tx(uint16_t data)
{
//	while( !(SPI5->SR & SPI_SR_TXE) );
//	while( !(SPI5->SR & (SPI_SR_TXE) ));
//	SPI5->DR = data;
//
//
//	while( !(SPI5->SR & (SPI_SR_BSY) ));

	while( !(SPI5->SR & (SPI_SR_TXE) ));
	SPI5->DR = data;
	while( (SPI5->SR & (SPI_SR_BSY) ));
}

uint16_t SPI_Rx()
{
	uint16_t data = 0;

	while( !(SPI5->SR & SPI_SR_RXNE) );
	data = SPI5->DR;
	return data;
}

void ili9341_Init(void)
{
  /* Initialize ILI9341 low level bus layer ----------------------------------*/
  LCD_IO_Init();

  /* Configure LCD */
  ili9341_WriteReg(0xCA);
  ili9341_WriteData(0xC3);
  ili9341_WriteData(0x08);
  ili9341_WriteData(0x50);
  ili9341_WriteReg(LCD_POWERB);
  ili9341_WriteData(0x00);
  ili9341_WriteData(0xC1);
  ili9341_WriteData(0x30);
  ili9341_WriteReg(LCD_POWER_SEQ);
  ili9341_WriteData(0x64);
  ili9341_WriteData(0x03);
  ili9341_WriteData(0x12);
  ili9341_WriteData(0x81);
  ili9341_WriteReg(LCD_DTCA);
  ili9341_WriteData(0x85);
  ili9341_WriteData(0x00);
  ili9341_WriteData(0x78);
  ili9341_WriteReg(LCD_POWERA);
  ili9341_WriteData(0x39);
  ili9341_WriteData(0x2C);
  ili9341_WriteData(0x00);
  ili9341_WriteData(0x34);
  ili9341_WriteData(0x02);
  ili9341_WriteReg(LCD_PRC);
  ili9341_WriteData(0x20);
  ili9341_WriteReg(LCD_DTCB);
  ili9341_WriteData(0x00);
  ili9341_WriteData(0x00);
  ili9341_WriteReg(LCD_FRMCTR1);
  ili9341_WriteData(0x00);
  ili9341_WriteData(0x1B);
  ili9341_WriteReg(LCD_DFC);
  ili9341_WriteData(0x0A);
  ili9341_WriteData(0xA2);
  ili9341_WriteReg(LCD_POWER1);
  ili9341_WriteData(0x10);
  ili9341_WriteReg(LCD_POWER2);
  ili9341_WriteData(0x10);
  ili9341_WriteReg(LCD_VCOM1);
  ili9341_WriteData(0x45);
  ili9341_WriteData(0x15);
  ili9341_WriteReg(LCD_VCOM2);
  ili9341_WriteData(0x90);
  ili9341_WriteReg(LCD_MAC);
  ili9341_WriteData(0xC8);
  ili9341_WriteReg(LCD_3GAMMA_EN);
  ili9341_WriteData(0x00);
  ili9341_WriteReg(LCD_RGB_INTERFACE);
  ili9341_WriteData(0xC2);
  ili9341_WriteReg(LCD_DFC);
  ili9341_WriteData(0x0A);
  ili9341_WriteData(0xA7);
  ili9341_WriteData(0x27);
  ili9341_WriteData(0x04);

  /* Colomn address set */
  ili9341_WriteReg(LCD_COLUMN_ADDR);
  ili9341_WriteData(0x00);
  ili9341_WriteData(0x00);
  ili9341_WriteData(0x00);
  ili9341_WriteData(0xEF);
  /* Page address set */
  ili9341_WriteReg(LCD_PAGE_ADDR);
  ili9341_WriteData(0x00);
  ili9341_WriteData(0x00);
  ili9341_WriteData(0x01);
  ili9341_WriteData(0x3F);
  ili9341_WriteReg(LCD_INTERFACE);
  ili9341_WriteData(0x01);
  ili9341_WriteData(0x00);
  ili9341_WriteData(0x06);

  ili9341_WriteReg(LCD_GRAM);
//  LCD_Delay(200);
  for(volatile uint32_t z =0; z<2600000; ++z ){};

  ili9341_WriteReg(LCD_GAMMA);
  ili9341_WriteData(0x01);

  ili9341_WriteReg(LCD_PGAMMA);
  ili9341_WriteData(0x0F);
  ili9341_WriteData(0x29);
  ili9341_WriteData(0x24);
  ili9341_WriteData(0x0C);
  ili9341_WriteData(0x0E);
  ili9341_WriteData(0x09);
  ili9341_WriteData(0x4E);
  ili9341_WriteData(0x78);
  ili9341_WriteData(0x3C);
  ili9341_WriteData(0x09);
  ili9341_WriteData(0x13);
  ili9341_WriteData(0x05);
  ili9341_WriteData(0x17);
  ili9341_WriteData(0x11);
  ili9341_WriteData(0x00);
  ili9341_WriteReg(LCD_NGAMMA);
  ili9341_WriteData(0x00);
  ili9341_WriteData(0x16);
  ili9341_WriteData(0x1B);
  ili9341_WriteData(0x04);
  ili9341_WriteData(0x11);
  ili9341_WriteData(0x07);
  ili9341_WriteData(0x31);
  ili9341_WriteData(0x33);
  ili9341_WriteData(0x42);
  ili9341_WriteData(0x05);
  ili9341_WriteData(0x0C);
  ili9341_WriteData(0x0A);
  ili9341_WriteData(0x28);
  ili9341_WriteData(0x2F);
  ili9341_WriteData(0x0F);

  ili9341_WriteReg(LCD_SLEEP_OUT);
  for(volatile uint32_t z =0; z<2600000; ++z ){};
  	ili9341_WriteReg(LCD_DISPLAY_ON);
//  /* GRAM start writing */
  ili9341_WriteReg(LCD_GRAM);
}

void LCD_IO_WriteData(uint16_t RegValue)
{
  /* Set WRX to send data */
  LCD_WRX_HIGH();

  /* Reset LCD control line(/CS) and Send data */
  LCD_CS_LOW();
  SPI_Tx(RegValue);

  /* Deselect: Chip Select high */
  LCD_CS_HIGH();
}

void LCD_IO_WriteReg(uint8_t Reg)
{
  /* Reset WRX to send command */
  LCD_WRX_LOW();

  /* Reset LCD control line(/CS) and Send command */
  LCD_CS_LOW();
  SPI_Tx(Reg);

  /* Deselect: Chip Select high */
  LCD_CS_HIGH();
}


void ili9341_WriteReg(uint8_t LCD_Reg)
{
  LCD_IO_WriteReg(LCD_Reg);
}

void ili9341_WriteData(uint16_t RegValue)
{
  LCD_IO_WriteData(RegValue);
}

void LCD_IO_Init(void)
{
    LCD_CS_LOW();
    LCD_CS_HIGH();
}

