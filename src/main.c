#include "stm32f4xx.h"
#include "lcd.h"
#include "spi.h"
#include "gyro.h"
#include "asia.h"

volatile uint32_t cnt = 0;

volatile uint8_t gyro_read_flag = 0;
void SysClockConfiguration();
void SysTickConfiguration();


void GPIOConfiguration();
void ExItConfiguration();

void TIMConfiguration();

extern Gyro_t gyroReadings;

int main(void)
{
	SysClockConfiguration();

	GPIOConfiguration();

	ExItConfiguration();

	TIMConfiguration();

	SPIConfiguration();

	LCD_GPIO_Config();
	LCD_Config();

	Gyro_Init();

	SysTickConfiguration();

	uint8_t active_layer = 0;
	while (1)
	{
		if( 1 == gyro_read_flag )
		{
			Gyro_ReadData();
			gyro_read_flag = 0;

			if( 1 == active_layer )
			{
//				LCD_clearLayer(0);
				LCD_drawSquare(53,  80 + gyroReadings.xAxis/350, 30, 0);
				LCD_drawSquare(125, 80 + gyroReadings.yAxis/350, 30, 0);
				LCD_drawSquare(195, 80 + gyroReadings.zAxis/350, 30, 0);
				LCD_setActiveLayer(0);
				active_layer = 0;
			}
			else
			{
//				LCD_clearLayer(1);
				LCD_drawSquare(53,  160 + gyroReadings.xAxis/350, 30, 1);
				LCD_drawSquare(125, 160 + gyroReadings.yAxis/350, 30, 1);
				LCD_drawSquare(195, 160 + gyroReadings.zAxis/350, 30, 1);
				LCD_setActiveLayer(1);
				active_layer = 1;
			}
		}
	}

	return 0;
} /* main */

void SysTick_Handler()
{
	cnt += 1;
	if(0 == cnt%10)
	{
		GPIOG->ODR ^= GPIO_ODR_ODR_13;
	}

	gyro_read_flag = 1;

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
	while (!(RCC->CR & RCC_CR_PLLRDY));

	RCC->CR |=  RCC_CR_PLLSAION;
	while (!(RCC->CR & RCC_CR_PLLSAIRDY));

	RCC->CFGR = RCC_CFGR_HPRE_0 | RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV2;

	FLASH->ACR = FLASH_ACR_DCRST | FLASH_ACR_ICRST;
	FLASH->ACR = FLASH_ACR_DCEN | FLASH_ACR_ICEN | FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_5WS;
	while ((FLASH->ACR & FLASH_ACR_LATENCY) != FLASH_ACR_LATENCY_5WS);


	RCC->CFGR |= RCC_CFGR_SW_PLL;
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
	__DSB();
}

void SysTickConfiguration()
{
	SysTick->LOAD  = (uint32_t)(4000000 - 1UL);                       /* set reload Lister */
	SysTick->VAL   = 0UL;                                             /* Load the SysTick Counter Value */
	SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk |
					 SysTick_CTRL_TICKINT_Msk   |
		             SysTick_CTRL_ENABLE_Msk;
}

void GPIOConfiguration()
{

	RCC->AHB1ENR  |=RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN
				  | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN
				  | RCC_AHB1ENR_GPIOEEN | RCC_AHB1ENR_GPIOFEN
				  | RCC_AHB1ENR_GPIOGEN;

	//PA0 - user btn
	GPIOA->MODER  |= 1<<0;

	// PG13, PG14 -Blinking LEDs
	GPIOG->MODER  |= (1<<28);
	GPIOG->MODER  |= (1<<26);
	GPIOG->ODR    |= (1<<13);
	GPIOG->ODR    |= (1<<14);

	// PE9, PE11, PE13, PE14 - TIM1 PWM
	GPIOE->MODER   |= GPIO_MODER_MODE9_1 | GPIO_MODER_MODE11_1 | GPIO_MODER_MODE13_1 | GPIO_MODER_MODE14_1;
	//pp mode
	GPIOE->OTYPER  |= GPIO_OTYPER_OT_9 | GPIO_OTYPER_OT_11 | GPIO_OTYPER_OT_13 | GPIO_OTYPER_OT_14 ;
	GPIOE->PUPDR   |= GPIO_PUPDR_PUPD9_0 | GPIO_PUPDR_PUPD11_0 | GPIO_PUPDR_PUPD13_0 | GPIO_PUPDR_PUPD14_0;
	//hight speed
	GPIOE->OSPEEDR |= GPIO_OSPEEDR_OSPEED9_1  | GPIO_OSPEEDR_OSPEED11_1 | GPIO_OSPEEDR_OSPEED13_1 | GPIO_OSPEEDR_OSPEED14_1;

	GPIOE->AFR[1]  |=  GPIO_AFRH_AFRH1_0 | GPIO_AFRH_AFRH3_0 | GPIO_AFRH_AFRH5_0 | GPIO_AFRH_AFRH6_0 ;

	//SPI
	// PF7, PF8, PF9
	GPIOF->MODER   |= GPIO_MODER_MODE7_1  | GPIO_MODER_MODE8_1 | GPIO_MODER_MODE9_1 ;
	//hight speed
	GPIOF->OSPEEDR |= GPIO_OSPEEDR_OSPEED7_0  | GPIO_OSPEEDR_OSPEED8_0 | GPIO_OSPEEDR_OSPEED9_0;

	GPIOF->AFR[0]  |=  GPIO_AFRH_AFRH7_0 | GPIO_AFRH_AFRH7_2 ;
	GPIOF->AFR[1]  |=  GPIO_AFRH_AFRH0_0 | GPIO_AFRH_AFRH0_2 | GPIO_AFRH_AFRH1_0 | GPIO_AFRH_AFRH1_2 ;


	//LCD
	//WRX, RDX
	GPIOD->MODER   |= GPIO_MODER_MODE13_0 | GPIO_MODER_MODE12_0;
	GPIOD->OSPEEDR |= GPIO_OSPEEDR_OSPEED12_1  | GPIO_OSPEEDR_OSPEED13_1;

	//NCS
	GPIOC->MODER   |= GPIO_MODER_MODE2_0;
	GPIOC->OSPEEDR |= GPIO_OSPEEDR_OSPEED2_1;
}

void ExItConfiguration()
{
	//EXT IT on PA0 - user btn1
	SYSCFG->EXTICR[0] |= 1;
	EXTI->RTSR |= EXTI_RTSR_TR0;
	NVIC_EnableIRQ(EXTI0_IRQn);
}

void EXTI0_IRQHandler(void)
{

}

void TIMConfiguration()
{
	TIM1->CCMR1  = TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;
	TIM1->CCMR1 |= TIM_CCMR1_OC2PE | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;
	TIM1->CCMR2 |= TIM_CCMR2_OC3PE | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1;
	TIM1->CCMR2 |= TIM_CCMR2_OC4PE | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1;
	TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
	TIM1->BDTR = TIM_BDTR_MOE;
	TIM1->PSC  = 167;
	TIM1->ARR  = 999;
	TIM1->CCR1 = 800;
	TIM1->CCR2 = 600;
	TIM1->CCR3 = 400;
	TIM1->CCR4 = 200;
	TIM1->EGR  = TIM_EGR_UG;
	TIM1->CR1  = TIM_CR1_ARPE | TIM_CR1_CEN;
}
