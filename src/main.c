#include "stm32f4xx.h"

volatile uint8_t tmp = 0;

void SysClockConfiguration();
void SysTickConfiguration();
void GPIOConfiguration();
void TIMConfiguration();
void SPIConfiguration();

int main(void)
{
	SysClockConfiguration();
	SysTickConfiguration();
	GPIOConfiguration();
	TIMConfiguration();

	while (1);

	return 0;
} /* main */


void SysTick_Handler()
{
	tmp += 1;
	if(0 == tmp%2)
	{
		GPIOG->ODR ^= GPIO_ODR_ODR_13;
	}

	GPIOG->ODR ^= GPIO_ODR_ODR_14;
}

void SysClockConfiguration()
{
	RCC->CR |= RCC_CR_HSEON;
	RCC->PLLCFGR = 8ul<<0 | 336ul<<6 | 0ul<<16 | 1ul<<22 | 1ul<<29;
	while (!(RCC->CR & RCC_CR_HSERDY));

	RCC->CR |= RCC_CR_PLLON;
	RCC->CFGR = RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV2;

	FLASH->ACR = FLASH_ACR_DCRST | FLASH_ACR_ICRST;
	FLASH->ACR = FLASH_ACR_DCEN | FLASH_ACR_ICEN | FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_5WS;
	while ((FLASH->ACR & FLASH_ACR_LATENCY) != FLASH_ACR_LATENCY_5WS);

	while (!(RCC->CR & RCC_CR_PLLRDY));
	RCC->CFGR |= RCC_CFGR_SW_PLL;
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOEEN  | RCC_AHB1ENR_GPIOGEN;
	RCC->APB2ENR = RCC_APB2ENR_TIM1EN;
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
	GPIOG->MODER |= (1<<28);
	GPIOG->MODER |= (1<<26);
	GPIOG->ODR |= (1<<13);
	GPIOG->ODR |= (1<<14);
}

void TIMConfiguration()
{
	GPIOE->MODER |= GPIO_MODER_MODE9_1 | GPIO_MODER_MODE11_1 | GPIO_MODER_MODE13_1 | GPIO_MODER_MODE14_1;
	//pp mode
	GPIOE->OTYPER |= GPIO_OTYPER_OT_9 | GPIO_OTYPER_OT_11 | GPIO_OTYPER_OT_13 | GPIO_OTYPER_OT_14 ;
	GPIOE->PUPDR  |= GPIO_PUPDR_PUPD9_0 | GPIO_PUPDR_PUPD11_0 | GPIO_PUPDR_PUPD13_0 | GPIO_PUPDR_PUPD14_0;
	//hight speed
	GPIOE->OSPEEDR |= GPIO_OSPEEDR_OSPEED9_1  | GPIO_OSPEEDR_OSPEED11_1 | GPIO_OSPEEDR_OSPEED13_1 | GPIO_OSPEEDR_OSPEED14_1;

	GPIOE->AFR[1] |=  GPIO_AFRH_AFRH1_0 | GPIO_AFRH_AFRH3_0 | GPIO_AFRH_AFRH5_0 | GPIO_AFRH_AFRH6_0 ;

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

}
