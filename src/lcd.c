#include "stm32f4xx.h"
#include "lcd.h"
#include "spi.h"
#include "asia.h"


void ili9341_Init(void)
{
  /* Initialize ILI9341 low level bus layer ----------------------------------*/
  LCD_IO_Init();

  /* Configure LCD */
  LCD_IO_WriteReg(0xCA);
  LCD_IO_WriteData(0xC3);
  LCD_IO_WriteData(0x08);
  LCD_IO_WriteData(0x50);
  LCD_IO_WriteReg(LCD_POWERB);
  LCD_IO_WriteData(0x00);
  LCD_IO_WriteData(0xC1);
  LCD_IO_WriteData(0x30);
  LCD_IO_WriteReg(LCD_POWER_SEQ);
  LCD_IO_WriteData(0x64);
  LCD_IO_WriteData(0x03);
  LCD_IO_WriteData(0x12);
  LCD_IO_WriteData(0x81);
  LCD_IO_WriteReg(LCD_DTCA);
  LCD_IO_WriteData(0x85);
  LCD_IO_WriteData(0x00);
  LCD_IO_WriteData(0x78);
  LCD_IO_WriteReg(LCD_POWERA);
  LCD_IO_WriteData(0x39);
  LCD_IO_WriteData(0x2C);
  LCD_IO_WriteData(0x00);
  LCD_IO_WriteData(0x34);
  LCD_IO_WriteData(0x02);
  LCD_IO_WriteReg(LCD_PRC);
  LCD_IO_WriteData(0x20);
  LCD_IO_WriteReg(LCD_DTCB);
  LCD_IO_WriteData(0x00);
  LCD_IO_WriteData(0x00);
  LCD_IO_WriteReg(LCD_FRMCTR1);
  LCD_IO_WriteData(0x00);
  LCD_IO_WriteData(0x1B);
  LCD_IO_WriteReg(LCD_DFC);
  LCD_IO_WriteData(0x0A);
  LCD_IO_WriteData(0xA2);
  LCD_IO_WriteReg(LCD_POWER1);
  LCD_IO_WriteData(0x10);
  LCD_IO_WriteReg(LCD_POWER2);
  LCD_IO_WriteData(0x10);
  LCD_IO_WriteReg(LCD_VCOM1);
  LCD_IO_WriteData(0x45);
  LCD_IO_WriteData(0x15);
  LCD_IO_WriteReg(LCD_VCOM2);
  LCD_IO_WriteData(0x90);
  LCD_IO_WriteReg(LCD_MAC);
  LCD_IO_WriteData(0xC8);
  LCD_IO_WriteReg(LCD_3GAMMA_EN);
  LCD_IO_WriteData(0x00);
  LCD_IO_WriteReg(LCD_RGB_INTERFACE);
  LCD_IO_WriteData(0xC2);
  LCD_IO_WriteReg(LCD_DFC);
  LCD_IO_WriteData(0x0A);
  LCD_IO_WriteData(0xA7);
  LCD_IO_WriteData(0x27);
  LCD_IO_WriteData(0x04);

  /* Colomn address set */
  LCD_IO_WriteReg(LCD_COLUMN_ADDR);
  LCD_IO_WriteData(0x00);
  LCD_IO_WriteData(0x00);
  LCD_IO_WriteData(0x00);
  LCD_IO_WriteData(0xEF);
  /* Page address set */
  LCD_IO_WriteReg(LCD_PAGE_ADDR);
  LCD_IO_WriteData(0x00);
  LCD_IO_WriteData(0x00);
  LCD_IO_WriteData(0x01);
  LCD_IO_WriteData(0x3F);
  LCD_IO_WriteReg(LCD_INTERFACE);
  LCD_IO_WriteData(0x01);
  LCD_IO_WriteData(0x00);
  LCD_IO_WriteData(0x06);

  LCD_IO_WriteReg(LCD_GRAM);
  LCD_Delay(200);

  LCD_IO_WriteReg(LCD_GAMMA);
  LCD_IO_WriteData(0x01);

  LCD_IO_WriteReg(LCD_PGAMMA);
  LCD_IO_WriteData(0x0F);
  LCD_IO_WriteData(0x29);
  LCD_IO_WriteData(0x24);
  LCD_IO_WriteData(0x0C);
  LCD_IO_WriteData(0x0E);
  LCD_IO_WriteData(0x09);
  LCD_IO_WriteData(0x4E);
  LCD_IO_WriteData(0x78);
  LCD_IO_WriteData(0x3C);
  LCD_IO_WriteData(0x09);
  LCD_IO_WriteData(0x13);
  LCD_IO_WriteData(0x05);
  LCD_IO_WriteData(0x17);
  LCD_IO_WriteData(0x11);
  LCD_IO_WriteData(0x00);
  LCD_IO_WriteReg(LCD_NGAMMA);
  LCD_IO_WriteData(0x00);
  LCD_IO_WriteData(0x16);
  LCD_IO_WriteData(0x1B);
  LCD_IO_WriteData(0x04);
  LCD_IO_WriteData(0x11);
  LCD_IO_WriteData(0x07);
  LCD_IO_WriteData(0x31);
  LCD_IO_WriteData(0x33);
  LCD_IO_WriteData(0x42);
  LCD_IO_WriteData(0x05);
  LCD_IO_WriteData(0x0C);
  LCD_IO_WriteData(0x0A);
  LCD_IO_WriteData(0x28);
  LCD_IO_WriteData(0x2F);
  LCD_IO_WriteData(0x0F);

  LCD_IO_WriteReg(LCD_SLEEP_OUT);
  LCD_Delay(200);
  LCD_IO_WriteReg(LCD_DISPLAY_ON);
//  /* GRAM start writing */
  LCD_IO_WriteReg(LCD_GRAM);
}

void LCD_IO_WriteData(uint16_t RegValue)
{
  /* Set WRX to send data */
  LCD_WRX_HIGH();

  /* Reset LCD control line(/CS) and Send data */
  LCD_CS_LOW();
  SPI_TxRx(RegValue, 0);

  /* Deselect: Chip Select high */
  LCD_CS_HIGH();
}

void LCD_IO_WriteReg(uint8_t Reg)
{
  /* Reset WRX to send command */
  LCD_WRX_LOW();

  /* Reset LCD control line(/CS) and Send command */
  LCD_CS_LOW();
  SPI_TxRx(Reg, 0);

  /* Deselect: Chip Select high */
  LCD_CS_HIGH();
}


void LCD_IO_Init(void)
{
    LCD_CS_LOW();
    LCD_CS_HIGH();
}


void LCD_Delay(uint8_t del)
{
	for(volatile uint32_t z =0; z<2600000; ++z ){};
}

void LCD_Config(void)
{
    ili9341_Init();

    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMA2EN | RCC_AHB1ENR_DMA2DEN | RCC_AHB1ENR_CCMDATARAMEN | RCC_AHB1ENR_CRCEN;
    RCC->AHB2ENR |= RCC_AHB2ENR_DCMIEN;
    RCC->AHB3ENR |= RCC_AHB3ENR_FMCEN;

    RCC->APB2RSTR |= RCC_APB2RSTR_LTDCRST;

    RCC->APB2RSTR &= ~(RCC_APB2RSTR_LTDCRST);

    RCC->APB2ENR |= RCC_APB2ENR_LTDCEN;


    //LTDC Synchronization Size Configuration Register
//    LTDC->SSCR |= 9ul<<16 | 1ul<<0;
//
//    //LTDC Back Porch Configuration Register
//    LTDC->BPCR |= 29ul<<16 | 3ul<<0;
//
//    //LTDC Active Width Configuration Register
//    LTDC->AWCR |= 269ul<<16 | 323ul<<0;
//
//    //LTDC Total Width Configuration Register
//    LTDC->TWCR |= 279ul<<16 | 327ul<<0;
//
//    //LTDC Background Color Configuration Register RGB
//    LTDC->BCCR |= 0xE0ul<<16 | 0x26ul<<8 | 0x4Cul<<0;
//
//    //LTDC Global Control Register
//    LTDC->GCR |= LTDC_GCR_LTDCEN;
//
////    LTDC->IER |= 0xe;
//
//    LTDC->SRCR |= 1;
//
//    //Layer 1 config:
//
//    // LTDC Layer1 Window Horizontal Position Config Register
////    LTDC_Layer1->WHPCR |= 0u<<16;
////    LTDC_Layer1->WHPCR |= 240u<<16;
//
//    LTDC_Layer1->WHPCR = 0x010D001E;
//
////    LTDC_Layer1->WVPCR |= 0u<<16;
////    LTDC_Layer1->WVPCR |= 160u<<16;
//
//    LTDC_Layer1->WVPCR |= 0x00A30004;
//
//    //RGB565 pixel format
//    LTDC_Layer1->PFCR |= 2;
//
//    //Alpha to 1.0
//    LTDC_Layer1->CACR = 0xff;
//
//
//
//
//    //addr
////    LTDC_Layer1->CFBAR = (uint32_t) ST_LOGO_2;
//
//    LTDC_Layer1->BFCR |= 3 | 2<<8;
//
//    LTDC_Layer1->DCCR = 0xff0000ff;
//
//    //    LTDC_Layer1->CFBLR |= ((160*2)<<16U | 160*2) + 3U;
//        LTDC_Layer1->CFBLR = 0x01e001e3;
//
//    //image height
////    LTDC_Layer1->CFBLNR = 160;
//    LTDC_Layer1->CFBLNR = 0xA0;
//    //turn on layer
//    LTDC_Layer1->CR |= 1;

#define LCD_WIDTH	 240
#define LCD_HEIGHT	320

#define HFP   10
#define HSYNC 10
#define HBP   20

#define VFP   4
#define VSYNC 2
#define VBP   2
#define ACTIVE_W (HSYNC + LCD_WIDTH + HBP - 1)
#define ACTIVE_H (VSYNC + LCD_HEIGHT + VBP - 1)

#define PIXELWIDHT 2

#define TOTAL_WIDTH  (HSYNC + HBP + LCD_WIDTH + HFP - 1)
#define TOTAL_HEIGHT (VSYNC + VBP + LCD_HEIGHT + VFP - 1)

//    /* PLL */
//    RCC->PLLSAICFGR = (200 << 6) | (7 << 24) | (4 << 28);
//    /* Enable SAI PLL */
//    RCC->CR |= RCC_CR_PLLSAION;
//    /* wait for SAI PLL ready */
//    while((RCC->CR & RCC_CR_PLLSAIRDY) == 0);
//
    /* enable clock for LTDC */
    RCC->APB2ENR |= RCC_APB2ENR_LTDCEN;
    /* Synchronization Size Configuration */
    LTDC->SSCR = ((HSYNC - 1) << 16) | (VSYNC - 1);
    /* Back Porch Configuration */
    LTDC->BPCR = ((HSYNC + HBP - 1) << 16) | ( VSYNC + VBP - 1);
    /* Active Width Configuration */
    LTDC->AWCR = (ACTIVE_W << 16) | (ACTIVE_H);
    /* Total Width Configuration */
    LTDC->TWCR = (TOTAL_WIDTH << 16) | (TOTAL_HEIGHT);
    /* Window Horizontal Position Configuration */
    LTDC_Layer1->WHPCR = HBP | ((HBP + LCD_WIDTH - 1) << 16);
    /* Window Vertical Position Configuration */
    LTDC_Layer1->WVPCR = VBP | ((VBP + LCD_HEIGHT - 1) << 16);
    /* Pixel Format Configuration */
    LTDC_Layer1->PFCR = 2;
    /* Color Frame Buffer Address */
    LTDC_Layer1->CFBAR = (uint32_t)asia;
    /* Color Frame Buffer Length */
    LTDC_Layer1->CFBLR = ((LCD_WIDTH * PIXELWIDHT) << 16) | ((LCD_WIDTH * PIXELWIDHT) + 3);
    /* Enable Layer */
    LTDC_Layer1->CR = LTDC_LxCR_LEN;
    /* Immediate Reload */
    LTDC->SRCR = LTDC_SRCR_IMR;
    /* Enable LTDC */
    LTDC->GCR = LTDC_GCR_LTDCEN;

}

void LCD_GPIO_Config(void)
{

	  /*
	   +------------------------+-----------------------+----------------------------+
	   +                       LCD pins assignment                                   +
	   +------------------------+-----------------------+----------------------------+
	   |  LCD_TFT R2 <-> PC.10  |  LCD_TFT G2 <-> PA.06 |  LCD_TFT B2 <-> PD.06      |
	   |  LCD_TFT R3 <-> PB.00  |  LCD_TFT G3 <-> PG.10 |  LCD_TFT B3 <-> PG.11      |
	   |  LCD_TFT R4 <-> PA.11  |  LCD_TFT G4 <-> PB.10 |  LCD_TFT B4 <-> PG.12      |
	   |  LCD_TFT R5 <-> PA.12  |  LCD_TFT G5 <-> PB.11 |  LCD_TFT B5 <-> PA.03      |
	   |  LCD_TFT R6 <-> PB.01  |  LCD_TFT G6 <-> PC.07 |  LCD_TFT B6 <-> PB.08      |
	   |  LCD_TFT R7 <-> PG.06  |  LCD_TFT G7 <-> PD.03 |  LCD_TFT B7 <-> PB.09      |
	   -------------------------------------------------------------------------------
	            |  LCD_TFT HSYNC <-> PC.06  | LCDTFT VSYNC <->  PA.04 |
	            |  LCD_TFT CLK   <-> PG.07  | LCD_TFT DE   <->  PF.10 |
	             -----------------------------------------------------

	  */

	/* LTDC pins configuraiton: PA3 -- 12 */
	GPIOA->MODER   |= GPIO_MODER_MODE3_1 | GPIO_MODER_MODE4_1 | GPIO_MODER_MODE6_1 |
			          GPIO_MODER_MODE11_1 | GPIO_MODER_MODE12_1;

	GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED3_1  | GPIO_OSPEEDR_OSPEED4_1 | GPIO_OSPEEDR_OSPEED6_1
			       | GPIO_OSPEEDR_OSPEED11_1 | GPIO_OSPEEDR_OSPEED12_1 ;

	GPIOA->AFR[0]  |= 14ul<<12 | 14ul<<16 | 14ul<<24 ;
	GPIOA->AFR[1]  |= 14ul<<12 | 14ul<<16 ;


	/* LTDC pins configuraiton: PB8 -- 11 */
	GPIOB->MODER   |= GPIO_MODER_MODE8_1 | GPIO_MODER_MODE9_1 | GPIO_MODER_MODE10_1 | GPIO_MODER_MODE11_1;
	GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEED8_1  | GPIO_OSPEEDR_OSPEED9_1 | GPIO_OSPEEDR_OSPEED10_1 | GPIO_OSPEEDR_OSPEED11_1;
	GPIOB->AFR[1]  |= 14ul<<0 | 14ul<<4 | 14ul<<8 | 14ul<<12;


	/* LTDC pins configuraiton: PC6 -- 10 */
	GPIOC->MODER   |= GPIO_MODER_MODE6_1 | GPIO_MODER_MODE7_1 | GPIO_MODER_MODE10_1;
	GPIOC->OSPEEDR |= GPIO_OSPEEDR_OSPEED6_1  | GPIO_OSPEEDR_OSPEED7_1 | GPIO_OSPEEDR_OSPEED10_1;
	GPIOC->AFR[0]  |= 14ul<<24 | 14ul<<28 ;
	GPIOC->AFR[1]  |= 14ul<<8;

	/* LTDC pins configuraiton: PD3 -- 6 */
	GPIOD->MODER   |=  GPIO_MODER_MODE3_1 | GPIO_MODER_MODE6_1;
	GPIOD->OSPEEDR |=  GPIO_OSPEEDR_OSPEED3_1 | GPIO_OSPEEDR_OSPEED6_1;
	GPIOD->AFR[0]  |= 14ul<<12 | 14ul<<24;


	/* LTDC pins configuraiton: PF10 */
	GPIOF->MODER   |= GPIO_MODER_MODE10_1;
	GPIOF->OSPEEDR |= GPIO_OSPEEDR_OSPEED10_1;
	GPIOF->AFR[1]  |= 14ul<<8;


	/* LTDC pins configuraiton: PG6 -- 11 */
	GPIOG->MODER   |= GPIO_MODER_MODE6_1 | GPIO_MODER_MODE7_1 | GPIO_MODER_MODE11_1;
	GPIOG->OSPEEDR |= GPIO_OSPEEDR_OSPEED6_1 | GPIO_OSPEEDR_OSPEED7_1 | GPIO_OSPEEDR_OSPEED11_1;
	GPIOG->AFR[0]  |= 14ul<<24 | 14ul<<28 ;
	GPIOG->AFR[1]  |= 14ul<<12;


	/* LTDC pins configuraiton: PB0 -- 1 */
	GPIOB->MODER   |= GPIO_MODER_MODE0_1 | GPIO_MODER_MODE1_1 ;
	GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEED0_1 | GPIO_OSPEEDR_OSPEED1_1;
	GPIOB->AFR[0]  |= 9ul<<0 | 9ul<<4 ;


	/* LTDC pins configuraiton: PG10 -- 12 */
	GPIOG->MODER   |= GPIO_MODER_MODE10_1 | GPIO_MODER_MODE12_1;
	GPIOG->OSPEEDR |= GPIO_OSPEEDR_OSPEED10_1 | GPIO_OSPEEDR_OSPEED12_1;
	GPIOG->AFR[1]  |= 9ul<<8 | 9ul<<16;

	//LCD
	//WRX, RDX
	GPIOD->MODER   |= GPIO_MODER_MODE13_0 | GPIO_MODER_MODE12_0;
	GPIOD->OSPEEDR |= GPIO_OSPEEDR_OSPEED12_1  | GPIO_OSPEEDR_OSPEED13_1;

	//NCS
	GPIOC->MODER   |= GPIO_MODER_MODE2_0;
	GPIOC->OSPEEDR |= GPIO_OSPEEDR_OSPEED2_1;

}
