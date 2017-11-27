#ifndef GYRO_H
#define GYRO_H

#include "stm32f4xx.h"

#define GYRO_CS_LOW()      GPIOC->ODR &= ~(GPIO_ODR_OD1)
#define GYRO_CS_HIGH()     GPIOC->ODR |=  (GPIO_ODR_OD1)

#define L3GD20_CTRL_REG1_ADDR         0x20  /* Control register 1 */
#define L3GD20_CTRL_REG2_ADDR         0x21  /* Control register 2 */
#define L3GD20_CTRL_REG3_ADDR         0x22  /* Control register 3 */
#define L3GD20_CTRL_REG4_ADDR         0x23  /* Control register 4 */
#define L3GD20_CTRL_REG5_ADDR         0x24  /* Control register 5 */

typedef struct
{
	int16_t xAxis;
	int16_t yAxis;
	int16_t zAxis;
}Gyro_t;

void Gyro_Init();
void Gyro_GPIO_Config();
void GYRO_IO_Write(uint16_t pBuffer, uint16_t WriteAddr);
void Gyro_ReadData();

#endif
