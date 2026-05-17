#ifndef __MAIN_H
#define __MAIN_H

#include "stm32h7xx_hal.h"

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_FDCAN1_Init(void);
void MX_I2C1_Init(void);
void MX_USART1_Init(void);
void Error_Handler(void);

#endif
