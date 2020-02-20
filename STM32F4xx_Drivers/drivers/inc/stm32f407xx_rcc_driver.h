/*
 * stm32f407xx_rcc_driver.h
 *
 *  Created on: Nov 7, 2019
 *      Author: AnaS9
 */

#ifndef INC_STM32F407XX_RCC_DRIVER_H_
#define INC_STM32F407XX_RCC_DRIVER_H_

#include "stm32f407xx.h"

/************************************************* APIs supported by this driver *******************************************************/

uint32_t RCC_GetPLLOutputClk(void);
uint32_t RCC_GetPCLK1Value(void);
uint32_t RCC_GetPCLK2Value(void);

/*********************************************************** USER MACROS ****************************************************************/

/*
 * @RCC_SYSCLK
 * Possible SYSCLK options
 */
#define RCC_SYSCLK_HSI					0
#define RCC_SYSCLK_HSE					1
#define RCC_SYSCLK_PLL					2

/*
 * @RCC_SYSCLK_VALUE
 * Some SYSCLK values
 */
#define RCC_SYSCLK_VALUE_HSI			16000000
#define RCC_SYSCLK_VALUE_HSE			8000000

#endif /* INC_STM32F407XX_RCC_DRIVER_H_ */
