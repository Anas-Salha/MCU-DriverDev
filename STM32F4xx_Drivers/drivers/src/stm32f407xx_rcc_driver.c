/*
 * stm32f407xx_rcc_driver.c
 *
 *  Created on: Nov 7, 2019
 *      Author: AnaS9
 */

#include "stm32f407xx_rcc_driver.h"

uint16_t AHB1_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint8_t APB1_PreScaler[4] = { 2, 4 , 8, 16};
uint8_t APB2_PreScaler[4] = { 2, 4 , 8, 16};
/*
 * RCC functions
 */

uint32_t RCC_GetPLLOutputClk(void){
	uint32_t temp = 0;

	return temp;
}

/********************************************************************************
 *@fn			-	RCC_GetPCLK1Value
 *
 *@brief		-	Gets value of PCLK1 (APB1 bus)
 *
 *@param[in]	-
 *@param[in]	-
 *@param[in]	-
 *
 *@return		-	PCLK1 value
 *
 *@Note			-

 */
uint32_t RCC_GetPCLK1Value(void){
	uint8_t clksrc, temp, ahb1p, apb1p;
	uint32_t pclk1, sysclk;

	clksrc = (RCC->CFGR << RCC_CFGR_SWS) & 0x3;

	if(clksrc == RCC_SYSCLK_HSI)
		sysclk = RCC_SYSCLK_VALUE_HSI;
	else if(clksrc == RCC_SYSCLK_HSE)
		sysclk = RCC_SYSCLK_VALUE_HSE;
	else if(clksrc == RCC_SYSCLK_PLL)
		sysclk = RCC_GetPLLOutputClk();

	//AHB1 prescaler
	temp = (RCC->CFGR << RCC_CFGR_HPRE) & 0xF;
	if(temp < 8)
		ahb1p = 1;
	else
		ahb1p = AHB1_PreScaler[temp - 8];

	//APB1 prescaler
	temp = (RCC->CFGR << RCC_CFGR_PPRE1) & 0x7;
	if(temp < 4)
		apb1p = 1;
	else
		apb1p = APB1_PreScaler[temp - 4];


	pclk1= ((sysclk/ahb1p)/apb1p);

	return pclk1;
}

/********************************************************************************
 *@fn			-	RCC_GetPCLK2Value
 *
 *@brief		-	Gets value of PCLK2 (APB2 bus)
 *
 *@param[in]	-
 *@param[in]	-
 *@param[in]	-
 *
 *@return		-	PCLK2 value
 *
 *@Note			-

 */
uint32_t RCC_GetPCLK2Value(void){
	uint8_t clksrc, temp, ahb1p, apb2p;
	uint32_t pclk2, sysclk;

	clksrc = (RCC->CFGR << RCC_CFGR_SWS) & 0x3;

	if(clksrc == RCC_SYSCLK_HSI)
		sysclk = RCC_SYSCLK_VALUE_HSI;
	else if(clksrc == RCC_SYSCLK_HSE)
		sysclk = RCC_SYSCLK_VALUE_HSE;
	else if(clksrc == RCC_SYSCLK_PLL)
		sysclk = RCC_GetPLLOutputClk();

	//AHB1 prescaler
	temp = (RCC->CFGR << RCC_CFGR_HPRE) & 0xF;
	if(temp < 8)
		ahb1p = 1;
	else
		ahb1p = AHB1_PreScaler[temp - 8];

	//APB1 prescaler
	temp = (RCC->CFGR << RCC_CFGR_PPRE2) & 0x7;
	if(temp < 4)
		apb2p = 1;
	else
		apb2p = APB1_PreScaler[temp - 4];


	pclk2= ((sysclk/ahb1p)/apb2p);

	return pclk2;
}
