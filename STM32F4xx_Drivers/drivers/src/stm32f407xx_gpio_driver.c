/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Oct 15, 2019
 *      Author: Anas-Salha
 */

#include "stm32f407xx_gpio_driver.h"

/*Peripheral clock setup
 *
 */

/********************************************************************************
 *@fn			-	GPIO_PCLK_Control
 *
 *@brief		-	Enables or disables peripheral clock for the given GPIO port
 *
 *@param[in]	-	GPIO peripheral base address
 *@param[in]	-	ENABLE or DISABLE macros
 *@param[in]	-
 *
 *@return		-	none
 *
 *@Note			-

 */
void GPIO_PCLK_Control(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi){

	if(EnorDi == ENABLE){
		if(pGPIOx == GPIOA)
			GPIOA_PCLK_EN();
		else if(pGPIOx == GPIOB)
			GPIOB_PCLK_EN();
		else if(pGPIOx == GPIOC)
			GPIOC_PCLK_EN();
		else if(pGPIOx == GPIOD)
			GPIOD_PCLK_EN();
		else if(pGPIOx == GPIOE)
			GPIOE_PCLK_EN();
		else if(pGPIOx == GPIOF)
			GPIOF_PCLK_EN();
		else if(pGPIOx == GPIOG)
			GPIOG_PCLK_EN();
		else if(pGPIOx == GPIOH)
			GPIOH_PCLK_EN();
		else if(pGPIOx == GPIOI)
			GPIOI_PCLK_EN();
	}else if(EnorDi == DISABLE){
		if(pGPIOx == GPIOA)
			GPIOA_PCLK_DI();
		else if(pGPIOx == GPIOB)
			GPIOB_PCLK_DI();
		else if(pGPIOx == GPIOC)
			GPIOC_PCLK_DI();
		else if(pGPIOx == GPIOD)
			GPIOD_PCLK_DI();
		else if(pGPIOx == GPIOE)
			GPIOE_PCLK_DI();
		else if(pGPIOx == GPIOF)
			GPIOF_PCLK_DI();
		else if(pGPIOx == GPIOG)
			GPIOG_PCLK_DI();
		else if(pGPIOx == GPIOH)
			GPIOH_PCLK_DI();
		else if(pGPIOx == GPIOI)
			GPIOI_PCLK_DI();
	}
}

/* Initialization & De-initialization
 *
 */

/********************************************************************************
 *@fn			-	GPIO_Init
 *
 *@brief		-	Initializes a given GPIO port
 *
 *@param[in]	-	GPIO peripheral handling structure address
 *@param[in]	-
 *@param[in]	-
 *
 *@return		- none
 *
 *@Note			-

 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){

	uint32_t temp = 0;

	GPIO_PCLK_Control(pGPIOHandle->pGPIOx, ENABLE); //enable clock for gpio port

	//1. configure mode
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){
		//non interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << 2*(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clear
		pGPIOHandle->pGPIOx->MODER |= temp;	//set
	}

	else{
		//1. configure corresponding register
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT){
			EXTI->FTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);		//set corresponding FTSR bit
			EXTI->RTSR &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);		//clear corresponding RTSR bit
		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT){
			EXTI->RTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);		//set corresponding RTSR bit
			EXTI->FTSR &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);		//clear corresponding FTSR bit
		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT){
			EXTI->FTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);		//set corresponding FTSR bit
			EXTI->RTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);		//set corresponding RTSR bit
		}

		//2. configure GPIO port selection in SYSCFG_EXTICR
		SYSCFG_PCLK_EN();

		uint8_t temp1 = 0;
		uint8_t temp2 = 0;
		uint8_t portcode = 0;

		temp1 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4);
		temp2 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4);
		portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);

		SYSCFG->EXTICR[temp1] |= (portcode<<(temp2*4));

		//3. enable EXTI interrupt delivery in IMR
		EXTI->IMR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	}
	temp = 0;

	//2. configure speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << 2*(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clear
	pGPIOHandle->pGPIOx->OSPEEDR |= temp; //set
	temp = 0;

	//3. configure PuPd settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << 2*(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clear
	pGPIOHandle->pGPIOx->PUPDR |= temp; //set
	temp = 0;

	//4. configure optype
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x3 << 2*(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clear
	pGPIOHandle->pGPIOx->OTYPER |= temp; //set
	temp = 0;

	//5. configure alt. functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN){
		uint8_t temp1, temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << 4*temp2); //clear
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4*temp2)); //set
	}


}

/********************************************************************************
 *@fn			-	GPIO_DeInit
 *
 *@brief		-	Deinitializes a given GPIO port
 *
 *@param[in]	-	GPIO peripheral base address
 *@param[in]	-
 *@param[in]	-
 *
 *@return		-	none
 *
 *@Note			-

 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){
	if(pGPIOx == GPIOA)
		GPIOA_REG_RESET();
	else if(pGPIOx == GPIOB)
		GPIOB_REG_RESET();
	else if(pGPIOx == GPIOB)
		GPIOC_REG_RESET();
	else if(pGPIOx == GPIOB)
		GPIOD_REG_RESET();
	else if(pGPIOx == GPIOB)
		GPIOE_REG_RESET();
	else if(pGPIOx == GPIOB)
		GPIOF_REG_RESET();
	else if(pGPIOx == GPIOB)
		GPIOG_REG_RESET();
	else if(pGPIOx == GPIOB)
		GPIOH_REG_RESET();
	else if(pGPIOx == GPIOB)
		GPIOI_REG_RESET();
}

/* Data read & write
 *
 */

/********************************************************************************
 *@fn			-	GPIO_InPinRead
 *
 *@brief		-	Reads the state of a given GPIO pin
 *
 *@param[in]	-	GPIO peripheral base address
 *@param[in]	-	GPIO pin number
 *@param[in]	-
 *
 *@return		-	State of given GPIO pin (1 or 0)
 *
 *@Note			-

 */
uint8_t GPIO_InPinRead(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x01);
	return value;
}

/********************************************************************************
 *@fn			-	GPIO_InPortRead
 *
 *@brief		-	Reads the state of a given GPIO port
 *
 *@param[in]	-	GPIO peripheral base address
 *@param[in]	-
 *@param[in]	-
 *
 *@return		-	State of all pins in given GPIO port (1 or 0 for each)
 *
 *@Note			-

 */
uint16_t GPIO_InPortRead(GPIO_RegDef_t *pGPIOx){
	uint16_t value;
	value = (uint16_t)(pGPIOx->IDR);
	return value;
}

/********************************************************************************
 *@fn			-	GPIO_OPinWrite
 *
 *@brief		-	Writes to a given GPIO pin
 *
 *@param[in]	-	GPIO peripheral base address
 *@param[in]	-	GPIO pin number
 *@param[in]	-	GPIO_PIN_SET or GPIO_PIN_RESET macros
 *
 *@return		-	none
 *
 *@Note			-

 */
void GPIO_OPinWrite(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value){
	if(Value == GPIO_PIN_SET){
		pGPIOx->ODR |= (1 << PinNumber);
	}else{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

/********************************************************************************
 *@fn			-	GPIO_OPortWrite
 *
 *@brief		-	Writes to all pins in a given GPIO port
 *
 *@param[in]	-	GPIO peripheral base address
 *@param[in]	-	GPIO_PIN_SET or GPIO_PIN_RESET macros
 *@param[in]	-
 *
 *@return		-	none
 *
 *@Note			-

 */
void GPIO_OPortWrite(GPIO_RegDef_t *pGPIOx, uint16_t Value){
	pGPIOx->ODR = Value;
}

/********************************************************************************
 *@fn			-	GPIO_OPinToggle
 *
 *@brief		-	Toggles a given GPIO pin
 *
 *@param[in]	-	GPIO peripheral base address
 *@param[in]	-	GPIO pin number
 *@param[in]	-
 *
 *@return		-	none
 *
 *@Note			-

 */
void GPIO_OPinToggle(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	pGPIOx->ODR ^= (1<<PinNumber);
}

/*IRQ configuration & handling
 *
 */

/********************************************************************************
 *@fn			-	GPIO_IRQITConfig
 *
 *@brief		-	Enables/disables the appropriate NVIC IRQ numbers
 *
 *@param[in]	-	IRQ number
 *@param[in]	-	ENABLE or DISABLE macros
 *@param[in]	-
 *
 *@return		-	none
 *
 *@Note			-

 */
void GPIO_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi){
	if (EnorDi == ENABLE){
		if(IRQNumber <= 31){
			*NVIC_ISER0 |= (1 << IRQNumber);
		}else if(IRQNumber < 64){
			*NVIC_ISER1 |= (1 << (IRQNumber - 32));
		}else if(IRQNumber < 96){
			*NVIC_ISER2 |= (1 << (IRQNumber - 64));
		}
	}else{
		if(IRQNumber <= 31){
			*NVIC_ICER0 |= (1 << IRQNumber);
		}else if(IRQNumber < 64){
			*NVIC_ICER1 |= (1 << (IRQNumber - 32));
		}else if(IRQNumber < 96){
			*NVIC_ICER2 |= (1 << (IRQNumber - 64));
		}
	}
}

/********************************************************************************
 *@fn			-	GPIO_IRQPriorityConfig
 *
 *@brief		-	Sets given interrupt priorities
 *
 *@param[in]	-	IRQ number
 *@param[in]	-	IRQ priority
 *@param[in]	-
 *
 *@return		-	none
 *
 *@Note			-

 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority){
	//1. Identify IRPx
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift = (iprx_section*8) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_IPR_BASEADDR + (iprx)) |= (IRQPriority << shift);
}

/********************************************************************************
 *@fn			-	GPIO_IRQHandling
 *
 *@brief		-
 *
 *@param[in]	-	GPIO pin number
 *@param[in]	-
 *@param[in]	-
 *
 *@return		-	none
 *
 *@Note			-

 */
void GPIO_IRQHandling(uint8_t PinNumber){
	//clear the appropriate pending register bit
	if(EXTI->PR & (1 << PinNumber)){
		EXTI->PR |= (1 << PinNumber);
	}
}
