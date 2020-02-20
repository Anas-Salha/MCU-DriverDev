/*
 * 001ExtInterrupt.c
 *
 *  Created on: Oct 18, 2019
 *      Author: AnaS9
 */


#include "stm32f407xx.h"
#include <stdint.h>
#include <string.h>

void delay(void);

int main(void){

	GPIO_Handle_t gpioButton_ext;
	memset(&gpioButton_ext, 0, sizeof(gpioButton_ext)); //clear all members of the struct
	gpioButton_ext.pGPIOx = GPIOD;
	gpioButton_ext.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
	gpioButton_ext.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;

	GPIO_Handle_t gpioLD3;
	memset(&gpioLD3, 0, sizeof(gpioLD3)); //clear all members of the struct
	gpioLD3.pGPIOx = GPIOD;
	gpioLD3.GPIO_PinConfig.GPIO_PinNumber = LD3_PIN;
	gpioLD3.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioLD3.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpioLD3.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;

	GPIO_PCLK_Control(GPIOD, ENABLE);

	GPIO_Init(&gpioButton_ext);
	GPIO_Init(&gpioLD3);

	GPIO_IRQITConfig(IRQ_NO_EXTI9_5, ENABLE);
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRIO15);

	while(1);

	return 0;
}

void EXTI9_5_IRQHandler(void){

	delay(); //200ms
	GPIO_IRQHandling(GPIO_PIN_5);
	GPIO_OPinToggle(GPIOD, LD3_PIN);

}

void delay(void){
	uint32_t i;
	for(i=0; i<500000/2; i++);
}
