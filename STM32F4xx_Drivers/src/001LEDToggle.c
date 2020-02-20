/*
 * 001LEDToggle.c
 *
 *  Created on: Oct 17, 2019
 *      Author: AnaS9
 */



#include "stm32f407xx.h"
#include <stdint.h>

void delay(void);

int main(void){

	GPIO_Handle_t gpioLD3;
	gpioLD3.pGPIOx = GPIOD;
	gpioLD3.GPIO_PinConfig.GPIO_PinNumber = LD3_PIN;
	gpioLD3.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioLD3.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	gpioLD3.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Handle_t gpioLD4;
	gpioLD4.pGPIOx = GPIOD;
	gpioLD4.GPIO_PinConfig.GPIO_PinNumber = LD4_PIN;
	gpioLD4.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioLD4.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;

	GPIO_Handle_t gpioLD5;
	gpioLD5.pGPIOx = GPIOD;
	gpioLD5.GPIO_PinConfig.GPIO_PinNumber = LD5_PIN;
	gpioLD5.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioLD5.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;

	GPIO_Handle_t gpioLD6;
	gpioLD6.pGPIOx = GPIOD;
	gpioLD6.GPIO_PinConfig.GPIO_PinNumber = LD6_PIN;
	gpioLD6.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioLD6.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;

	GPIO_Handle_t gpioLED_ex;
	gpioLED_ex.pGPIOx = GPIOA;
	gpioLED_ex.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_8;
	gpioLED_ex.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioLED_ex.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpioLED_ex.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Handle_t gpioB1;
	gpioB1.pGPIOx = GPIOA;
	gpioB1.GPIO_PinConfig.GPIO_PinNumber = USER_BUTTON;
	gpioB1.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;

	GPIO_Handle_t gpioButton_ex;
	gpioButton_ex.pGPIOx = GPIOB;
	gpioButton_ex.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	gpioButton_ex.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	gpioButton_ex.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PCLK_Control(gpioButton_ex.pGPIOx, ENABLE);
	GPIO_PCLK_Control(gpioLED_ex.pGPIOx, ENABLE);
	GPIO_PCLK_Control(gpioB1.pGPIOx, ENABLE);
	GPIO_PCLK_Control(gpioLD3.pGPIOx, ENABLE);

	GPIO_Init(&gpioButton_ex);
	GPIO_Init(&gpioLED_ex);
	GPIO_Init(&gpioB1);
	GPIO_Init(&gpioLD3);
	GPIO_Init(&gpioLD4);
	GPIO_Init(&gpioLD5);
	GPIO_Init(&gpioLD6);

	while(1){
		if(GPIO_InPinRead(gpioButton_ex.pGPIOx,GPIO_PIN_12) == 1){
			while(1){
				if(GPIO_InPinRead(gpioButton_ex.pGPIOx,GPIO_PIN_12) == 0)
					break;
			}
			GPIO_OPinToggle(gpioLD3.pGPIOx, LD3_PIN);
			delay();
			GPIO_OPinToggle(gpioLD4.pGPIOx, LD4_PIN);
			delay();
			GPIO_OPinToggle(gpioLD5.pGPIOx, LD5_PIN);
			delay();
			GPIO_OPinToggle(gpioLD6.pGPIOx, LD6_PIN);
			delay();
			GPIO_OPinToggle(gpioLED_ex.pGPIOx, GPIO_PIN_8);
			delay();
		}
	}
	return 0;
}


void delay(void){
	uint32_t i;
	for(i=0; i<500000; i++);
}
