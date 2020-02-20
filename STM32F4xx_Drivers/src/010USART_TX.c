/*
 * 010USART_TX.c
 *
 *  Created on: Nov 7, 2019
 *      Author: AnaS9
 */

#include "stm32f407xx.h"
#include <stdint.h>
#include <string.h>

/*
 * TX -> PA2
 * RX -> PA3
 * BR: 115200 bps
 * Frame: 1 stop bit, 8 data bits, no parity
 */

/*
 * Function Prototypes
 */
void USART2_GPIOInits(void);
void USART_Inits(void);
void GPIO_ButtonInits(void);
void delay(void);

/*
 * Declarations
 */
USART_Handle_t USART2Handle;
char msg[1024] = "UART TX Testing.. \n\r";


int main(void){
	USART2_GPIOInits();	//initialize gpio used for USART transmission
	USART_Inits();	//initialize USART peripheral
	GPIO_ButtonInits();	//initialize user button

	USART_PeripheralControl(USART2Handle.pUSARTx, ENABLE);	//enable clock for USART2 peripheral

	while(1){
		//send message after button press
		if(GPIO_InPinRead(GPIOA, USER_BUTTON) == 1){
			while(GPIO_InPinRead(GPIOA, USER_BUTTON));
			USART_DataTX(&USART2Handle, (uint8_t*)msg, strlen(msg)); //send message
		}
	}


	return 0;
}

void USART2_GPIOInits(void){
	GPIO_Handle_t gpioUSART;
	memset(&gpioUSART, 0, sizeof(gpioUSART)); //clear all members of the structure
	gpioUSART.pGPIOx = GPIOA;
	gpioUSART.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	gpioUSART.GPIO_PinConfig.GPIO_PinAltFunMode = 7;
	gpioUSART.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpioUSART.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;
	gpioUSART.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	//TX
	gpioUSART.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_2;
	GPIO_Init(&gpioUSART);

	//RX
	gpioUSART.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_3;
	GPIO_Init(&gpioUSART);
}

void USART_Inits(void){
	memset(&USART2Handle, 0, sizeof(USART2Handle)); //clear all members of the structure
	USART2Handle.pUSARTx = USART2;
	USART2Handle.USARTConfig.USART_Mode = USART_MODE_TX;
	USART2Handle.USARTConfig.USART_Baud = 115200;
	USART2Handle.USARTConfig.USART_HWFlowCtrl = USART_HWFLOW_NONE;
	USART2Handle.USARTConfig.USART_ParityCtrl = USART_PARITY_DI;
	USART2Handle.USARTConfig.USART_StopBitsNum = USART_STOPBITS_1;
	USART2Handle.USARTConfig.USART_WordLen = USART_WORDLEN_8BITS;

	USART_Init(&USART2Handle);
}

void GPIO_ButtonInits(void){
	GPIO_Handle_t gpioB1;
	memset(&gpioB1, 0, sizeof(gpioB1)); //clear all members of the structure
	gpioB1.pGPIOx = GPIOA;
	gpioB1.GPIO_PinConfig.GPIO_PinNumber = USER_BUTTON;
	gpioB1.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIO_Init(&gpioB1);
}







void delay(void){
	uint32_t i=0;
	for(i=0; i<250000; i++);
}
