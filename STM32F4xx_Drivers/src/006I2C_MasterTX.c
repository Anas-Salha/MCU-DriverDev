/*
 * main.c
 *
 *  Created on: Oct 27, 2019
 *      Author: AnaS9
 */

#include "stm32f407xx.h"
#include <stdint.h>
#include <string.h>

/*
 * PB6 SCL
 * PB7 SDA
 */

/*
 * Address macros
 */
#define MY_ADDR				0x61
#define SLAVE_ADDR			0x68

/*
 * Function Prototypes
 */
void I2C_GPIOInits(void);
void GPIO_ButtonInits(void);
void I2C_Inits(void);
void delay(void);

I2C_Handle_t i2cMaster;

int main(void){

	uint8_t data[] = "How you doin'?\n";

	GPIO_ButtonInits(); //initialize user button
	I2C_GPIOInits(); //initialize required I2C gpio pins
	I2C_Inits(); //initialize I2C
	I2C_PeripheralControl(I2C1, ENABLE); //enable I2C1

	while(1){
		if(GPIO_InPinRead(GPIOA, USER_BUTTON) == 1){
			while(GPIO_InPinRead(GPIOA, USER_BUTTON));
			I2C_MasterTX(&i2cMaster, data, strlen((char*)data), SLAVE_ADDR);
		}
	}

	I2C_PeripheralControl(I2C1, DISABLE); //disable I2C1
	return 0;
}

void GPIO_ButtonInits(void){
	GPIO_Handle_t gpioB1;
	memset(&gpioB1, 0, sizeof(gpioB1)); //clear all members of the structure
	gpioB1.pGPIOx = GPIOA;
	gpioB1.GPIO_PinConfig.GPIO_PinNumber = USER_BUTTON;
	gpioB1.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIO_Init(&gpioB1);
}

void I2C_GPIOInits(void){
	GPIO_Handle_t I2CPINS;
	memset(&I2CPINS, 0, sizeof(I2CPINS)); //clear all members of the structure
	I2CPINS.pGPIOx = GPIOB;
	I2CPINS.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPINS.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	I2CPINS.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2CPINS.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;

	//SCL
	I2CPINS.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_6;
	GPIO_Init(&I2CPINS);

	//SDA
	I2CPINS.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_7;
	GPIO_Init(&I2CPINS);
}

void I2C_Inits(void){
	memset(&i2cMaster, 0, sizeof(i2cMaster)); //clear all members of the structure
	i2cMaster.pI2Cx = I2C1;
	i2cMaster.I2CConfig.I2C_ACKCtrl = I2C_ACK_EN;
	i2cMaster.I2CConfig.I2C_DeviceAddr = MY_ADDR;
	i2cMaster.I2CConfig.I2C_SCLKSpeed = I2C_SPEED_SM;

	I2C_Init(&i2cMaster);
}

void delay(void){
	uint32_t i=0;
	for(i=0; i<250000; i++);
}



