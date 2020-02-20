/*
 * 009I2C_Slave.c
 *
 *  Created on: Nov 4, 2019
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
 * Function Prototypes
 */
void I2C_GPIOInits(void);
void GPIO_ButtonInits(void);
void I2C_Inits(void);
void delay(void);

/*
 * Address macros
 */
#define MY_ADDR				0x69

/*
 * Command Codes
 */
#define CMD_LEN_REQ			0x51
#define CMD_DATA_REQ		0x52

/*
 * Declarations
 */
I2C_Handle_t i2cSlave;
uint8_t TxBuffer[32] = "STM32 slave mode testing";
uint8_t cmd_code = 0;

int main(void){
	GPIO_ButtonInits(); //initialize user button
	I2C_GPIOInits(); //initialize required I2C gpio pins
	I2C_Inits(); //initialize I2C

	//I2C IRQ config
	I2C_IRQITConfig(IRQ_NO_I2C1_EV, ENABLE);
	I2C_IRQITConfig(IRQ_NO_I2C1_ER, ENABLE);

	I2C_PeripheralControl(I2C1, ENABLE); //enable I2C1
	I2C_InterruptControlBits(I2C1, ENABLE); //enable interrupt control bits

	//enable ACKing
	if(i2cSlave.I2CConfig.I2C_ACKCtrl == I2C_ACK_EN){
		I2C_ManageACK(i2cSlave.pI2Cx, I2C_ACK_EN);
	}

	while(1);

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
	memset(&i2cSlave, 0, sizeof(i2cSlave)); //clear all members of the structure
	i2cSlave.pI2Cx = I2C1;
	i2cSlave.I2CConfig.I2C_ACKCtrl = I2C_ACK_EN;
	i2cSlave.I2CConfig.I2C_DeviceAddr = MY_ADDR;
	i2cSlave.I2CConfig.I2C_SCLKSpeed = I2C_SPEED_SM;

	I2C_Init(&i2cSlave);
}

void delay(void){
	uint32_t i=0;
	for(i=0; i<250000; i++);
}


void I2C1_EV_IRQHandler(void){
	I2C_EV_IRQHandling(&i2cSlave);
}

void I2C1_ER_IRQHandler(void){
	I2C_ER_IRQHandling(&i2cSlave);
}

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv){
	static uint8_t cnt = 0;

	if(AppEv == I2C_EV_DATA_REQ){
		//Send data
		if(cmd_code == CMD_LEN_REQ)			//Case 1. CMD_LEN_REQ
			I2C_SlaveTX(pI2CHandle->pI2Cx, strlen((char *)TxBuffer));
		else if(cmd_code == CMD_DATA_REQ)	//Case 2. CMD_DATA_REQ
			I2C_SlaveTX(pI2CHandle->pI2Cx, TxBuffer[cnt++]);

	}else if(AppEv == I2C_EV_DATA_RCV){
		//Receive data
		cmd_code = I2C_SlaveRX(pI2CHandle->pI2Cx);

	}else if(AppEv == I2C_EV_STOP){
		//Stop reception

	}else if(AppEv == I2C_ER_AF){
		//Stop transmission
		cmd_code = 0xFF;
		cnt = 0;


	}
}
