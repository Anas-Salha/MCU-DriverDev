/*
 * 008I2C_MasterRX_IT.c
 *
 *  Created on: Nov 2, 2019
 *      Author: AnaS9
 */



#include "stm32f407xx.h"
#include <stdint.h>
#include <string.h>

/*
 * PB6 SCL
 * PB7 SDA
 */

//Flag variable
uint8_t RxComplete = RESET;

/*
 * Address macros
 */
#define MY_ADDR				0x61
#define SLAVE_ADDR			0x68

/*
 * Command Codes
 */
#define CMD_LEN_REQ			0x51
#define CMD_DATA_REQ		0x52

/*
 * Function Prototypes
 */
void I2C_GPIOInits(void);
void GPIO_ButtonInits(void);
void I2C_Inits(void);
void delay(void);

I2C_Handle_t i2cMaster;

int main(void){
	uint8_t Len, cmd_code;
	uint8_t Data[32] = {0};

	GPIO_ButtonInits(); //initialize user button
	I2C_GPIOInits(); //initialize required I2C gpio pins
	I2C_Inits(); //initialize I2C

	//I2C IRQ config
	I2C_IRQITConfig(IRQ_NO_I2C1_EV, ENABLE);
	I2C_IRQITConfig(IRQ_NO_I2C1_ER, ENABLE);

	I2C_PeripheralControl(I2C1, ENABLE); //enable I2C1

	//enable ACKing
	if(i2cMaster.I2CConfig.I2C_ACKCtrl == I2C_ACK_EN){
		I2C_ManageACK(i2cMaster.pI2Cx, I2C_ACK_EN);
	}

	//MAIN CODE
	while(1){
		//0. Wait for button click
		if(GPIO_InPinRead(GPIOA, USER_BUTTON) == 1){
			while(GPIO_InPinRead(GPIOA, USER_BUTTON));
			//1. Send LEN_REQUEST
			cmd_code = CMD_LEN_REQ;
			while(I2C_MasterTX_IT(&i2cMaster, &cmd_code, 1, SLAVE_ADDR, I2C_SR_EN) != I2C_READY);
			//2. Receive length
			while(I2C_MasterRX_IT(&i2cMaster, &Len, 1, SLAVE_ADDR, I2C_SR_EN) != I2C_READY);
			//3. Send DATA_REQUEST
			cmd_code = CMD_DATA_REQ;
			while(I2C_MasterTX_IT(&i2cMaster, &cmd_code, 1, SLAVE_ADDR, I2C_SR_EN) != I2C_READY);
			//4. Receive data
			while(I2C_MasterRX_IT(&i2cMaster, Data, Len, SLAVE_ADDR, I2C_SR_DI) != I2C_READY);
			//Wait for reception completion
			RxComplete = RESET;
			while(RxComplete != SET);
			uint8_t x = 0;
			(void)x;
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


void I2C1_EV_IRQHandler(void){
	I2C_EV_IRQHandling(&i2cMaster);
}

void I2C1_ER_IRQHandler(void){
	I2C_ER_IRQHandling(&i2cMaster);
}

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t ApEv){
	uint8_t status = 0;
	if(ApEv == I2C_EV_TX_CMPLT){
		status = 1;
	}else if(ApEv == I2C_EV_RX_CMPLT){
		status = 2;
		RxComplete = SET;
	}else if(ApEv == I2C_ER_AF){
		status = 3;
		I2C_CloseTX(pI2CHandle);
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		while(1); //infinite loop
	}
	(void)status;
}
