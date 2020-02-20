/*
 * 005SPI_CMD_Handling.c
 *
 *  Created on: Oct 23, 2019
 *      Author: AnaS9
 */


/*
 * PB12 -> SPI2_NSS
 * PB13 -> SPI2_SCLK
 * PB14 -> SPI2_MISO
 * PB15 -> SPI2_MOSI
 * AF5
 */

#include "stm32f407xx.h"
#include <stdint.h>
#include <string.h>

/*
 * Command codes
 */
#define CMD_LED_CTRL			0x50
#define CMD_SENSOR_READ			0x51
#define CMD_LED_READ			0x52
#define CMD_PRINT				0x53
#define CMD_ID_READ				0x54

/*
 * Generic macros
 */
#define LED_OFF					0
#define LED_ON					1
#define LED_PIN					9
#define ACK						0xF5
#define NACK					0xA5

/*
 * Arduino analog pins
 */
#define ANALOG_PIN0				0
#define ANALOG_PIN1				1
#define ANALOG_PIN2				2
#define ANALOG_PIN3				3
#define ANALOG_PIN4				4


/*
 * Function Prototypes
 */
void GPIO_ButtonInit(void);
void SPI2_GPIOInits(void);
void SPI2_Inits(void);
uint8_t SPI_VerifyACK(uint8_t ackbyte);
void delay(void);

int main(void){

	uint8_t i = 0;
	uint8_t dummywrite = 0xff;
	uint8_t dummyread;
	uint8_t ackbyte;
	uint8_t args[2];
	uint8_t cmdcode;

	GPIO_ButtonInit(); //initialize user button
	SPI2_GPIOInits(); //initialize required SPI2 gpio pins
	SPI2_Inits(); //initialize SPI2
	SPI_SSOEConfig(SPI2, ENABLE); //enable SPI_CR2_SSOE bit
	SPI_PeripheralControl(SPI2, ENABLE); //enable SPI2

	while(1){
		//for(i=0; i<=1; i++){
		if(GPIO_InPinRead(GPIOA, USER_BUTTON) == 1){
			while(GPIO_InPinRead(GPIOA, USER_BUTTON));
			if(i==0){
				//1. CMD_LED_CTRL	<pin no>	<value>
				cmdcode = CMD_LED_CTRL;
				SPI_DataTX(SPI2, &cmdcode, 1); //send command
				SPI_DataRX(SPI2, &dummyread, 1); //dummy read
				SPI_DataTX(SPI2, &dummywrite, 1); //dummy write
				SPI_DataRX(SPI2, &ackbyte, 1); //ack read
				//send arguments
				if(SPI_VerifyACK(ackbyte)){
					args[0] = LED_PIN;
					args[1] = LED_ON;
					SPI_DataTX(SPI2, args, 2);
					i++;
				}

			}else if(i==1){
				//2. CMD_SENSOR_READ	<analog pin number>
				cmdcode = CMD_SENSOR_READ;
				SPI_DataTX(SPI2, &cmdcode, 1); //send command
				SPI_DataRX(SPI2, &dummyread, 1); //dummy read
				SPI_DataTX(SPI2, &dummywrite, 1); //dummy write
				SPI_DataRX(SPI2, &ackbyte, 1); //ack read

				//send arguments
				if(SPI_VerifyACK(ackbyte)){
					memset(args, 0, sizeof(args));
					args[0] = ANALOG_PIN0;
					SPI_DataTX(SPI2, args, 1);
					delay(); //delay
					SPI_DataRX(SPI2, &dummyread, 1); //dummy read
					SPI_DataTX(SPI2, &dummywrite, 1); //dummy write
					//receive data
					uint8_t analog_read;
					SPI_DataRX(SPI2, &analog_read, 1);
					i++; //increment counter
				}

			}else if(i==2){
				//3. CMD_LED_READ	<pin no>
				cmdcode = CMD_LED_READ;
				SPI_DataTX(SPI2, &cmdcode, 1); //send command
				SPI_DataRX(SPI2, &dummyread, 1); //dummy read
				SPI_DataTX(SPI2, &dummywrite, 1); //dummy write
				SPI_DataRX(SPI2, &ackbyte, 1); //ack read

				//send arguments
				if(SPI_VerifyACK(ackbyte)){
					memset(args, 0, sizeof(args));
					args[0] = LED_PIN;
					SPI_DataTX(SPI2, args, 1);
					delay();
					SPI_DataRX(SPI2, &dummyread, 1); //dummy read
					SPI_DataTX(SPI2, &dummywrite, 1); //dummy write

					//receive data
					uint8_t led_read;
					SPI_DataRX(SPI2, &led_read, 1);
					i++;
				}
			}else if(i==3){
				//4. CMD_PRINT	<len>	<message>
				cmdcode = CMD_PRINT;
				SPI_DataTX(SPI2, &cmdcode, 1); //send command
				SPI_DataRX(SPI2, &dummyread, 1); //dummy read
				SPI_DataTX(SPI2, &dummywrite, 1); //dummy write
				SPI_DataRX(SPI2, &ackbyte, 1); //ack read

				if(SPI_VerifyACK(ackbyte)){
					char message[] = "How you doin'?";
					memset(args, 0, sizeof(args));
					args[0] = strlen(message);
					SPI_DataTX(SPI2, &args[0], 1);
					SPI_DataTX(SPI2, (uint8_t *)message, args[0]);
					i++;
				}

			}else if(i==4){
				//5. CMD_ID_READ
				cmdcode = CMD_ID_READ;
				SPI_DataTX(SPI2, &cmdcode, 1); //send command
				SPI_DataRX(SPI2, &dummyread, 1); //dummy read
				SPI_DataTX(SPI2, &dummywrite, 1); //dummy write
				SPI_DataRX(SPI2, &ackbyte, 1); //ack read

				if(SPI_VerifyACK(ackbyte)){
					SPI_DataTX(SPI2, &dummywrite, 1); //dummy write
					//receive data
					uint8_t ID_read[10];
					for(i=0; i<10; i++){
						SPI_DataTX(SPI2, &dummywrite, 1);
						SPI_DataRX(SPI2, &ID_read[i], 1);
					}
					i=0; //reset counter
				}
			}
		}
	}
	SPI_PeripheralControl(SPI2, DISABLE); //disable SPI2
	return 0;
}

void GPIO_ButtonInit(void){
	GPIO_Handle_t gpioB1;
	memset(&gpioB1, 0, sizeof(gpioB1)); //clear all members of the structure
	gpioB1.pGPIOx = GPIOA;
	gpioB1.GPIO_PinConfig.GPIO_PinNumber = USER_BUTTON;
	gpioB1.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIO_Init(&gpioB1);
}

void SPI2_GPIOInits(void){
	GPIO_Handle_t SPIPins;
	memset(&SPIPins, 0, sizeof(SPIPins)); //clear all members of the structure
	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;

	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GPIO_Init(&SPIPins);

	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GPIO_Init(&SPIPins);

	//MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15;
	GPIO_Init(&SPIPins);
}

void SPI2_Inits(void){

	SPI_Handle_t spi2Master;
	memset(&spi2Master, 0, sizeof(spi2Master)); //clear all members of the structure
	spi2Master.pSPIx = SPI2;
	spi2Master.SPIConfig.SPI_BusConfig = SPI_BUS_FDUPLEX;
	spi2Master.SPIConfig.SPI_DeviceMode = SPI_MODE_MASTER;
	spi2Master.SPIConfig.SPI_SCLKSpeed = SPI_SCLK_SPEED_DIV8;
	spi2Master.SPIConfig.SPI_DFF = SPI_DFF_8BIT;
	spi2Master.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	spi2Master.SPIConfig.SPI_CPHA = SPI_CPHA_EDGE1;
	spi2Master.SPIConfig.SPI_SSM = SPI_SSM_DI;

	SPI_Init(&spi2Master);
}


uint8_t SPI_VerifyACK(uint8_t ackbyte){
	if(ackbyte == ACK)
		return 1;
	return 0;
}



