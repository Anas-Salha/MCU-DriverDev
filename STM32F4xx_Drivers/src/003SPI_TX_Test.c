/*
 * 003SPI_TX_Test.c
 *
 *  Created on: Oct 21, 2019
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

void SPI2_GPIOInits(void);
void SPI2_Inits(void);

int main(void){

	char user_data[] = "Hello World!";
	SPI2_GPIOInits(); //initialize required SPI2 gpio pins
	SPI2_Inits(); //initialize SPI2
	SPI_SSIConfig(SPI2, ENABLE); //pulls NSS high internally (avoid MODF error)
	SPI_DataTX(SPI2, (uint8_t *)user_data, strlen(user_data));

	return 0;
}

void SPI2_GPIOInits(void){
	GPIO_Handle_t SPIPins;
	memset(&SPIPins, 0, sizeof(SPIPins)); //clear all members of the structure
	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	//NSS
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	//GPIO_Init(&SPIPins);

	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GPIO_Init(&SPIPins);

	//MISO
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;
	//GPIO_Init(&SPIPins);

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
	spi2Master.SPIConfig.SPI_SCLKSpeed = SPI_SCLK_SPEED_DIV2;
	spi2Master.SPIConfig.SPI_DFF = SPI_DFF_8BIT;
	spi2Master.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	spi2Master.SPIConfig.SPI_CPHA = SPI_CPHA_EDGE1;
	spi2Master.SPIConfig.SPI_SSM = SPI_SSM_EN;

	SPI_Init(&spi2Master);
}
