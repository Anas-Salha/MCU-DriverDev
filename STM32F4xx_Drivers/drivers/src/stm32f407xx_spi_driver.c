/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Oct 20, 2019
 *      Author: AnaS9
 */

#include "stm32f407xx_spi_driver.h"

/*
 * Local helper functions
 */
static void SPI_TXE_Handler(SPI_Handle_t *pSPIHandle);
static void SPI_RXNE_Handler(SPI_Handle_t *pSPIHandle);
static void SPI_OVR_Handler(SPI_Handle_t *pSPIHandle);

/*Peripheral clock setup
 *
 */

/********************************************************************************
 *@fn			-	SPI_PCLK_Control
 *
 *@brief		-	Enables or disables peripheral clock for the given SPIx (x=1/2/3)
 *
 *@param[in]	-	SPI peripheral base address
 *@param[in]	-	ENABLE or DISABLE macros
 *@param[in]	-
 *
 *@return		-	none
 *
 *@Note			-

 */
void SPI_PCLK_Control(SPI_RegDef_t *pSPIx, uint8_t EnorDi){

	if(EnorDi == ENABLE){
		if(pSPIx == SPI1)
			SPI1_PCLK_EN();
		else if(pSPIx == SPI2)
			SPI2_PCLK_EN();
		else if(pSPIx == SPI3)
			SPI3_PCLK_EN();
	}else if (EnorDi == DISABLE){
		if(pSPIx == SPI1)
			SPI1_PCLK_DI();
		else if(pSPIx == SPI2)
			SPI2_PCLK_DI();
		else if(pSPIx == SPI3)
			SPI3_PCLK_DI();
	}
}

/*
 * Initialization & De-initialization
 */

/********************************************************************************
 *@fn			-	SPI_Init
 *
 *@brief		-	Initializes SPIx
 *
 *@param[in]	-	SPI peripheral handling structure address
 *@param[in]	-
 *@param[in]	-
 *
 *@return		-	none
 *
 *@Note			-

 */
void SPI_Init(SPI_Handle_t *pSPIHandle){

	uint32_t tempreg = 0;

	SPI_PCLK_Control(pSPIHandle->pSPIx, ENABLE); //enable clock for SPIx

	//1. Device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	//2. Bus configuration
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_FDUPLEX){
		//bidi mode cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_HDUPLEX){
		//bidi mode set
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_SIMPLEX_RX){
		//bidi mode cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		//rxonly bit set
		tempreg |= (1<<SPI_CR1_RXONLY);
	}

	//3. Data frame format
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	//4. Clock phase
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	//5. Clock polarity
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	//6. Slave select management
	tempreg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

	//7. Clock speed
	tempreg |= pSPIHandle->SPIConfig.SPI_SCLKSpeed << SPI_CR1_BR;

	//Adjust CR1 accordingly
	pSPIHandle->pSPIx->CR1 = tempreg;
}

/********************************************************************************
 *@fn			-	SPI_DeInit
 *
 *@brief		-	De-initializes SPIx
 *
 *@param[in]	-	SPI peripheral base address
 *@param[in]	-
 *@param[in]	-
 *
 *@return		-	none
 *
 *@Note			-

 */
void SPI_DeInit(SPI_RegDef_t *pSPIx){
	if(pSPIx == SPI1)
		SPI1_REG_RESET();
	else if(pSPIx == SPI2)
		SPI2_REG_RESET();
	else if(pSPIx == SPI3)
		SPI3_REG_RESET();
}


/*
 * Data transmit & receive
 */

/********************************************************************************
 *@fn			-	SPI_DataTX
 *
 *@brief		-	Transmits data using given SPIx
 *
 *@param[in]	-	SPI peripheral base address
 *@param[in]	-	Transmit buffer address
 *@param[in]	-	Transmitted message length
 *
 *@return		-	none
 *
 *@Note			-	Blocking call

 */
void SPI_DataTX(SPI_RegDef_t *pSPIx, uint8_t *pTXBuffer, uint32_t Len){

	//transmit data
	while(Len > 0){
		//1. wait until TXE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_FLAG_TXE) == FLAG_RESET);

		//2. check DFF bit in CR1
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF)){
			//16bit
			//3. load data to DR
			pSPIx->DR = *((uint16_t*)pTXBuffer);
			//4. decrement data length twice
			Len--;
			Len--;
			//5. increment TXBuffer twice
			(uint16_t*)pTXBuffer++;
		}else{
			//8bit
			//3. load data to DR
			pSPIx->DR = *pTXBuffer;
			//4. decrement data length
			Len--;
			//5. increment TXBuffer
			pTXBuffer++;
		}
	}
}

/********************************************************************************
 *@fn			-	SPI_DataRX
 *
 *@brief		-	Receives data using given SPIx
 *
 *@param[in]	-	SPI peripheral base address
 *@param[in]	-	Receive buffer address
 *@param[in]	-	Received message length
 *
 *@return		-	none
 *
 *@Note			-

 */
void SPI_DataRX(SPI_RegDef_t *pSPIx, uint8_t *pRXBuffer, uint32_t Len){

	//transmit data
	while(Len > 0){
		//1. wait until RXNE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_FLAG_RXNE) == FLAG_RESET);

		//2. check DFF bit in CR1
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF)){
			//16bit
			//3. load data from DR
			*((uint16_t*)pRXBuffer) = pSPIx->DR;
			//4. decrement data length twice
			Len--;
			Len--;
			//5. increment RXBuffer twice
			(uint16_t*)pRXBuffer++;
		}else{
			//8bit
			//3. load data from DR
			*pRXBuffer = pSPIx->DR;
			//4. decrement data length
			Len--;
			//5. increment RXBuffer
			pRXBuffer++;
		}
	}
}

/********************************************************************************
 *@fn			-	SPI_ITDataTX
 *
 *@brief		-	Transmits data using given SPIx
 *
 *@param[in]	-	SPI peripheral handling structure address
 *@param[in]	-	Transmit buffer address
 *@param[in]	-	Transmitted message length
 *
 *@return		-	none
 *
 *@Note			-	Non-blocking call

 */
uint8_t SPI_ITDataTX(SPI_Handle_t *pSPIHandle, uint8_t *pTXBuffer, uint32_t Len){

	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_TX){
		//1. Save TX buffer address and length info in global variables
		pSPIHandle->pTxBuffer = pTXBuffer;
		pSPIHandle->TxLen = Len;

		//2. Mark SPI state as busy
		pSPIHandle->TxState = SPI_BUSY_TX;

		//3. Enable TXEIE bit
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);

		//4. Data transmission will be handled in ISR
	}

	return state;
}

/********************************************************************************
 *@fn			-	SPI_ITDataRX
 *
 *@brief		-	Receives data using given SPIx
 *
 *@param[in]	-	SPI peripheral handling structure address
 *@param[in]	-	Transmit buffer address
 *@param[in]	-	Transmitted message length
 *
 *@return		-	none
 *
 *@Note			-	Non-blocking call

 */
uint8_t SPI_ITDataRX(SPI_Handle_t *pSPIHandle, uint8_t *pRXBuffer, uint32_t Len){

	uint8_t state = pSPIHandle->RxState;

	if(state != SPI_BUSY_RX){
		//1. Save Rx buffer address and length info in global variables
		pSPIHandle->pRxBuffer = pRXBuffer;
		pSPIHandle->RxLen = Len;

		//2. Mark SPI state as busy
		pSPIHandle->RxState = SPI_BUSY_RX;

		//3. Enable RXNEIE bit
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);

		//4. Data transmission will be handled in ISR
	}

	return state;
}

/*
 * IRQ configuration & ISR handling
 */

/********************************************************************************
 *@fn			-	SPI_IRQITConfig
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
void SPI_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi){
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
 *@fn			-	SPI_IRQPriorityConfig
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
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority){
	//1. Identify IRPx
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift = (iprx_section*8) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_IPR_BASEADDR + (iprx)) |= (IRQPriority << shift);
}

/********************************************************************************
 *@fn			-	SPI_IRQHandling
 *
 *@brief		-
 *
 *@param[in]	-	SPI
 *@param[in]	-
 *@param[in]	-
 *
 *@return		-	none
 *
 *@Note			-

 */
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle){

	uint8_t temp1, temp2;
	temp1 = pSPIHandle->pSPIx->SR & (1<<SPI_SR_TXE);
	temp2 = pSPIHandle->pSPIx->CR2 & (1<<SPI_CR2_TXEIE);
	if(temp1 && temp2){
		//handle TXE
		SPI_TXE_Handler(pSPIHandle);
	}

	temp1 = pSPIHandle->pSPIx->SR & (1<<SPI_SR_RXNE);
	temp2 = pSPIHandle->pSPIx->CR2 & (1<<SPI_CR2_RXNEIE);
	if(temp1 && temp2){
		//handle RXNE
		SPI_RXNE_Handler(pSPIHandle);
	}

	temp1 = pSPIHandle->pSPIx->SR & (1<<SPI_SR_OVR);
	temp2 = pSPIHandle->pSPIx->CR2 & (1<<SPI_CR2_ERRIE);
	if(temp1 && temp2){
		//handle OVR
		SPI_OVR_Handler(pSPIHandle);
	}
}


/*
 * Other peripheral control APIs
 */

/********************************************************************************
 *@fn			-	SPI_GetFlagStatus
 *
 *@brief		-	Gets flag status of given flag in SPIx SR register
 *
 *@param[in]	-	SPI peripheral base address
 *@param[in]	-	Macro from @SPI_FLAGNAME
 *@param[in]	-
 *
 *@return		-	Flag status (1 or 0)
 *
 *@Note			-

 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t Flagname){
	if(pSPIx->SR & Flagname)
		return FLAG_SET;
	return FLAG_RESET;
}

/********************************************************************************
 *@fn			-	SPI_PeripheralControl
 *
 *@brief		-	Enables or disables the given SPIx (x=1/2/3) peripheral
 *
 *@param[in]	-	SPI peripheral base address
 *@param[in]	-	ENABLE or DISABLE macros
 *@param[in]	-
 *
 *@return		-	none
 *
 *@Note			-

 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}
	else if(EnorDi == DISABLE){
		while(SPI_GetFlagStatus(pSPIx, SPI_FLAG_BSY));
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

/********************************************************************************
 *@fn			-	SPI_SSIConfig
 *
 *@brief		-	Sets/resets the SSI bit
 *
 *@param[in]	-	SPI peripheral base address
 *@param[in]	-	ENABLE or DISABLE macros
 *@param[in]	-
 *
 *@return		-	none
 *
 *@Note			-

 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
	if(EnorDi == ENABLE)
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	else if(EnorDi == DISABLE)
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
}

/********************************************************************************
 *@fn			-	SPI_SSOEConfig
 *
 *@brief		-	Sets/resets the SSOE bit
 *
 *@param[in]	-	SPI peripheral base address
 *@param[in]	-	ENABLE or DISABLE macros
 *@param[in]	-
 *
 *@return		-	none
 *
 *@Note			-

 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
	if(EnorDi == ENABLE)
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	else if(EnorDi == DISABLE)
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
}

/*
 * Helper functions implementations
 */
static void SPI_TXE_Handler(SPI_Handle_t *pSPIHandle){

	//1. check DFF bit in CR1
	if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)){
		//16bit
		//2. load data to DR
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		//3. decrement data length twice
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		//4. increment TXBuffer twice
		(uint16_t*)pSPIHandle->pTxBuffer++;
	}else{
		//8bit
		//2. load data to DR
		pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
		//3. decrement data length
		pSPIHandle->TxLen--;
		//4. increment TXBuffer
		pSPIHandle->pTxBuffer++;
	}
	if(pSPIHandle->TxLen == 0){
		SPI_CloseTX(pSPIHandle);
	}
}

static void SPI_RXNE_Handler(SPI_Handle_t *pSPIHandle){

	//1. check DFF bit in CR1
	if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)){
		//16bit
		//2. load data from DR
		*((uint16_t*)pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
		//3. decrement data length twice
		pSPIHandle->RxLen--;
		pSPIHandle->RxLen--;
		//4. increment RXBuffer twice
		(uint16_t*)pSPIHandle->pRxBuffer++;
	}else{
		//8bit
		//2. load data from DR
		*pSPIHandle->pRxBuffer = pSPIHandle->pSPIx->DR;
		//3. decrement data length
		pSPIHandle->RxLen--;
		//4. increment RXBuffer
		pSPIHandle->pRxBuffer++;
	}

	if(pSPIHandle->RxLen == 0){
		SPI_CloseRX(pSPIHandle);
	}
}

static void SPI_OVR_Handler(SPI_Handle_t *pSPIHandle){

	//1. clear OVR
	if(pSPIHandle->TxState != SPI_BUSY_TX){
		uint8_t temp;
		uint8_t temp_avoiderr;
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;

		//just to avoid warnings
		temp_avoiderr = temp;
		temp = temp_avoiderr;
	}

	//2. inform application
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}

void SPI_CloseTX(SPI_Handle_t *pSPIHandle){
	//close SPI communication
	pSPIHandle->pSPIx->CR2 &= ~(1<<SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxState = SPI_READY;
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
}

void SPI_CloseRX(SPI_Handle_t *pSPIHandle){
	//close SPI communication
	pSPIHandle->pSPIx->CR2 &= ~(1<<SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxState = SPI_READY;
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
}

void SPI_ClearOVR(SPI_Handle_t *pSPIHandle){
	uint8_t temp;
	temp = pSPIHandle->pSPIx->DR;
	temp = pSPIHandle->pSPIx->SR;

	//just to avoid warnings
	(void)temp;
}


/*
 * Application callback
 */
__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t ApEv){
	//Weak implementation
}
