/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: Oct 27, 2019
 *      Author: AnaS9
 */

#include "stm32f407xx_i2c_driver.h"

/*
 * Local helper functions
 */


static void I2C_ExecuteAddrPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr, uint8_t WorR);
static void I2C_ClearAddrFlag(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle);


/*
 * Peripheral clock setup
 */

/********************************************************************************
 *@fn			-	I2C_PCLK_Control
 *
 *@brief		-	Enables or disables peripheral clock for the given I2Cx (x=1/2/3)
 *
 *@param[in]	-	I2C peripheral base address
 *@param[in]	-	ENABLE or DISABLE macros
 *@param[in]	-
 *
 *@return		-	none
 *
 *@Note			-

 */
void I2C_PCLK_Control(I2C_RegDef_t *pI2Cx, uint8_t EnorDi){

	if(EnorDi == ENABLE){
		if(pI2Cx == I2C1)
			I2C1_PCLK_EN();
		else if(pI2Cx == I2C2)
			I2C2_PCLK_EN();
		else if(pI2Cx == I2C3)
			I2C3_PCLK_EN();
	}else if (EnorDi == DISABLE){
		if(pI2Cx == I2C1)
			I2C1_PCLK_DI();
		else if(pI2Cx == I2C2)
			I2C2_PCLK_DI();
		else if(pI2Cx == I2C3)
			I2C3_PCLK_DI();
	}
}


/*
 * Initialization & De-initialization
 */

/********************************************************************************
 *@fn			-	I2C_Init
 *
 *@brief		-	Initializes I2Cx
 *
 *@param[in]	-	I2C peripheral handling structure address
 *@param[in]	-
 *@param[in]	-
 *
 *@return		-	none
 *
 *@Note			-

 */
void I2C_Init(I2C_Handle_t *pI2CHandle){
	uint32_t tempreg = 0;
	uint32_t pclk1_value = 0;
	uint16_t ccr_value = 0;

	//1. Enable peripheral clock
	I2C_PCLK_Control(pI2CHandle->pI2Cx, ENABLE);

	//2. Configure clock speed
	//2.1. Configure I2C_CR2_FREQ
	pclk1_value = RCC_GetPCLK1Value();
	tempreg = 0;
	tempreg |= pclk1_value/1000000U;
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F);

	//2.2. Configure I2C_CCR_CCR
	tempreg = 0;
	if(pI2CHandle->I2CConfig.I2C_SCLKSpeed <= I2C_SPEED_SM){
		//standard mode
		ccr_value = (pclk1_value / (2 * pI2CHandle->I2CConfig.I2C_SCLKSpeed));
	}else{
		//fast mode
		tempreg |= (1 << I2C_CCR_FS);
		tempreg |= (pI2CHandle->I2CConfig.I2C_FMDutyCycle << I2C_CCR_DUTY);

		if(pI2CHandle->I2CConfig.I2C_FMDutyCycle == I2C_FM_DUTY2){
			ccr_value = (pclk1_value / (3 * pI2CHandle->I2CConfig.I2C_SCLKSpeed));
		}else if(pI2CHandle->I2CConfig.I2C_FMDutyCycle == I2C_FM_DUTY16_9){
			ccr_value = (pclk1_value / (25 * pI2CHandle->I2CConfig.I2C_SCLKSpeed));
		}
	}
	tempreg |= (ccr_value & 0xFFF);
	pI2CHandle->pI2Cx->CCR = tempreg;

	//3. Configure device address
	tempreg = 0;
	tempreg |= pI2CHandle->I2CConfig.I2C_DeviceAddr << 1;
	tempreg |= (1<<14);
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	//4. Configure rise time
	if(pI2CHandle->I2CConfig.I2C_SCLKSpeed <= I2C_SPEED_SM)
		tempreg = (pclk1_value / 1000000U) + 1;
	else
		tempreg = ((0.3 * pclk1_value) / 1000000U) + 1;

	pI2CHandle->pI2Cx->TRISE = tempreg & 0x3F;
}

/********************************************************************************
 *@fn			-	I2C_DeInit
 *
 *@brief		-	De-initializes I2Cx
 *
 *@param[in]	-	I2C peripheral base address
 *@param[in]	-
 *@param[in]	-
 *
 *@return		-	none
 *
 *@Note			-

 */
void I2C_DeInit(I2C_RegDef_t *pI2Cx){
	if(pI2Cx == I2C1)
		I2C1_REG_RESET();
	else if(pI2Cx == I2C2)
		I2C2_REG_RESET();
	else if(pI2Cx == I2C3)
		I2C3_REG_RESET();
}


/*
 * Data transmit & receive
 */

/********************************************************************************
 *@fn			-	I2C_MasterTX
 *
 *@brief		-	Transmits data in master mode
 *
 *@param[in]	-	I2C peripheral handling structure address
 *@param[in]	-	Transmit buffer address
 *@param[in]	-	Transmitted message length
 *@param[in]	-	Slave receiver address
 *@param[in]	- 	Repeated start state
 *
 *@return		-	none
 *
 *@Note			-	Blocking call

 */
void I2C_MasterTX(I2C_Handle_t *pI2CHandle, uint8_t *pTXBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr){
	//1. Generate START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. Wait for [EV5], ie. SB = 1
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB, I2C_SR1));

	//3. Send address of the slave & the r/nw bit
	I2C_ExecuteAddrPhase(pI2CHandle->pI2Cx, SlaveAddr, WRITE);

	//4. Wait for [EV6], ie. ADDR = 1
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR, I2C_SR1));

	//5. Clear ADDR flag
	I2C_ClearAddrFlag(pI2CHandle);

	//6. Send the data until Len becomes 0
	while(Len > 0){
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE, I2C_SR1));
		pI2CHandle->pI2Cx->DR = *pTXBuffer; //write data to DR
		pTXBuffer++; //increment transmit buffer
		Len--; //decrement length
	}

	//7. Wait for [EV8_2], ie. BTF = 1, TXE = 1
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE, I2C_SR1));
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF, I2C_SR1));

	//8. Generate STOP condition
	if(Sr == I2C_SR_DI){
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx); //Generate STOP condition
	}

}

/********************************************************************************
 *@fn			-	I2C_MasterRX
 *
 *@brief		-	Receives data in master mode
 *
 *@param[in]	-	I2C peripheral handling structure address
 *@param[in]	-	Receive buffer address
 *@param[in]	-	Received message length
 *@param[in]	-	Slave transmitter address
 *@param[in]	-	Repeated start state
 *
 *@return		-	none
 *
 *@Note			-	Blocking call

 */
void I2C_MasterRX(I2C_Handle_t *pI2CHandle, uint8_t *pRXBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr){

	//1. Generate START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. Wait for [EV5], ie. SB = 1
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB, I2C_SR1));

	//3. Send address of the slave & the r/nw bit
	I2C_ExecuteAddrPhase(pI2CHandle->pI2Cx, SlaveAddr, READ);

	//4. Wait for [EV6], ie. ADDR = 1
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR, I2C_SR1));


	if(Len == 1){
		I2C_ManageACK(pI2CHandle->pI2Cx, I2C_ACK_DI); //Disable ACKing

		//5. Clear ADDR flag
		I2C_ClearAddrFlag(pI2CHandle);

		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE, I2C_SR1)); //Wait for RXNE bit

		if(Sr == I2C_SR_DI){
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx); //Generate STOP condition
		}

		//6. read data from DR
		*pRXBuffer = pI2CHandle->pI2Cx->DR;

	}else if(Len > 1){
		//5. Clear ADDR flag
		I2C_ClearAddrFlag(pI2CHandle);

		//6. Receive data
		while(Len > 0){
			while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE, I2C_SR1));  //Wait for RXNE bit
			if(Len == 2){
				I2C_ManageACK(pI2CHandle->pI2Cx, I2C_ACK_DI); //Disable ACKing
				if(Sr == I2C_SR_DI){
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx); //Generate STOP condition
				}
			}
			*pRXBuffer = pI2CHandle->pI2Cx->DR; //read data from DR
			pRXBuffer++;//increment receive buffer
			Len--; //decrement length
		}
	}

	//7. Re-enable ACK
	if(pI2CHandle->I2CConfig.I2C_ACKCtrl == I2C_ACK_EN){
		I2C_ManageACK(pI2CHandle->pI2Cx, I2C_ACK_EN);
	}
}

/********************************************************************************
 *@fn			-	I2C_MasterTX_IT
 *
 *@brief		-	Transmits data in master mode
 *
 *@param[in]	-	I2C peripheral handling structure address
 *@param[in]	-	Transmit buffer address
 *@param[in]	-	Transmitted message length
 *@param[in]	-	Slave receiver address
 *@param[in]	-	Repeated start state
 *
 *@return		-	Peripheral state (macros from @I2C_APP_STATES)
 *
 *@Note			-	Non-blocking call (interrupt)

 */
uint8_t I2C_MasterTX_IT(I2C_Handle_t *pI2CHandle, uint8_t *pTXBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr){

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_TX) && (busystate != I2C_BUSY_RX)){
		pI2CHandle->pTxBuffer = pTXBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//1. Generate START condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//2. Enable Interrupt Control Bits
		I2C_InterruptControlBits(pI2CHandle->pI2Cx, ENABLE);
	}

	return busystate;
}

/********************************************************************************
 *@fn			-	I2C_MasterRX_IT
 *
 *@brief		-	Receives data in master mode
 *
 *@param[in]	-	I2C peripheral handling structure address
 *@param[in]	-	Transmit buffer address
 *@param[in]	-	Transmitted message length
 *@param[in]	-	Slave receiver address
 *@param[in]	-	Repeated start state
 *
 *@return		-	none
 *
 *@Note			-	Non-blocking call (interrupt)

 */
uint8_t I2C_MasterRX_IT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr)
{

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_TX) && (busystate != I2C_BUSY_RX)){
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_RX;
		pI2CHandle->RxSize = Len; //Rxsize is used in the ISR code to manage the data reception
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//1. Generate START condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//2. Enable Interrupt Control Bits
		I2C_InterruptControlBits(pI2CHandle->pI2Cx, ENABLE);
	}

	return busystate;
}

/********************************************************************************
 *@fn			-	I2C_SlaveTX
 *
 *@brief		-	Transmits data in slave mode
 *
 *@param[in]	-	I2Cx (x=1,2,3) peripheral base address
 *@param[in]	-	Data to transmit
 *@param[in]	-
 *@param[in]	-
 *
 *@return		-	none
 *
 *@Note			-

 */
void I2C_SlaveTX(I2C_RegDef_t *pI2Cx, uint8_t data){
	pI2Cx->DR = data;
}

/********************************************************************************
 *@fn			-	I2C_SlaveTX
 *
 *@brief		-	Receivess data in slave mode
 *
 *@param[in]	-	I2Cx (x=1,2,3) peripheral base address
 *@param[in]	-
 *@param[in]	-
 *@param[in]	-
 *
 *@return		-	Received data
 *
 *@Note			-

 */
uint8_t I2C_SlaveRX(I2C_RegDef_t *pI2Cx){
	return (uint8_t)pI2Cx->DR;
}

/*
 * IRQ configuration & ISR handling
 */

/********************************************************************************
 *@fn			-	I2C_IRQITConfig
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
void I2C_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi){
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
 *@fn			-	I2C_IRQPriorityConfig
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
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority){
	//1. Identify IRPx
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift = (iprx_section*8) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_IPR_BASEADDR + (iprx)) |= (IRQPriority << shift);
}

/********************************************************************************
 *@fn			-	I2C_EV_IRQHandling
 *
 *@brief		-	Handles interrupts caused by events
 *
 *@param[in]	-	I2C peripheral handling structure address
 *@param[in]	-
 *@param[in]	-
 *
 *@return		-	none
 *
 *@Note			-

 */
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle){

	uint32_t temp1, temp2, temp3;
	uint8_t WorR;

	temp1 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN);
	temp2 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN);

	//1. Interrupt generated by SB
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB);
	if(temp1 && temp3){
		//Execute address phase
		if(pI2CHandle->TxRxState == I2C_BUSY_TX)
			WorR = WRITE;
		else if(pI2CHandle->TxRxState == I2C_BUSY_RX)
			WorR = READ;
		I2C_ExecuteAddrPhase(pI2CHandle->pI2Cx, pI2CHandle->DevAddr, WorR);
	}

	//2. Interrupt generated by ADDR
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR);
	if(temp1 && temp3){
		//Clear ADDR flag
		I2C_ClearAddrFlag(pI2CHandle);
	}

	//3. Interrupt generated by STOPF
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_STOPF);
	if(temp1 && temp3){
		//Clear STOPF flag
		/// Cleared by 1) reading SR1 (already done) and 2) writing to CR1
		pI2CHandle->pI2Cx->CR1 |= 0x0000;

		//Notify application that STOP is detected
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
	}

	//4. Interrupt generated by BTF
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF);
	if(temp1 && temp3){
		if(pI2CHandle->TxRxState == I2C_BUSY_TX){
			//Make sure TXE is set
			if(pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE)){

				//Make sure LEN is 0
				if(pI2CHandle->TxLen == 0)
				{
					//1. Generate STOP condition
					if(pI2CHandle->Sr == I2C_SR_DI)
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

					//2. Reset all member elements of the handle structure
					I2C_CloseTX(pI2CHandle);

					//3. Notify application that transmission is complete
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
				}
			}
		}
	}

	//5. Interrupt generated by TXE
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE);
	if(temp1 && temp2 && temp3){
		//check for device mode (Master/Slave)
		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)){
			//Master mode
			//check if busy in transmission
			if(pI2CHandle->TxRxState == I2C_BUSY_TX){
				I2C_MasterHandleTXEInterrupt(pI2CHandle);
			}
		}else{
			//Slave mode
			//check if device is in transmitter mode
			if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA))
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
		}
	}

	//6. Interrupt generated by RXNE
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RXNE);
	if(temp1 && temp2 && temp3){

		//check for device mode (master/slave)
		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)){
			//check if busy in reception
			if(pI2CHandle->TxRxState == I2C_BUSY_RX){
				I2C_MasterHandleRXNEInterrupt(pI2CHandle);

			}
		}else{
			//Slave mode
			//check if device is in receiver mode
			if(! (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA)))
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
		}
	}

}

/********************************************************************************
 *@fn			-	I2C_ER_IRQHandling
 *
 *@brief		-	Handles interrupts caused by errors
 *
 *@param[in]	-	I2C peripheral handling structure address
 *@param[in]	-
 *@param[in]	-
 *
 *@return		-	none
 *
 *@Note			-

 */
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle){
	uint32_t temp1, temp2;

	temp1 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITERREN);

	/***********************Check for Bus error************************************/
	temp2 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BERR);
	if(temp1 && temp2){
		//1. Clear BERR flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_BERR);

		//2. Notify application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ER_BERR);
	}

	/***********************Check for arbitration lost error************************************/
	temp2 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ARLO);
	if(temp1 && temp2){
		//1. Clear ARLO flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_ARLO);

		//2. Notify application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ER_ARLO);
	}

	/***********************Check for ACK failure  error************************************/
	temp2 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_AF);
	if(temp1 && temp2){
		//1. Clear AF flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_AF);

		//2. Notify application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ER_AF);
	}

	/***********************Check for Overrun/underrun error************************************/
	temp2 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_OVR);
	if(temp1 && temp2){
		//1. Clear OVR flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_OVR);

		//2. Notify application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ER_OVR);
	}

	/***********************Check for Timeout error************************************/
	temp2 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TIMEOUT);
	if(temp1 && temp2){
		//1. Clear TIMEOUT flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_TIMEOUT);

		//2. Notify application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ER_TIMEOUT);
	}
}


/*
 * Other peripheral control APIs
 */

/********************************************************************************
 *@fn			-	I2C_GetFlagStatus
 *
 *@brief		-	Gets flag status of given flag in I2Cx SR register
 *
 *@param[in]	-	I2C peripheral base address
 *@param[in]	-	Macro from @I2C_FLAGNAME
 *@param[in]	-	Macro from @I2C_SR
 *
 *@return		-	Flag status (1 or 0)
 *
 *@Note			-

 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t Flagname, uint8_t Reg){
	if(Reg == I2C_SR1){
		if(pI2Cx->SR1 & Flagname)
			return FLAG_SET;
	}else if(Reg == I2C_SR2){
		if(pI2Cx->SR2 & Flagname)
			return FLAG_SET;
	}

	return FLAG_RESET;
}

/********************************************************************************
 *@fn			-	I2C_PeripheralControl
 *
 *@brief		-	Enables or disables the given I2Cx (x=1/2/3) peripheral
 *
 *@param[in]	-	I2C peripheral base address
 *@param[in]	-	ENABLE or DISABLE macros
 *@param[in]	-
 *
 *@return		-	none
 *
 *@Note			-

 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	}
	else if(EnorDi == DISABLE){
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}


/********************************************************************************
 *@fn			-	I2C_ManageACK
 *
 *@brief		-	Enables or disables ACKing for the given I2Cx (x=1/2/3) peripheral
 *
 *@param[in]	-	I2C peripheral base address
 *@param[in]	-	ENABLE or DISABLE macros
 *@param[in]	-
 *
 *@return		-	none
 *
 *@Note			-

 */
void I2C_ManageACK(I2C_RegDef_t *pI2Cx, uint8_t EnorDi){
	if(EnorDi == I2C_ACK_EN){
		pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	}else if(EnorDi == I2C_ACK_DI){
		pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
	}
}

/********************************************************************************
 *@fn			-	I2C_CloseTX
 *
 *@brief		-	Closes I2C interrupt based transmission
 *
 *@param[in]	-	I2C handler structure address
 *@param[in]	-
 *@param[in]	-
 *
 *@return		-	none
 *
 *@Note			-

 */
void I2C_CloseTX(I2C_Handle_t *pI2CHandle){
	//1. Disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	//2. Disable ITEVTEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

	//3. Reset I2C handle member elements to default values
	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;

	//4. Enable ACKing
	if(pI2CHandle->I2CConfig.I2C_ACKCtrl == I2C_ACK_EN)
		I2C_ManageACK(pI2CHandle->pI2Cx, ENABLE);
}

/********************************************************************************
 *@fn			-	I2C_CloseRX
 *
 *@brief		-	Closes I2C interrupt based reception
 *
 *@param[in]	-	I2C handler structure address
 *@param[in]	-
 *@param[in]	-
 *
 *@return		-	none
 *
 *@Note			-

 */
void I2C_CloseRX(I2C_Handle_t *pI2CHandle){
	//1. Disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	//2. Disable ITEVTEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

	//3. Reset I2C handle member elements to default values
	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	//4. Enable ACKing
	if(pI2CHandle->I2CConfig.I2C_ACKCtrl == I2C_ACK_EN)
		I2C_ManageACK(pI2CHandle->pI2Cx, ENABLE);
}

/********************************************************************************
 *@fn			-	I2C_GenerateStartCondition
 *
 *@brief		-	Generates I2C START condition
 *
 *@param[in]	-	I2Cx (x=1,2,3) peripheral base address
 *@param[in]	-
 *@param[in]	-
 *
 *@return		-	none
 *
 *@Note			-

 */
void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx){
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

/********************************************************************************
 *@fn			-	I2C_GenerateStopCondition
 *
 *@brief		-	Generates I2C STOP condition
 *
 *@param[in]	-	I2Cx (x=1,2,3) peripheral base address
 *@param[in]	-
 *@param[in]	-
 *
 *@return		-	none
 *
 *@Note			-

 */
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx){
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

/********************************************************************************
 *@fn			-	I2C_InterruptControlBits
 *
 *@brief		-	Enables/disables interrup control bits
 *
 *@param[in]	-	I2Cx (x=1,2,3) peripheral base address
 *@param[in]	-	ENABLE or DISABLE macros
 *@param[in]	-
 *
 *@return		-	none
 *
 *@Note			-

 */
void I2C_InterruptControlBits(I2C_RegDef_t *pI2Cx, uint8_t EnorDi){

	if(EnorDi == ENABLE){
		pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN); //Enable ITBUFEN Control Bit
		pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN); //Enable ITEVTEN Control Bit
		pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN); //Enable ITERREN Control Bit

	}else if(EnorDi == DISABLE){
		pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN); //Disable ITBUFEN Control Bit
		pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN); //Disable ITEVTEN Control Bit
		pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITERREN); //Disable ITERREN Control Bit
	}
}

/*
 * Helper functions implementations
*/

static void I2C_ExecuteAddrPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr, uint8_t WorR){
	SlaveAddr = SlaveAddr << 1;
	if(WorR == WRITE)
		SlaveAddr &= ~(1);
	else if (WorR == READ)
		SlaveAddr |= (1);

	pI2Cx->DR = SlaveAddr;
}

static void I2C_ClearAddrFlag(I2C_Handle_t *pI2CHandle){
	uint32_t dummyread;

	//check for device mode (Master/Slave)
	if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)){
		//Master mode
		if(pI2CHandle->TxRxState == I2C_BUSY_RX){
			//Reception mode
			if(pI2CHandle->RxSize == 1)
				I2C_ManageACK(pI2CHandle->pI2Cx, I2C_ACK_DI); //disable ACK
		}
	}

	//Clear ADDR
	dummyread = pI2CHandle->pI2Cx->SR1;
	dummyread = pI2CHandle->pI2Cx->SR2;

	(void)dummyread;
}

static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle){

	//Case 1: 1-byte transfer
	if(pI2CHandle->RxSize == 1){
		*(pI2CHandle->pRxBuffer) = pI2CHandle->pI2Cx->DR; //Read data from DR
		pI2CHandle->RxLen--; //decrement Rx length
	}

	//Case 2: multiple bytes transfer
	if(pI2CHandle->RxSize > 1){

			if(pI2CHandle->RxLen == 2)
				I2C_ManageACK(pI2CHandle->pI2Cx, DISABLE);

			*(pI2CHandle->pRxBuffer) = pI2CHandle->pI2Cx->DR; //Read data from DR
			pI2CHandle->pRxBuffer++; //increment rx buffer address
			pI2CHandle->RxLen--; //decrement Rx length
	}

	if(pI2CHandle->RxLen == 0){
		//1. Generate STOP condition
		if(pI2CHandle->Sr == I2C_SR_DI)
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		//2. Reset all member elements of the handle structure
		I2C_CloseRX(pI2CHandle);

		//3. Notify application that reception is complete
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
	}
}

static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle){
	//check if there is still data to send
	if(pI2CHandle->TxLen > 0){

		//1. Write data to DR
		pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);

		//2. Update handler
		pI2CHandle->pTxBuffer++; //increment tx buffer address
		pI2CHandle->TxLen--; //decrement tx length
	}
}


/*
 * Application callback
 */
__weak void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t ApEv){
	//Weak implementation
}

