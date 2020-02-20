/*
 * stm32f407xx_usart_driver.c
 *
 *  Created on: Nov 6, 2019
 *      Author: AnaS9
 */

#include "stm32f407xx_USART_driver.h"

/*
 * Peripheral clock setup
 */

/********************************************************************************
 *@fn			-	USART_PCLK_Control
 *
 *@brief		-	Enables or disables peripheral clock for the given USARTx (x=1/2/3/4/5/6)
 *
 *@param[in]	-	USART peripheral base address
 *@param[in]	-	ENABLE or DISABLE macros
 *@param[in]	-
 *
 *@return		-	none
 *
 *@Note			-

 */
void USART_PCLK_Control(USART_RegDef_t *pUSARTx, uint8_t EnorDi){

	if(EnorDi == ENABLE){
		if(pUSARTx == USART1)
			USART1_PCLK_EN();
		else if(pUSARTx == USART2)
			USART2_PCLK_EN();
		else if(pUSARTx == USART3)
			USART3_PCLK_EN();
		else if(pUSARTx == UART4)
			UART4_PCLK_EN();
		else if(pUSARTx == UART5)
			UART5_PCLK_EN();
		else if(pUSARTx == USART6)
			USART6_PCLK_EN();

	}else if (EnorDi == DISABLE){
		if(pUSARTx == USART1)
			USART1_PCLK_DI();
		else if(pUSARTx == USART2)
			USART2_PCLK_DI();
		else if(pUSARTx == USART3)
			USART3_PCLK_DI();
		else if(pUSARTx == UART4)
			UART4_PCLK_DI();
		else if(pUSARTx == UART5)
			UART5_PCLK_DI();
		else if(pUSARTx == USART6)
			USART6_PCLK_DI();
	}
}


/*
 * Initialization & De-initialization
 */

/********************************************************************************
 *@fn			-	USART_Init
 *
 *@brief		-	Initializes USARTx
 *
 *@param[in]	-	USART peripheral handling structure address
 *@param[in]	-
 *@param[in]	-
 *
 *@return		-	none
 *
 *@Note			-

 */
void USART_Init(USART_Handle_t *pUSARTHandle){


	//Temporary variable
	uint32_t tempreg=0;

/******************************** Configuration of CR1******************************************/

	//Enable clock for given USART peripheral
	 USART_PCLK_Control(pUSARTHandle->pUSARTx, ENABLE);

	//Configure USART mode
	if (pUSARTHandle->USARTConfig.USART_Mode == USART_MODE_RX){
		//Enable Rx
		tempreg |= (1 << USART_CR1_RE);

	}else if (pUSARTHandle->USARTConfig.USART_Mode == USART_MODE_TX){
		//Enable Tx
		tempreg |= ( 1 << USART_CR1_TE );

	}else if (pUSARTHandle->USARTConfig.USART_Mode == USART_MODE_TXRX){
		//Enable Tx & Rx
		tempreg |= (1 << USART_CR1_RE);
		tempreg |= (1 << USART_CR1_TE);
	}

    //Configure the word length
	tempreg |= (pUSARTHandle->USARTConfig.USART_WordLen << USART_CR1_M);


    //Configuration of parity control bit fields
	if ( pUSARTHandle->USARTConfig.USART_ParityCtrl == USART_PARITY_EVEN){
		//Enable parity control
		tempreg |= ( 1 << USART_CR1_PCE);
		//Even parity is selected by default

	}else if (pUSARTHandle->USARTConfig.USART_ParityCtrl == USART_PARITY_ODD ){
		//Enable parity control
	    tempreg |= ( 1 << USART_CR1_PCE);

	    //Select odd parity
	    tempreg |= ( 1 << USART_CR1_PS);
	}

   //Program the CR1 register
	pUSARTHandle->pUSARTx->CR1 = tempreg;

/******************************** Configuration of CR2******************************************/

	tempreg=0;

	//Number of stop bits inserted during USART frame transmission
	tempreg |= (pUSARTHandle->USARTConfig.USART_StopBitsNum << USART_CR2_STOP);

	//Program the CR2 register
	pUSARTHandle->pUSARTx->CR2 = tempreg;

/******************************** Configuration of CR3******************************************/

	tempreg=0;

	//Configuration of USART hardware flow control
	if ( pUSARTHandle->USARTConfig.USART_HWFlowCtrl == USART_HWFLOW_CTS){
		//Enable CTS flow control
		tempreg |= ( 1 << USART_CR3_CTSE);

	}else if (pUSARTHandle->USARTConfig.USART_HWFlowCtrl == USART_HWFLOW_RTS){
		//Enable RTS flow control
		tempreg |= (1 << USART_CR3_RTSE);

	}else if (pUSARTHandle->USARTConfig.USART_HWFlowCtrl == USART_HWFLOW_BOTH){
		//Enable both CTS and RTS Flow control
		tempreg |= ( 1 << USART_CR3_CTSE);
		tempreg |= (1 << USART_CR3_RTSE);
	}


	pUSARTHandle->pUSARTx->CR3 = tempreg;

/******************************** Configuration of BRR(Baudrate register)******************************************/

	USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USARTConfig.USART_Baud);
}

/********************************************************************************
 *@fn			-	USART_DeInit
 *
 *@brief		-	De-initializes USARTx
 *
 *@param[in]	-	USART peripheral handling structure address
 *@param[in]	-
 *@param[in]	-
 *
 *@return		-	none
 *
 *@Note			-

 */
void USART_DeInit(USART_RegDef_t *pUSARTx){
	if(pUSARTx == USART1)
		USART1_REG_RESET();
	else if(pUSARTx == USART2)
		USART2_REG_RESET();
	else if(pUSARTx == USART3)
		USART3_REG_RESET();
	else if(pUSARTx == UART4)
		UART4_REG_RESET();
	else if(pUSARTx == UART5)
		UART5_REG_RESET();
	else if(pUSARTx == USART6)
		USART6_REG_RESET();
}

/*
 * Data transmit & receive
 */

/********************************************************************************
 *@fn			-	USART_DataTX
 *
 *@brief		-	Transmits data
 *
 *@param[in]	-	USART peripheral handling structure address
 *@param[in]	-	Transmit buffer address
 *@param[in]	-	Transmitted message length
 *
 *@return		-	none
 *
 *@Note			-	Blocking call

 */
void USART_DataTX(USART_Handle_t *pUSARTHandle, uint8_t *pTXBuffer, uint32_t Len){

	uint16_t *pdata;
   //Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < Len; i++)
	{
		//Wait until TXE flag is set in the SR
		while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_TXE));

        //check word length
		if(pUSARTHandle->USARTConfig.USART_WordLen == USART_WORDLEN_9BITS){
			//9BIT
			pdata = (uint16_t*) pTXBuffer;
			pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF); //load 9 bits of data

			//check for parity control
			if(pUSARTHandle->USARTConfig.USART_ParityCtrl == USART_PARITY_DI){
				//No parity
				//Increment pTxBuffer twice
				pTXBuffer++;
				pTXBuffer++;
			}else{
				//= 9th bit replaced by parity bit by the hardware, 8 bits of data transferred
				pTXBuffer++; //increment pTXBuffer once
			}
		}
		else{
			//8BIT
			pUSARTHandle->pUSARTx->DR = (*pTXBuffer  & (uint8_t)0xFF);
			pTXBuffer++; //increment pTXBuffer once
		}
	}

	//Wait till TC flag is set in the SR
	while( ! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_TC));
}

/********************************************************************************
 *@fn			-	USART_DataRX
 *
 *@brief		-	Receives data
 *
 *@param[in]	-	USART peripheral handling structure address
 *@param[in]	-	Receive buffer address
 *@param[in]	-	Received message length
 *
 *@return		-	none
 *
 *@Note			-	Blocking call

 */
void USART_DataRX(USART_Handle_t *pUSARTHandle, uint8_t *pRXBuffer, uint32_t Len){
	//Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < Len; i++)
	{
		//Wait until RXNE flag is set in the SR
		while(! (pUSARTHandle->pUSARTx->SR & (1 << USART_SR_RXNE)) )

		//Check the wordlength
		if(pUSARTHandle->USARTConfig.USART_WordLen == USART_WORDLEN_9BITS){
			//9BIT
			//check for parity control
			if(pUSARTHandle->USARTConfig.USART_ParityCtrl == USART_PARITY_DI){
				//No parity
				//read only first 9 bits
				*((uint16_t*) pRXBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x1FF);

				//Increment the pRxBuffer twice
				pRXBuffer++;
				pRXBuffer++;

			}else{
				//Parity is used, so, 8bits will be of user data and 1 bit is parity
				 *pRXBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);

				 //Increment the pRxBuffer
				pRXBuffer++;
			}
		}else{
			//8BIT
			//check for parity control
			if(pUSARTHandle->USARTConfig.USART_ParityCtrl == USART_PARITY_DI){
				//No parity is used , so all 8bits will be of user data
				 *pRXBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF); //read 8 bits from DR

			}else{
				//Parity is used, so , 7 bits will be of user data and 1 bit is parity
				//read only 7 bits , hence mask the DR with 0X7F
				 *pRXBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR & (uint8_t)0x7F);
			}
			pRXBuffer++; //increment the pRxBuffer
		}
	}
}

/********************************************************************************
 *@fn			-	USART_DataTX_IT
 *
 *@brief		-	Transmits data using interrupts
 *
 *@param[in]	-	USART peripheral handling structure address
 *@param[in]	-	Transmit buffer address
 *@param[in]	-	Transmitted message length
 *
 *@return		-	none
 *
 *@Note			-	Non-blocking call

 */
uint8_t USART_DataTX_IT(USART_Handle_t *pUSARTHandle, uint8_t *pTXBuffer, uint32_t Len){
	uint8_t txstate = pUSARTHandle->TxState;

	if(txstate != USART_BUSY_TX){
		pUSARTHandle->TxLen = Len;
		pUSARTHandle->pTxBuffer = pTXBuffer;
		pUSARTHandle->TxState = USART_BUSY_TX;

		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TXEIE); //Enable interrupt for TXE
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TCIE); //Enable interrupt for TC
	}

	return txstate;
}

/********************************************************************************
 *@fn			-	USART_DataRX_IT
 *
 *@brief		-	Receives data using interrupts
 *
 *@param[in]	-	USART peripheral handling structure address
 *@param[in]	-	Receive buffer address
 *@param[in]	-	Received message length
 *
 *@return		-	none
 *
 *@Note			-	Non-blocking call

 */
uint8_t USART_DataRX_IT(USART_Handle_t *pUSARTHandle, uint8_t *pRXBuffer, uint32_t Len){
	uint8_t rxstate = pUSARTHandle->RxState;

	if(rxstate != USART_BUSY_RX){
		pUSARTHandle->RxLen = Len;
		pUSARTHandle->pRxBuffer = pRXBuffer;
		pUSARTHandle->RxState = USART_BUSY_RX;

		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_RXNEIE); //Enable interrupt for RXNE
	}

	return rxstate;
}


/*
 * IRQ configuration & ISR handling
 */

/********************************************************************************
 *@fn			-	USART_IRQITConfig
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
void USART_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi){
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
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority){
	//1. Identify IPRx
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift = (iprx_section*8) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_IPR_BASEADDR + (iprx)) |= (IRQPriority << shift);
}

/*********************************************************************
 * @fn      	-	USART_IRQHandler
 *
 * @brief       -	Handles interrupts
 *
 * @param[in]   -	USART handle structure address
 * @param[in]   -
 * @param[in]   -
 *
 * @return      -
 *
 * @Note		-

 */
void USART_IRQHandling(USART_Handle_t *pUSARTHandle)
{
	uint32_t temp1, temp2, temp3;
	uint32_t read_to_clear;
	uint16_t *pdata;
/*************************Check for TC flag ********************************************/

    //Check the state of TC bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_TC);

	//Check the state of TCIE bit
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_TCIE);

	if(temp1 && temp2 )
	{
		//this interrupt is because of TC

		//close transmission and call application callback if TxLen is zero
		if ( pUSARTHandle->TxState == USART_BUSY_TX)
		{
			//Check the TxLen . If it is zero then close the data transmission
			if(! pUSARTHandle->TxLen )
			{
				//Implement the code to clear the TC flag
				pUSARTHandle->pUSARTx->SR &= ~( 1 << USART_SR_TC);

				//Implement the code to clear the TCIE control bit
				pUSARTHandle->pUSARTx->CR1 &= ~( 1 << USART_CR1_TCIE);

				//Reset the application state
				pUSARTHandle->TxState = USART_READY;

				//Reset Buffer address to NULL
				pUSARTHandle->pTxBuffer = NULL;

				//Reset the length to zero
				pUSARTHandle->TxLen = 0;

				//Call the applicaton call back with event USART_EVENT_TX_CMPLT
				USART_ApplicationEventCallback(pUSARTHandle,USART_EV_TX_CMPLT);
			}
		}
	}

/*************************Check for TXE flag ********************************************/

	//Implement the code to check the state of TXE bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_TXE);

	//Implement the code to check the state of TXEIE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_TXEIE);

	if(temp1 && temp2 )
	{
		//this interrupt is because of TXE

		if(pUSARTHandle->TxState == USART_BUSY_TX)
		{
			//Keep sending data until Txlen reaches to zero
			if(pUSARTHandle->TxLen > 0)
			{
				//Check the USART_WordLength item for 9BIT or 8BIT in a frame
				if(pUSARTHandle->USARTConfig.USART_WordLen == USART_WORDLEN_9BITS)
				{
					//if 9BIT , load the DR with 2bytes masking the bits other than first 9 bits
					pdata = (uint16_t*) pUSARTHandle->pTxBuffer;

					//loading only first 9 bits , so we have to mask with the value 0x01FF
					pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

					//check for USART_ParityControl
					if(pUSARTHandle->USARTConfig.USART_ParityCtrl == USART_PARITY_DI)
					{
						//No parity is used in this transfer , so, 9bits of user data will be sent
						//Implement the code to increment pTxBuffer twice
						(pUSARTHandle->pTxBuffer)++;
						(pUSARTHandle->pTxBuffer)++;

						//Implement the code to decrement the length
						(pUSARTHandle->TxLen)--;
					}
					else
					{
						//Parity bit is used in this transfer . so , 8bits of user data will be sent
						//The 9th bit will be replaced by parity bit by the hardware
						(pUSARTHandle->pTxBuffer)++;

						//Implement the code to decrement the length
						(pUSARTHandle->TxLen)--;
					}
				}
				else
				{
					//This is 8bit data transfer
					pUSARTHandle->pUSARTx->DR = (*(pUSARTHandle->pTxBuffer)  & (uint8_t)0xFF);

					//Implement the code to increment the buffer address
					(pUSARTHandle->pTxBuffer)++;

					//Implement the code to decrement the length
					(pUSARTHandle->TxLen)--;
				}

			}
			if (pUSARTHandle->TxLen == 0 )
			{
				//TxLen is zero
				//Implement the code to clear the TXEIE bit (disable interrupt for TXE flag )
				pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_TXEIE);
			}
		}
	}

/*************************Check for RXNE flag ********************************************/

	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_RXNE);
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_RXNEIE);


	if(temp1 && temp2 )
	{
		//this interrupt is because of rxne
		//this interrupt is because of txe
		if(pUSARTHandle->RxState == USART_BUSY_RX)
		{
			//TXE is set so send data
			if(pUSARTHandle->RxLen > 0)
			{
				//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
				if(pUSARTHandle->USARTConfig.USART_WordLen == USART_WORDLEN_9BITS)
				{
					//We are going to receive 9bit data in a frame

					//Now, check are we using USART_ParityControl control or not
					if(pUSARTHandle->USARTConfig.USART_ParityCtrl == USART_PARITY_DI)
					{
						//No parity is used. so, all 9bits will be of user data

						//read only first 9 bits so mask the DR with 0x01FF
						*((uint16_t*) pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x01FF);

						//Now increment the pRxBuffer two times
						(pUSARTHandle->pRxBuffer)++;
						(pUSARTHandle->pRxBuffer)++;

						//Implement the code to decrement the length
						(pUSARTHandle->RxLen)--;
					}
					else
					{
						//Parity is used. so, 8bits will be of user data and 1 bit is parity
						 *(pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);

						 //Now increment the pRxBuffer
						 (pUSARTHandle->pRxBuffer)++;

						 //Implement the code to decrement the length
						 (pUSARTHandle->RxLen)--;
					}
				}
				else
				{
					//We are going to receive 8bit data in a frame

					//Now, check are we using USART_ParityControl control or not
					if(pUSARTHandle->USARTConfig.USART_ParityCtrl == USART_PARITY_DI)
					{
						//No parity is used , so all 8bits will be of user data

						//read 8 bits from DR
						 *(pUSARTHandle->pRxBuffer) = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
					}

					else
					{
						//Parity is used, so , 7 bits will be of user data and 1 bit is parity

						//read only 7 bits , hence mask the DR with 0X7F
						 *(pUSARTHandle->pRxBuffer) = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0x7F);

					}

					//Now , increment the pRxBuffer
					(pUSARTHandle->pRxBuffer)++;

					//Implement the code to decrement the length
					(pUSARTHandle->RxLen)--;
				}


			}//if of >0

			if(! pUSARTHandle->RxLen)
			{
				//disable the rxne
				pUSARTHandle->pUSARTx->CR1 &= ~( 1 << USART_CR1_RXNEIE );
				pUSARTHandle->RxState = USART_READY;
				USART_ApplicationEventCallback(pUSARTHandle,USART_EV_RX_CMPLT);
			}
		}
	}


/*************************Check for CTS flag ********************************************/
//Note : CTS feature is not applicable for UART4 and UART5

	//Implement the code to check the status of CTS bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_CTS);

	//Implement the code to check the state of CTSE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_CTSE);

	//Implement the code to check the state of CTSIE bit in CR3 (This bit is not available for UART4 & UART5.)
	temp3 = pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_CTSIE);


	if(temp1  && temp2 )
	{
		//Implement the code to clear the CTS flag in SR
		pUSARTHandle->pUSARTx->SR &= ~(1 << USART_SR_CTS);

		//this interrupt is because of cts
		USART_ApplicationEventCallback(pUSARTHandle,USART_EV_CTS);
	}

/*************************Check for IDLE detection flag ********************************************/

	//Implement the code to check the status of IDLE flag bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_IDLE);

	//Implement the code to check the state of IDLEIE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_IDLEIE);


	if(temp1 && temp2)
	{
		//Implement the code to clear the IDLE flag. Read SR (already done) then read DR.
		read_to_clear = pUSARTHandle->pUSARTx->DR;
		(void)read_to_clear; //to avoid warnings
		//this interrupt is because of idle
		USART_ApplicationEventCallback(pUSARTHandle,USART_EV_IDLE);
	}

/*************************Check for Overrun detection flag ********************************************/

	//Implement the code to check the status of ORE flag  in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_ORE);

	//Implement the code to check the status of RXNEIE  bit in the CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_RXNEIE);


	if(temp1  && temp2 )
	{
		//this interrupt is because of Overrun error
		USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_ORE);
	}



/*************************Check for Error Flag ********************************************/

//Noise Flag, Overrun error and Framing Error in multibuffer communication
//We dont discuss multibuffer communication in this course. please refer to the RM
//The blow code will get executed in only if multibuffer mode is used.

	temp2 =  pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_EIE) ;

	if(temp2 )
	{
		temp1 = pUSARTHandle->pUSARTx->SR;
		if(temp1 & ( 1 << USART_SR_FE))
		{
			/*
				This bit is set by hardware when a de-synchronization, excessive noise or a break character
				is detected. It is cleared by a software sequence (an read to the USART_SR register
				followed by a read to the USART_DR register).
			*/
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_FE);
		}

		if(temp1 & ( 1 << USART_SR_NF) )
		{
			/*
				This bit is set by hardware when noise is detected on a received frame. It is cleared by a
				software sequence (an read to the USART_SR register followed by a read to the
				USART_DR register).
			*/
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_NF);
		}

		if(temp1 & ( 1 << USART_SR_ORE) )
		{
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_ORE);
		}
	}
	(void)temp3;
}

/*
 * Other peripheral control APIs
 */

/********************************************************************************
 *@fn			-	USART_GetFlagStatus
 *
 *@brief		-	Gets flag status of given flag in USARTx SR register
 *
 *@param[in]	-	USART peripheral base address
 *@param[in]	-	Macro from @USART_FLAGNAME
 *@param[in]	-
 *
 *@return		-	Flag status (1 or 0)
 *
 *@Note			-

 */
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint32_t Flagname){

	if(pUSARTx->SR & Flagname)
		return FLAG_SET;

	return FLAG_RESET;
}

/********************************************************************************
 *@fn			-	USART_PeripheralControl
 *
 *@brief		-	Enables or disables the given USARTx (x=1/2/3) peripheral
 *
 *@param[in]	-	USART peripheral base address
 *@param[in]	-	ENABLE or DISABLE macros
 *@param[in]	-
 *
 *@return		-	none
 *
 *@Note			-

 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		pUSARTx->CR1 |= (1 << USART_CR1_UE);
	}
	else if(EnorDi == DISABLE){
		pUSARTx->CR1 &= ~(1 << USART_CR1_UE);
	}
}

/*********************************************************************
 * @fn      	-	USART_SetBaudRate
 *
 * @brief       -	Sets desired USART baud rate
 *
 * @param[in]   -	USARTx peripheral base address
 * @param[in]   -	Desired baud rate
 * @param[in]   -
 *
 * @return      -	none
 *
 * @Note        -

 */
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate)
{

	//Variable to hold the APB clock
	uint32_t PCLKx;

	uint32_t usartdiv;

	//variables to hold Mantissa and Fraction values
	uint32_t M_part,F_part;

  uint32_t tempreg=0;

  //Get the value of APB bus clock in to the variable PCLKx
  if(pUSARTx == USART1 || pUSARTx == USART6)
  {
	  //USART1 and USART6 are hanging on APB2 bus
	  PCLKx = RCC_GetPCLK2Value();
  }else
  {
	  PCLKx = RCC_GetPCLK1Value();
  }

  //Check for OVER8 configuration bit
  if(pUSARTx->CR1 & (1 << USART_CR1_OVER8))
  {
	  //OVER8 = 1 , over sampling by 8
	  usartdiv = ((25 * PCLKx) / (2 *BaudRate));
  }else
  {
	  //over sampling by 16
	  usartdiv = ((25 * PCLKx) / (4 *BaudRate));
  }

  //Calculate the Mantissa part
  M_part = usartdiv/100;

  //Place the Mantissa part in appropriate bit position . refer USART_BRR
  tempreg |= M_part << USART_BRR_DIV_MANTISSA;

  //Extract the fraction part
  F_part = (usartdiv - (M_part * 100));

  //Calculate the final fractional
  if(pUSARTx->CR1 & ( 1 << USART_CR1_OVER8))
   {
	  //OVER8 = 1 , over sampling by 8
	  F_part = ((( F_part * 8)+ 50) / 100)& ((uint8_t)0x07);

   }else
   {
	   //over sampling by 16
	   F_part = ((( F_part * 16)+ 50) / 100) & ((uint8_t)0x0F);

   }

  //Place the fractional part in appropriate bit position . refer USART_BRR
  tempreg |= F_part << USART_BRR_DIV_FRACTION;

  //copy the value of tempreg in to BRR register
  pUSARTx->BRR = tempreg;
}

/*********************************************************************
 * @fn      	-	USART_ClearOREFlag
 *
 * @brief       -	Clears ORE flag
 *
 * @param[in]   -	USARTx peripheral base address
 * @param[in]   -
 * @param[in]   -
 *
 * @return      -	none
 *
 * @Note        -

 */
void USART_ClearOREFlag(USART_RegDef_t *pUSARTx){
	uint8_t temp;
	temp = pUSARTx->SR;
	temp = pUSARTx->DR;

	//just to avoid warnings
	(void)temp;
}
/*
 * Application callback
 */
__weak void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t ApEv){
	//Weak implementation
}
