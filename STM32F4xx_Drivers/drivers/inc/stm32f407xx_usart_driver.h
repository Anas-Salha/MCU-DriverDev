/*
 * stm32f407xx_usart_driver.h
 *
 *  Created on: Nov 6, 2019
 *      Author: AnaS9
 */

#ifndef INC_STM32F407XX_USART_DRIVER_H_
#define INC_STM32F407XX_USART_DRIVER_H_


#include "stm32f407xx.h"
#include <stdint.h>

/***************************************************** CONFIGURATION STRUCTURE *********************************************************/

typedef struct{
	uint8_t USART_Mode;						/*!<possible values from @USART_MODE>*/
	uint32_t USART_Baud;						/*!<possible values from @USART_BAUD>*/
	uint8_t USART_StopBitsNum;				/*!<possible values from @USART_STOPBITS>*/
	uint8_t USART_WordLen;					/*!<possible values from @USART_WORDLEN>*/
	uint8_t USART_ParityCtrl;				/*!<possible values from @USART_PARITY_CTRL>*/
	uint8_t USART_HWFlowCtrl;				/*!<possible values from @USART_HWFLOW_CTRL>*/
}USART_Config_t;

/******************************************************** HANDLE STRUCTURE *************************************************************/

typedef struct{
	USART_RegDef_t 	*pUSARTx;				/*<Pointer that holds the base address of USARTx(x:1,2,3,4,5,6) peripheral>*/
	USART_Config_t 	USARTConfig;			/*<Structure that holds the USART configuration settings>*/
	uint8_t			*pTxBuffer;				/*<Store app. Tx buffer address>*/
	uint8_t			*pRxBuffer;				/*<Store app. Rx buffer address>*/
	uint32_t		TxLen;					/*<Store app. Tx length>*/
	uint32_t		RxLen;					/*<Store app. Rx length>*/
	uint8_t			TxState;				/*<Store app. Tx state>*/
	uint8_t			RxState;				/*<Store app. Rx state>*/
}USART_Handle_t;

/*********************************************************** USER MACROS ****************************************************************/

/*
 *@USART_MODE
 *Possible options for USART_Mode
 */
#define USART_MODE_TX 					0
#define USART_MODE_RX 					1
#define USART_MODE_TXRX  				2

/*
 *@USART_BAUD
 *Possible options for USART_Baud
 */
#define USART_BAUD_1200					1200
#define USART_BAUD_2400					2400
#define USART_BAUD_9600					9600
#define USART_BAUD_19200 				19200
#define USART_BAUD_38400 				38400
#define USART_BAUD_57600 				57600
#define USART_BAUD_115200 				115200
#define USART_BAUD_230400 				230400
#define USART_BAUD_460800 				460800
#define USART_BAUD_921600 				921600
#define USART_BAUD_2M 					2000000
#define SUART_BAUD_3M 					3000000

/*
 *@USART_STOPBITS
 *Possible options for USART_StopBitsNum
 */
#define USART_STOPBITS_1     			0
#define USART_STOPBITS_0_5   			1
#define USART_STOPBITS_2     			2
#define USART_STOPBITS_1_5   			3


/*
 *@USART_WORDLEN
 *Possible options for USART_WordLen
 */
#define USART_WORDLEN_8BITS  			0
#define USART_WORDLEN_9BITS 			1

/*
 *@USART_PARITY_CTRL
 *Possible options for USART_ParityCtrl
 */
#define USART_PARITY_DI   				0
#define USART_PARITY_EVEN  				1
#define USART_PARITY_ODD   				2

/*
 *@USART_HWFLOW_CTRL
 *Possible options for USART_HWFlowCtrl
 */
#define USART_HWFLOW_NONE    			0
#define USART_HWFLOW_CTS    			1
#define USART_HWFLOW_RTS    			2
#define USART_HWFLOW_BOTH				3

/*
 * @USART_FLAGNAME
 * USART SR flag definitions
 */
#define	USART_FLAG_PE					(1 << USART_SR_PE)
#define	USART_FLAG_FE					(1 << USART_SR_FE)
#define	USART_FLAG_NF					(1 << USART_SR_NF)
#define	USART_FLAG_ORE					(1 << USART_SR_ORE)
#define	USART_FLAG_IDLE					(1 << USART_SR_IDLE)
#define	USART_FLAG_RXNE					(1 << USART_SR_RXNE)
#define	USART_FLAG_TC					(1 << USART_SR_TC)
#define	USART_FLAG_TXE					(1 << USART_SR_TXE)
#define	USART_FLAG_LBD					(1 << USART_SR_LBD)
#define	USART_FLAG_CTS					(1 << USART_SR_CTS)

/*
 * @USART_APP_STATES
 * USART application states
 */
#define USART_READY						0
#define USART_BUSY_RX					1
#define USART_BUSY_TX					2

/*
 * @USART_APP_CALLBACK
 * Possible USART application callbacks
 */
#define USART_EV_TX_CMPLT				1
#define USART_EV_RX_CMPLT				2
#define USART_EV_CTS					3
#define USART_EV_IDLE					4
#define USART_ERR_ORE					5
#define USART_ERR_FE					6
#define USART_ERR_NF					7
/************************************************* APIs supported by this driver *******************************************************/

/*
 * Peripheral clock setup
 */
void USART_PCLK_Control(USART_RegDef_t *pUSARTx, uint8_t EnorDi);


/*
 * Initialization & De-initialization
 */
void USART_Init(USART_Handle_t *pUSARTHandle);
void USART_DeInit(USART_RegDef_t *pUSARTx);


/*
 * Data transmit & receive
 */
void USART_DataTX(USART_Handle_t *pUSARTHandle, uint8_t *pTXBuffer, uint32_t Len);
void USART_DataRX(USART_Handle_t *pUSARTHandle, uint8_t *pRXBuffer, uint32_t Len);
uint8_t USART_DataTX_IT(USART_Handle_t *pUSARTHandle, uint8_t *pTXBuffer, uint32_t Len);
uint8_t USART_DataRX_IT(USART_Handle_t *pUSARTHandle, uint8_t *pRXBuffer, uint32_t Len);

/*
 * IRQ configuration & ISR handling
 */
void USART_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void USART_IRQHandling(USART_Handle_t *pUSARTHandle);

/*
 * Other peripheral control APIs
 */
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint32_t Flagname);
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi);
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate);
void USART_ClearOREFlag(USART_RegDef_t *pUSARTx);

/*
 * Application callback
 */
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t ApEv);

#endif /* INC_STM32F407XX_USART_DRIVER_H_ */

