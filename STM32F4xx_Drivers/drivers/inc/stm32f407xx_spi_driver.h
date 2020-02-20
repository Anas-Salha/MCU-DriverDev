/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: Oct 20, 2019
 *      Author: AnaS9
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h"
#include <stdint.h>

/******************************************************CONFIGURATION STRUCTURE**********************************************************/

typedef struct{
	uint8_t	SPI_DeviceMode;					/*!<possible values from @SPI_DEVICE_MODES>*/
	uint8_t SPI_BusConfig;					/*!<possible values from @SPI_BUS_CONFIG>*/
	uint8_t SPI_DFF;						/*!<possible values from @SPI_DFF>*/
	uint8_t SPI_CPHA;						/*!<possible values from @SPI_CLOCK_PHASE>*/
	uint8_t SPI_CPOL;						/*!<possible values from @SPI_CLOCK_POLARITY>*/
	uint8_t SPI_SSM;						/*!<possible values from @SPI_SSM>*/
	uint8_t SPI_SCLKSpeed;					/*!<possible values from @SPI_SCLK_SPEED>*/
}SPI_Config_t;

/*********************************************************HANDLE STRUCTURE**************************************************************/

typedef struct{
	SPI_RegDef_t 	*pSPIx;				/*<Pointer that holds the base address of SPIx(x:0,1,2) peripheral>*/
	SPI_Config_t 	SPIConfig;			/*<Structure that holds the SPI configuration settings>*/
	uint8_t			*pTxBuffer;			/*<Store app. Tx buffer address>*/
	uint8_t			*pRxBuffer;			/*<Store app. Rx buffer address>*/
	uint32_t		TxLen;				/*<Store app. Tx length>*/
	uint32_t		RxLen;				/*<Store app. Rx length>*/
	uint8_t			TxState;			/*<Store app. Tx state>*/
	uint8_t			RxState;			/*<Store app. Rx state>*/
}SPI_Handle_t;


/*
 * @SPI_DEVICE_MODES
 * SPI device modes
 */
#define SPI_MODE_SLAVE						0
#define SPI_MODE_MASTER						1

/*
 * @SPI_BUS_CONFIG
 * SPI bus configuration
 */
#define SPI_BUS_HDUPLEX					1
#define SPI_BUS_FDUPLEX					2
#define SPI_BUS_SIMPLEX_RX				3

/*
 * @SPI_DFF
 * SPI data frame formats
 */
#define SPI_DFF_8BIT					0
#define SPI_DFF_16BIT					1

/*
 * @SPI_CLOCK_PHASE
 * SPI SCLK phase
 */
#define SPI_CPHA_EDGE1					0
#define SPI_CPHA_EDGE2					1

/*
 * @SPI_CLOCK_POLARITY
 * SPI SCLK polarity
 */
#define SPI_CPOL_LOW					0
#define SPI_CPOL_HIGH					1

/*
 * @SPI_SSM
 * SPI slave select management
 */
#define SPI_SSM_DI						0
#define SPI_SSM_EN						1

/*
 * @SPI_SCLK_SPEED
 * SPI SCLK speed
 */
#define SPI_SCLK_SPEED_DIV2				0
#define SPI_SCLK_SPEED_DIV4				1
#define SPI_SCLK_SPEED_DIV8				2
#define SPI_SCLK_SPEED_DIV16			3
#define SPI_SCLK_SPEED_DIV32			4
#define SPI_SCLK_SPEED_DIV64			5
#define SPI_SCLK_SPEED_DIV128			6
#define SPI_SCLK_SPEED_DIV256			7


/*
 * @SPI_FLAGNAME
 * SPI status flag definitions
 */
#define SPI_FLAG_RXNE					(1 << SPI_SR_RXNE)
#define SPI_FLAG_TXE					(1 << SPI_SR_TXE)
#define SPI_FLAG_CHSIDE					(1 << SPI_SR_CHSIDE)
#define SPI_FLAG_UDR					(1 << SPI_SR_UDR)
#define SPI_FLAG_CRCERR					(1 << SPI_SR_CRCERR)
#define SPI_FLAG_MODF					(1 << SPI_SR_MODF)
#define SPI_FLAG_OVR					(1 << SPI_SR_OVR)
#define SPI_FLAG_BSY					(1 << SPI_SR_BSY)
#define SPI_FLAG_FRE					(1 << SPI_SR_FRE)

/*
 * @SPI_APP_STATES
 * Possible SPI application states
 */
#define SPI_READY						0
#define SPI_BUSY_TX						1
#define SPI_BUSY_RX						2

/*
 * @SPI_APP_CALLBACK
 * Possible SPI application callbacks
 */
#define SPI_EVENT_TX_CMPLT				1
#define SPI_EVENT_RX_CMPLT				2
#define SPI_EVENT_OVR_ERR				3

/**************************************************APIs supported by this driver********************************************************/
/*
 * Peripheral clock setup
 */
void SPI_PCLK_Control(SPI_RegDef_t *pSPIx, uint8_t EnorDi);


/*
 * Initialization & De-initialization
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);


/*
 * Data transmit & receive
 */
void SPI_DataTX(SPI_RegDef_t *pSPIx, uint8_t *pTXBuffer, uint32_t Len);
void SPI_DataRX(SPI_RegDef_t *pSPIx, uint8_t *pRXBuffer, uint32_t Len);

uint8_t SPI_ITDataTX(SPI_Handle_t *pSPIHandle, uint8_t *pTXBuffer, uint32_t Len);
uint8_t SPI_ITDataRX(SPI_Handle_t *pSPIHandle, uint8_t *pRXBuffer, uint32_t Len);

/*
 * IRQ configuration & ISR handling
 */
void SPI_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);

/*
 * Other peripheral control APIs
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t Flagname);
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_CloseTX(SPI_Handle_t *pSPIHandle);
void SPI_CloseRX(SPI_Handle_t *pSPIHandle);
void SPI_ClearOVR(SPI_Handle_t *pSPIHandle);

/*
 * Application callback
 */
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t ApEv);

#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
