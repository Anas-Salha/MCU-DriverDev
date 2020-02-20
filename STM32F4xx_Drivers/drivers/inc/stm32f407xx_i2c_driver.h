/*
 * stm32f4xx_i2c_driver.h
 *
 *  Created on: Oct 27, 2019
 *      Author: AnaS9
 */

#ifndef INC_STM32F407XX_I2C_DRIVER_H_
#define INC_STM32F407XX_I2C_DRIVER_H_

#include "stm32f407xx.h"
#include <stdint.h>

/***************************************************** CONFIGURATION STRUCTURE *********************************************************/

typedef struct{
	uint32_t I2C_SCLKSpeed;					/*!<possible values from @I2C_SCLK_SPEED>*/
	uint8_t I2C_DeviceAddr;
	uint8_t I2C_ACKCtrl;					/*!<possible values from @I2C_ACK_CTRL>*/
	uint8_t I2C_FMDutyCycle;				/*!<possible values from @I2C_FM_DUTY>*/
}I2C_Config_t;

/******************************************************** HANDLE STRUCTURE *************************************************************/

typedef struct{
	I2C_RegDef_t 	*pI2Cx;				/*<Pointer that holds the base address of I2Cx(x:1,2,3) peripheral>*/
	I2C_Config_t 	I2CConfig;			/*<Structure that holds the I2C configuration settings>*/
	uint8_t			*pTxBuffer;			/*<Store app. Tx buffer address>*/
	uint8_t			*pRxBuffer;			/*<Store app. Rx buffer address>*/
	uint32_t		TxLen;				/*<Store app. Tx length>*/
	uint32_t		RxLen;				/*<Store app. Rx length>*/
	uint8_t			TxRxState;			/*<Store app. Tx/Rx state, possible values @I2C_APP_STATES>*/
	uint8_t			DevAddr;			/*<Store slave address>*/
	uint32_t		RxSize;				/*<Store app. Rx size>*/
	uint8_t			Sr;					/*<Store app. repeated start value>*/
}I2C_Handle_t;

/*********************************************************** USER MACROS ****************************************************************/

/*
 * @I2C_SCLK_SPEED
 * I2C clock possible speeds
 */
#define I2C_SPEED_SM				100000
#define I2C_SPEED_FM2K				200000
#define I2C_SPEED_FM4K				400000

/*
 * @I2C_ACK_CTRL
 * I2C acknowledgment enable & disable
 */
#define I2C_ACK_DI					0
#define I2C_ACK_EN					1

/*
 * @I2C_FM_DUTY
 * I2C fast mode duty cycle
 */
#define I2C_FM_DUTY2				0
#define I2C_FM_DUTY16_9				1

/*
 * @I2C_APP_STATES
 * I2C applications states
 */
#define I2C_READY					0
#define I2C_BUSY_RX					1
#define I2C_BUSY_TX					2

/*
 * @I2C_FLAGNAME
 * I2C SR1 flag definitions
 */
#define	I2C_FLAG_SB					(1 << I2C_SR1_SB)
#define	I2C_FLAG_ADDR				(1 << I2C_SR1_ADDR)
#define	I2C_FLAG_BTF				(1 << I2C_SR1_BTF)
#define	I2C_FLAG_ADD10				(1 << I2C_SR1_ADD10)
#define	I2C_FLAG_STOPF				(1 << I2C_SR1_STOPF)
#define	I2C_FLAG_RXNE				(1 << I2C_SR1_RXNE)
#define	I2C_FLAG_TXE				(1 << I2C_SR1_TXE)
#define	I2C_FLAG_BERR				(1 << I2C_SR1_BERR)
#define	I2C_FLAG_ARLO				(1 << I2C_SR1_ARLO)
#define	I2C_FLAG_AF					(1 << I2C_SR1_AF)
#define	I2C_FLAG_OVR				(1 << I2C_SR1_OVR)
#define	I2C_FLAG_PECERR				(1 << I2C_SR1_PECERR)
#define	I2C_FLAG_TIMEOUT			(1 << I2C_SR1_TIMOUT)
#define	I2C_FLAG_SMBALERT			(1 << I2C_SR1_SMBALERT)
#define	I2C_FLAG_MSL				(1 << I2C_SR2_MSL)
#define	I2C_FLAG_BUSY				(1 << I2C_SR2_BUSY)
#define	I2C_FLAG_TRA				(1 << I2C_SR2_TRA)
#define	I2C_FLAG_GENCALL			(1 << I2C_SR2_GENCALL)
#define	I2C_FLAG_SMBDEFAULT			(1 << I2C_SR2_SMBDEFAULT)
#define	I2C_FLAG_SMBHOST			(1 << I2C_SR2_SMBHOST)
#define	I2C_FLAG_DUALF				(1 << I2C_SR2_DUALF)
#define	I2C_FLAG_PEC				(1 << I2C_SR2_PEC)

/*
 * @I2C_SR
 * Status register macros
 */
#define I2C_SR1						1
#define I2C_SR2						2

/*
 * @I2C_APP_CALLBACK
 * Possible I2C application callbacks
 */
#define I2C_EV_TX_CMPLT				1
#define I2C_EV_RX_CMPLT				2
#define I2C_EV_STOP					3
#define I2C_ER_BERR					4
#define I2C_ER_ARLO					5
#define I2C_ER_AF					6
#define I2C_ER_OVR					7
#define I2C_ER_TIMEOUT				8
#define I2C_EV_DATA_REQ				9
#define I2C_EV_DATA_RCV				10

/*
 * Generic Macros
 */
#define WRITE						1
#define READ						2
#define I2C_SR_EN					SET
#define I2C_SR_DI					RESET


/************************************************* APIs supported by this driver *******************************************************/

/*
 * Peripheral clock setup
 */
void I2C_PCLK_Control(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);


/*
 * Initialization & De-initialization
 */
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);


/*
 * Data transmit & receive
 */
void I2C_MasterTX(I2C_Handle_t *pI2CHandle, uint8_t *pTXBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);
void I2C_MasterRX(I2C_Handle_t *pI2CHandle, uint8_t *pRXBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);
uint8_t I2C_MasterTX_IT(I2C_Handle_t *pI2CHandle, uint8_t *pTXBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);
uint8_t I2C_MasterRX_IT(I2C_Handle_t *pI2CHandle, uint8_t *pRXBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);
void I2C_SlaveTX(I2C_RegDef_t *pI2Cx, uint8_t data);
uint8_t I2C_SlaveRX(I2C_RegDef_t *pI2Cx);

/*
 * IRQ configuration & ISR handling
 */
void I2C_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);

/*
 * Other peripheral control APIs
 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t Flagname, uint8_t Reg);
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
void I2C_ManageACK(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
void I2C_CloseTX(I2C_Handle_t *pI2CHandle);
void I2C_CloseRX(I2C_Handle_t *pI2CHandle);
void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
void I2C_InterruptControlBits(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/*
 * Application callback
 */
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t ApEv);

#endif /* INC_STM32F407XX_I2C_DRIVER_H_ */
