/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Oct 15, 2019
 *      Author: AnaS9
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"
#include <stdint.h>

/******************************************************CONFIGURATION STRUCTURE**********************************************************/

typedef struct{
	uint8_t	GPIO_PinNumber;					/*!<possible values from @GPIO_PIN_NUMBERS>*/
	uint8_t GPIO_PinMode;					/*!<possible values from @GPIO_PIN_MODES>*/
	uint8_t GPIO_PinSpeed;					/*!<possible values from @GPIO_PIN_SPEED>*/
	uint8_t GPIO_PinPuPdControl;			/*!<possible values from @GPIO_PIN_PUPD>*/
	uint8_t GPIO_PinOPType;					/*!<possible values from @GPIO_PIN_OPTYPE>*/
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

/*********************************************************HANDLE STRUCTURE**************************************************************/

typedef struct{
	GPIO_RegDef_t *pGPIOx;						/*<Pointer that holds the base address of the GPIO port to which the pin belongs>*/
	GPIO_PinConfig_t GPIO_PinConfig;			/*<Structure that holds the GPIO pin configuration settings>*/

}GPIO_Handle_t;


/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */
#define GPIO_PIN_0						0
#define GPIO_PIN_1						1
#define GPIO_PIN_2						2
#define GPIO_PIN_3						3
#define GPIO_PIN_4						4
#define GPIO_PIN_5						5
#define GPIO_PIN_6						6
#define GPIO_PIN_7						7
#define GPIO_PIN_8						8
#define GPIO_PIN_9						9
#define GPIO_PIN_10						10
#define GPIO_PIN_11						11
#define GPIO_PIN_12						12
#define GPIO_PIN_13						13
#define GPIO_PIN_14						14
#define GPIO_PIN_15						15

/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 */
#define GPIO_MODE_IN					0
#define GPIO_MODE_OUT					1
#define GPIO_MODE_ALTFN					2
#define GPIO_MODE_ANALOG				3
#define GPIO_MODE_IT_FT					4
#define GPIO_MODE_IT_RT					5
#define GPIO_MODE_IT_RFT				6

/*
 * @GPIO_PIN_OPTYPE
 * GPIO pin possible output types
 */
#define GPIO_OP_TYPE_PP					0
#define GPIO_OP_TYPE_OD					1

/*
 * @GPIO_PIN_SPEED
 * GPIO pin possible output speeds
 */
#define GPIO_SPEED_LOW					0
#define GPIO_SPEED_MEDIUM				1
#define GPIO_SPEED_HIGH					2
#define GPIO_SPEED_VHIGH				3

/*
 * @GPIO_PIN_PUPD
 * GPIO PuPd possible configurations
 */
#define GPIO_NO_PUPD					0
#define GPIO_PU							1
#define GPIO_PD							2

/*
 * @ON_BOARD_LEDs
 * On board LED pin number macros
 */
#define LD3_PIN							GPIO_PIN_13
#define LD4_PIN							GPIO_PIN_12
#define LD5_PIN							GPIO_PIN_14
#define LD6_PIN							GPIO_PIN_15

/*
 * @ON_USER_BUTTONS
 * On board user button pin number macro
 */
#define USER_BUTTON						GPIO_PIN_0


/**************************************************APIs supported by this driver********************************************************/

/*Peripheral clock setup
 *
 */
void GPIO_PCLK_Control(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/* Initialization & De-initialization
 *
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/* Data read & write
 *
 */
uint8_t GPIO_InPinRead(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_InPortRead(GPIO_RegDef_t *pGPIOx);
void GPIO_OPinWrite(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_OPortWrite(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_OPinToggle(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*IRQ configuration & handling
 *
 */
void GPIO_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);








#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
