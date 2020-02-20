/*
 * stm32f407xx.h
 *
 *  Created on: Oct 14, 2019
 *      Author: AnaS9
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>
#include <stddef.h>

#define __vo		volatile
#define __weak		__attribute__((weak))
/**************************************** 0. Processor Specific Details **************************************/
/*
 * Arm Cortex M4 Processor NVIC ISERx register addresses
 */
#define NVIC_ISER0						( (__vo uint32_t *)0xE000E100 )
#define NVIC_ISER1						( (__vo uint32_t *)0xE000E104 )
#define NVIC_ISER2						( (__vo uint32_t *)0xE000E108 )
#define NVIC_ISER3						( (__vo uint32_t *)0xE000E10C )
#define NVIC_ISER4						( (__vo uint32_t *)0xE000E110 )
#define NVIC_ISER5						( (__vo uint32_t *)0xE000E114 )
#define NVIC_ISER6						( (__vo uint32_t *)0xE000E118 )
#define NVIC_ISER7						( (__vo uint32_t *)0xE000E11C )

/*
 * Arm Cortex M4 Processor NVIC ICERx register addresses
 */
#define NVIC_ICER0						( (__vo uint32_t *)0XE000E180 )
#define NVIC_ICER1						( (__vo uint32_t *)0XE000E184 )
#define NVIC_ICER2						( (__vo uint32_t *)0xE000E188 )
#define NVIC_ICER3						( (__vo uint32_t *)0xE000E18C )
#define NVIC_ICER4						( (__vo uint32_t *)0xE000E190 )
#define NVIC_ICER5						( (__vo uint32_t *)0xE000E194 )
#define NVIC_ICER6						( (__vo uint32_t *)0xE000E198 )
#define NVIC_ICER7						( (__vo uint32_t *)0xE000E19C )

/*
 * Arm Cortex M4 Processor NVIC IPRx registers base address
 */
#define NVIC_IPR_BASEADDR				( (__vo uint32_t *)0xE000E400 )

/*
 * Arm Cortex M4 Processor number of IPRx bits implemented by MCU
 */
#define NO_PR_BITS_IMPLEMENTED			4


/************************************** 1. Base Addresses: Flash & SRAM **************************************/
#define FLASH_BASEADDR					0x08000000U					//base address of the flash memory
#define SRAM1_BASEADDR					0x20000000U 				//base address of SRAM1 (112KB)
#define SRAM2_BASEADDR					0x2001C000U					//base address of SRAM2 (16KB)
#define ROM_BASEADDR					0x1FFF0000U					//base address of the system memory
#define SRAM 							SRAM1_BASEADDR				//SRAM1 is the main SRAM

/************************************** 2. Base Addresses: Bus Domains ***************************************/
#define PERIPH_BASEADDR					0x40000000U					//base address of peripherals
#define APB1PERIPH_BASEADDR				PERIPH_BASEADDR					//base address of APB1 peripherals
#define APB2PERIPH_BASEADDR				(PERIPH_BASEADDR + 0x10000)		//base address of APB2 peripherals
#define AHB1PERIPH_BASEADDR				(PERIPH_BASEADDR + 0x20000)		//base address of AHB1 peripherals
#define AHB2PERIPH_BASEADDR				0x50000000U					//base address of AHB2 peripherals
#define AHB3PERIPH_BASEADDR				0xA0000000U					//base address of AHB3 peripherals

/************************************** 3. Base Addresses: Peripherals ***************************************/
/*APB1 Peripherals*/
#define SPI2_BASEADDR					(APB1PERIPH_BASEADDR + 0x3800)		//base address of SPI2 registers
#define SPI3_BASEADDR					(APB1PERIPH_BASEADDR + 0x3C00)		//base address of SPI3 registers
#define USART2_BASEADDR					(APB1PERIPH_BASEADDR + 0x4400)		//base address of USART2 registers
#define USART3_BASEADDR					(APB1PERIPH_BASEADDR + 0x4800)		//base address of USART3 registers
#define UART4_BASEADDR					(APB1PERIPH_BASEADDR + 0x4C00)		//base address of UART4 registers
#define UART5_BASEADDR					(APB1PERIPH_BASEADDR + 0x5000)		//base address of UART5 registers
#define I2C1_BASEADDR					(APB1PERIPH_BASEADDR + 0x5400)		//base address of I2C1 registers
#define I2C2_BASEADDR					(APB1PERIPH_BASEADDR + 0x5800)		//base address of I2C2 registers
#define I2C3_BASEADDR					(APB1PERIPH_BASEADDR + 0x5C00)		//base address of I2C3 registers


/*APB2 Peripherals*/
#define USART1_BASEADDR					(APB2PERIPH_BASEADDR + 0x1000)		//base address of USART1 registers
#define USART6_BASEADDR					(APB2PERIPH_BASEADDR + 0x1400)		//base address of USART6 registers
#define SPI1_BASEADDR					(APB2PERIPH_BASEADDR + 0x3000)		//base address of SPI1 registers
#define SYSCFG_BASEADDR					(APB2PERIPH_BASEADDR + 0x3800)		//base address of SYSCFG registers
#define EXTI_BASEADDR					(APB2PERIPH_BASEADDR + 0x3C00)		//base address of EXTI registers


/*AHB1 Peripherals*/
#define GPIOA_BASEADDR					AHB1PERIPH_BASEADDR					//base address of GPIOA registers
#define GPIOB_BASEADDR					(AHB1PERIPH_BASEADDR + 0x0400)		//base address of GPIOB registers
#define GPIOC_BASEADDR					(AHB1PERIPH_BASEADDR + 0x0800)		//base address of GPIOC registers
#define GPIOD_BASEADDR					(AHB1PERIPH_BASEADDR + 0x0C00)		//base address of GPIOD registers
#define GPIOE_BASEADDR					(AHB1PERIPH_BASEADDR + 0x1000)		//base address of GPIOE registers
#define GPIOF_BASEADDR					(AHB1PERIPH_BASEADDR + 0x1400)		//base address of GPIOF registers
#define GPIOG_BASEADDR					(AHB1PERIPH_BASEADDR + 0x1800)		//base address of GPIOG registers
#define GPIOH_BASEADDR					(AHB1PERIPH_BASEADDR + 0x1C00)		//base address of GPIOH registers
#define GPIOI_BASEADDR					(AHB1PERIPH_BASEADDR + 0x2000)		//base address of GPIOI registers
#define CRC_BASEADDR					(AHB1PERIPH_BASEADDR + 0x3000)		//base address of CRC registers
#define RCC_BASEADDR					(AHB1PERIPH_BASEADDR + 0x3800)		//base address of RCC registers


/************************************* 4. Peripheral Register Structures **************************************/
//SPI Register Structure
typedef struct
{
	__vo	uint32_t CR1;					/*!<SPI control register 1,						Address Offset: 0x00>*/
	__vo	uint32_t CR2;					/*!<SPI control register 2,						Address Offset: 0x04>*/
	__vo	uint32_t SR;					/*!<SPI status register 1,						Address Offset: 0x08>*/
	__vo	uint32_t DR;					/*!<SPI data register 1,						Address Offset: 0x0C>*/
	__vo	uint32_t CRCPR;					/*!<SPI CRC polynomial register,				Address Offset: 0x10>*/
	__vo	uint32_t RXCRCR;				/*!<SPI RX CRC register,						Address Offset: 0x14>*/
	__vo	uint32_t TXCRCR;				/*!<SPI TX CRC register,						Address Offset: 0x18>*/
	__vo	uint32_t I2SCFGR;				/*!<SPI_I2S configuration register,				Address Offset: 0x1C>*/
	__vo	uint32_t I2SPR;					/*!<SPI_I2S prescaler register,					Address Offset: 0x20>*/
}SPI_RegDef_t;

//USART Register Structure
typedef struct
{
	__vo	uint32_t SR;					/*!<Short Description,						Address Offset: 0x00>*/
	__vo	uint32_t DR;					/*!<Short Description,						Address Offset: 0x04>*/
	__vo	uint32_t BRR;					/*!<Short Description,						Address Offset: 0x08>*/
	__vo	uint32_t CR1;					/*!<Short Description,						Address Offset: 0x0C>*/
	__vo	uint32_t CR2;					/*!<Short Description,						Address Offset: 0x10>*/
	__vo	uint32_t CR3;					/*!<Short Description,						Address Offset: 0x14>*/
	__vo	uint32_t GTPR;					/*!<Short Description,						Address Offset: 0x18>*/
}USART_RegDef_t;

//I2C Register Structure
typedef struct
{
	__vo	uint32_t CR1;					/*!<Short Description,						Address Offset: 0x00>*/
	__vo	uint32_t CR2;					/*!<Short Description,						Address Offset: 0x04>*/
	__vo	uint32_t OAR1;					/*!<Short Description,						Address Offset: 0x08>*/
	__vo	uint32_t OAR2;					/*!<Short Description,						Address Offset: 0x0C>*/
	__vo	uint32_t DR;					/*!<Short Description,						Address Offset: 0x10>*/
	__vo	uint32_t SR1;					/*!<Short Description,						Address Offset: 0x14>*/
	__vo	uint32_t SR2;					/*!<Short Description,						Address Offset: 0x18>*/
	__vo	uint32_t CCR;					/*!<Short Description,						Address Offset: 0x1C>*/
	__vo	uint32_t TRISE;					/*!<Short Description,						Address Offset: 0x20>*/
	__vo	uint32_t FLTR;					/*!<Short Description,						Address Offset: 0x24>*/
}I2C_RegDef_t;

//SYSCFG Register Structure
typedef struct
{
	__vo	uint32_t MEMRMP;				/*!<Short Description,						Address Offset: 0x00>*/
	__vo	uint32_t PMC;					/*!<Short Description,						Address Offset: 0x04>*/
	__vo	uint32_t EXTICR[4];				/*!<Short Description,						Address Offset: 0x08-0x14>*/
	uint32_t	RESERVED1[2];				/*!<Short Description,						Address Offset: 0x18-0x1C>*/
	__vo	uint32_t CMPCR;					/*!<Short Description,						Address Offset: 0x20>*/
}SYSCFG_RegDef_t;

//EXTI Register Structure
typedef struct
{
	__vo	uint32_t IMR;					/*!<Short Description,						Address Offset: 0x00>*/
	__vo	uint32_t EMR;					/*!<Short Description,						Address Offset: 0x04>*/
	__vo	uint32_t RTSR;					/*!<Short Description,						Address Offset: 0x08>*/
	__vo	uint32_t FTSR;					/*!<Short Description,						Address Offset: 0x0C>*/
	__vo	uint32_t SWIER;					/*!<Short Description,						Address Offset: 0x10>*/
	__vo	uint32_t PR;					/*!<Short Description,						Address Offset: 0x14>*/
}EXTI_RegDef_t;

//GPIO Register Structure
typedef struct
{
	__vo	uint32_t MODER;					/*!<GPIO port mode register,						Address Offset: 0x00>*/
	__vo	uint32_t OTYPER;				/*!<GPIO port output type register,					Address Offset: 0x04>*/
	__vo	uint32_t OSPEEDR;				/*!<GPIO port output speed register					Address Offset: 0x08>*/
	__vo	uint32_t PUPDR;					/*!<GPIO port pull-up/pull-down register,			Address Offset: 0x0C>*/
	__vo	uint32_t IDR;					/*!<GPIO port input data register,					Address Offset: 0x10>*/
	__vo	uint32_t ODR;					/*!<GPIO port output data register,					Address Offset: 0x14>*/
	__vo	uint32_t BSRR;					/*!<GPIO port bit set/reset register,				Address Offset: 0x18>*/
	__vo	uint32_t LCKR;					/*!<GPIO port configuration lock register,			Address Offset: 0x1C>*/
	__vo	uint32_t AFR[2];				/*!<AFR[0]: GPIO alternate function low register, AFR[1] GPIO alternate function high register			Address Offset: 0x20>*/
}GPIO_RegDef_t;

//CRC Register Structure
typedef struct
{
	__vo	uint32_t DR;					/*!<Short Description,						Address Offset: 0x00>*/
	__vo	uint32_t IDR;					/*!<Short Description,						Address Offset: 0x04>*/
	__vo	uint32_t CR;					/*!<Short Description,						Address Offset: 0x08>*/
}CRC_RegDef_t;

//RCC Register Structure
typedef struct
{
	__vo	uint32_t CR;					/*!<Short Description,						Address Offset: 0x00>*/
	__vo	uint32_t PLLCFGR;				/*!<Short Description,						Address Offset: 0x04>*/
	__vo	uint32_t CFGR;					/*!<Short Description,						Address Offset: 0x08>*/
	__vo	uint32_t CIR;					/*!<Short Description,						Address Offset: 0x0C>*/
	__vo	uint32_t AHB1RSTR;				/*!<Short Description,						Address Offset: 0x10>*/
	__vo	uint32_t AHB2RSTR;				/*!<Short Description,						Address Offset: 0x14>*/
	__vo	uint32_t AHB3RSTR;				/*!<Short Description,						Address Offset: 0x18>*/
	uint32_t RESERVED0;						/*!<Short Description,						Address Offset: 0x1C>*/
	__vo	uint32_t APB1RSTR;				/*!<Short Description,						Address Offset: 0x20>*/
	__vo	uint32_t APB2RSTR;				/*!<Short Description,						Address Offset: 0x24>*/
	uint32_t RESERVED1[2];					/*!<Short Description,						Address Offset: 0x28>*/
	__vo	uint32_t AHB1ENR;				/*!<Short Description,						Address Offset: 0x30>*/
	__vo	uint32_t AHB2ENR;				/*!<Short Description,						Address Offset: 0x34>*/
	__vo	uint32_t AHB3ENR;				/*!<Short Description,						Address Offset: 0x38>*/
	uint32_t RESERVED2;						/*!<Short Description,						Address Offset: 0x3C>*/
	__vo	uint32_t APB1ENR;				/*!<Short Description,						Address Offset: 0x40>*/
	__vo	uint32_t APB2ENR;				/*!<Short Description,						Address Offset: 0x44>*/
	uint32_t RESERVED3[2];					/*!<Short Description,						Address Offset: 0x48>*/
	__vo	uint32_t AHB1LPENR;				/*!<Short Description,						Address Offset: 0x50>*/
	__vo	uint32_t AHB2LPENR;				/*!<Short Description,						Address Offset: 0x54>*/
	__vo	uint32_t AHB3LPENR;				/*!<Short Description,						Address Offset: 0x58>*/
	uint32_t RESERVED4;						/*!<Short Description,						Address Offset: 0x5C>*/
	__vo	uint32_t APB1LPENR;				/*!<Short Description,						Address Offset: 0x60>*/
	__vo	uint32_t APB2LPENR;				/*!<Short Description,						Address Offset: 0x64>*/
	uint32_t RESERVED5[2];					/*!<Short Description,						Address Offset: 0x68>*/
	__vo	uint32_t BDCR;					/*!<Short Description,						Address Offset: 0x70>*/
	__vo	uint32_t CSR;					/*!<Short Description,						Address Offset: 0x74>*/
	uint32_t RESERVED6[2];					/*!<Short Description,						Address Offset: 0x78>*/
	__vo	uint32_t SSCGR;					/*!<Short Description,						Address Offset: 0x80>*/
	__vo	uint32_t PLLI2SCFGR;			/*!<Short Description,						Address Offset: 0x84>*/
}RCC_RegDef_t;

//Peripheral Definitions
#define	GPIOA					((GPIO_RegDef_t *)GPIOA_BASEADDR)
#define	GPIOB					((GPIO_RegDef_t *)GPIOB_BASEADDR)
#define	GPIOC					((GPIO_RegDef_t *)GPIOC_BASEADDR)
#define	GPIOD					((GPIO_RegDef_t *)GPIOD_BASEADDR)
#define	GPIOE					((GPIO_RegDef_t *)GPIOE_BASEADDR)
#define	GPIOF					((GPIO_RegDef_t *)GPIOF_BASEADDR)
#define	GPIOG					((GPIO_RegDef_t *)GPIOG_BASEADDR)
#define	GPIOH					((GPIO_RegDef_t *)GPIOH_BASEADDR)
#define	GPIOI					((GPIO_RegDef_t *)GPIOI_BASEADDR)

#define RCC						((RCC_RegDef_t *)RCC_BASEADDR)

#define EXTI					((EXTI_RegDef_t *)EXTI_BASEADDR)

#define SYSCFG					((SYSCFG_RegDef_t *)SYSCFG_BASEADDR)

#define SPI1					((SPI_RegDef_t *)SPI1_BASEADDR)
#define SPI2					((SPI_RegDef_t *)SPI2_BASEADDR)
#define SPI3					((SPI_RegDef_t *)SPI3_BASEADDR)

#define I2C1					((I2C_RegDef_t *)I2C1_BASEADDR)
#define I2C2					((I2C_RegDef_t *)I2C2_BASEADDR)
#define I2C3					((I2C_RegDef_t *)I2C3_BASEADDR)

#define USART1					((USART_RegDef_t *)USART1_BASEADDR)
#define USART2					((USART_RegDef_t *)USART2_BASEADDR)
#define USART3					((USART_RegDef_t *)USART3_BASEADDR)
#define UART4					((USART_RegDef_t *)UART4_BASEADDR)
#define UART5					((USART_RegDef_t *)UART5_BASEADDR)
#define USART6					((USART_RegDef_t *)USART6_BASEADDR)

/************************************* 5. Clock Enable/Disable Macros **************************************/

/*
 * Clock Enable Macros for SPIx Peripherals
 */

#define SPI1_PCLK_EN()				(RCC->APB2ENR |= (1<<12))
#define SPI2_PCLK_EN()				(RCC->APB1ENR |= (1<<14))
#define SPI3_PCLK_EN()				(RCC->APB1ENR |= (1<<15))

/*
 * Clock Enable Macros for USARTx Peripherals
 */

#define USART1_PCLK_EN()			(RCC->APB2ENR |= (1<<4))
#define USART2_PCLK_EN()			(RCC->APB1ENR |= (1<<17))
#define USART3_PCLK_EN()			(RCC->APB1ENR |= (1<<18))
#define UART4_PCLK_EN()				(RCC->APB1ENR |= (1<<19))
#define UART5_PCLK_EN()				(RCC->APB1ENR |= (1<<20))
#define USART6_PCLK_EN()			(RCC->APB2ENR |= (1<<5))
/*
 * Clock Enable Macros for I2Cx Peripherals
 */

#define I2C1_PCLK_EN()				(RCC->APB1ENR |= (1<<21))
#define I2C2_PCLK_EN()				(RCC->APB1ENR |= (1<<22))
#define I2C3_PCLK_EN()				(RCC->APB1ENR |= (1<<23))

/*
 * Clock Enable Macros for GPIOx Peripherals
 */

#define GPIOA_PCLK_EN()				(RCC->AHB1ENR |= (1<<0))
#define GPIOB_PCLK_EN()				(RCC->AHB1ENR |= (1<<1))
#define GPIOC_PCLK_EN()				(RCC->AHB1ENR |= (1<<2))
#define GPIOD_PCLK_EN()				(RCC->AHB1ENR |= (1<<3))
#define GPIOE_PCLK_EN()				(RCC->AHB1ENR |= (1<<4))
#define GPIOF_PCLK_EN()				(RCC->AHB1ENR |= (1<<5))
#define GPIOG_PCLK_EN()				(RCC->AHB1ENR |= (1<<6))
#define GPIOH_PCLK_EN()				(RCC->AHB1ENR |= (1<<7))
#define GPIOI_PCLK_EN()				(RCC->AHB1ENR |= (1<<8))

/*
 * Clock Enable Macros for SYSCFG Peripherals
 */

#define SYSCFG_PCLK_EN()			(RCC->APB2ENR |= (1<<14))

/*
 * Clock Disable Macros for SPIx Peripherals
 */

#define SPI1_PCLK_DI()				(RCC->APB2ENR &= ~(1<<12))
#define SPI2_PCLK_DI()				(RCC->APB1ENR &= ~(1<<14))
#define SPI3_PCLK_DI()				(RCC->APB1ENR &= ~(1<<15))

/*
 * Clock Disable Macros for USARTx Peripherals
 */

#define USART1_PCLK_DI()			(RCC->APB2ENR &= ~(1<<4))
#define USART2_PCLK_DI()			(RCC->APB1ENR &= ~(1<<17))
#define USART3_PCLK_DI()			(RCC->APB1ENR &= ~(1<<18))
#define UART4_PCLK_DI()				(RCC->APB1ENR &= ~(1<<19))
#define UART5_PCLK_DI()				(RCC->APB1ENR &= ~(1<<20))
#define USART6_PCLK_DI()			(RCC->APB2ENR &= ~(1<<5))
/*
 * Clock Disable Macros for I2Cx Peripherals
 */

#define I2C1_PCLK_DI()				(RCC->APB1ENR &= ~(1<<21))
#define I2C2_PCLK_DI()				(RCC->APB1ENR &= ~(1<<22))
#define I2C3_PCLK_DI()				(RCC->APB1ENR &= ~(1<<23))

/*
 * Clock Disable Macros for GPIOx Peripherals
 */

#define GPIOA_PCLK_DI()				(RCC->AHB1ENR &= ~(1<<0))
#define GPIOB_PCLK_DI()				(RCC->AHB1ENR &= ~(1<<1))
#define GPIOC_PCLK_DI()				(RCC->AHB1ENR &= ~(1<<2))
#define GPIOD_PCLK_DI()				(RCC->AHB1ENR &= ~(1<<3))
#define GPIOE_PCLK_DI()				(RCC->AHB1ENR &= ~(1<<4))
#define GPIOF_PCLK_DI()				(RCC->AHB1ENR &= ~(1<<5))
#define GPIOG_PCLK_DI()				(RCC->AHB1ENR &= ~(1<<6))
#define GPIOH_PCLK_DI()				(RCC->AHB1ENR &= ~(1<<7))
#define GPIOI_PCLK_DI()				(RCC->AHB1ENR &= ~(1<<8))

/*
 * Clock Disable Macros for SYSCFG Peripherals
 */

#define SYSCFG_PCLK_DI()			(RCC->APB2ENR &= ~(1<<14))


/************************************* 6. IRQ Definitions **************************************/

#define IRQ_NO_EXTI0				6
#define IRQ_NO_EXTI1				7
#define IRQ_NO_EXTI2				8
#define IRQ_NO_EXTI3				9
#define IRQ_NO_EXTI4				10
#define IRQ_NO_EXTI9_5				23
#define IRQ_NO_I2C1_EV				31
#define IRQ_NO_I2C1_ER				32
#define IRQ_NO_I2C2_EV				33
#define IRQ_NO_I2C2_ER				34
#define IRQ_NO_SPI1					35
#define IRQ_NO_SPI2					36
#define IRQ_NO_USART1				37
#define IRQ_NO_USART2				38
#define IRQ_NO_USART3				39
#define IRQ_NO_EXTI15_10			40
#define IRQ_NO_SPI3					51
#define IRQ_NO_UART4				52
#define IRQ_NO_UART5				53
#define IRQ_NO_USART6				71
#define IRQ_NO_I2C3_EV				72
#define IRQ_NO_I2C3_ER				73

#define NVIC_IRQ_PRIO0				0
#define NVIC_IRQ_PRIO1				1
#define NVIC_IRQ_PRIO2				2
#define NVIC_IRQ_PRIO3				3
#define NVIC_IRQ_PRIO4				4
#define NVIC_IRQ_PRIO5				5
#define NVIC_IRQ_PRIO6				6
#define NVIC_IRQ_PRIO7				7
#define NVIC_IRQ_PRIO8				8
#define NVIC_IRQ_PRIO9				9
#define NVIC_IRQ_PRIO10				10
#define NVIC_IRQ_PRIO11				11
#define NVIC_IRQ_PRIO12				12
#define NVIC_IRQ_PRIO13				13
#define NVIC_IRQ_PRIO14				14
#define NVIC_IRQ_PRIO15				15

/************************************* 7. Bit Position Definitions **************************************/
/*
 * SPI_CR1 bit position definitions
 */

#define SPI_CR1_CPHA				0
#define SPI_CR1_CPOL				1
#define SPI_CR1_MSTR				2
#define SPI_CR1_BR					3
#define SPI_CR1_SPE					6
#define SPI_CR1_LSBFIRST			7
#define SPI_CR1_SSI					8
#define SPI_CR1_SSM					9
#define SPI_CR1_RXONLY				10
#define SPI_CR1_DFF					11
#define SPI_CR1_CRCNEXT				12
#define SPI_CR1_CRCEN				13
#define SPI_CR1_BIDIOE				14
#define SPI_CR1_BIDIMODE			15

/*
 * SPI_CR2 bit position definitions
 */
#define SPI_CR2_RXDMAEN				0
#define SPI_CR2_TXDMAEN				1
#define SPI_CR2_SSOE				2
#define SPI_CR2_FRF					4
#define SPI_CR2_ERRIE				5
#define SPI_CR2_RXNEIE				6
#define SPI_CR2_TXEIE				7

/*
 * SPI_SR bit position definitions
 */
#define SPI_SR_RXNE					0
#define SPI_SR_TXE					1
#define SPI_SR_CHSIDE				2
#define SPI_SR_UDR					3
#define SPI_SR_CRCERR				4
#define SPI_SR_MODF					5
#define SPI_SR_OVR					6
#define SPI_SR_BSY					7
#define SPI_SR_FRE					8

/*
 * I2C_CR1 bit position definitions
 */
#define	I2C_CR1_PE					0
#define	I2C_CR1_SMBUS				1
#define	I2C_CR1_SMBTYPE				3
#define	I2C_CR1_ENARP				4
#define	I2C_CR1_ENPEC				5
#define	I2C_CR1_ENGC				6
#define	I2C_CR1_NOSTRETCH			7
#define	I2C_CR1_START				8
#define	I2C_CR1_STOP				9
#define	I2C_CR1_ACK					10
#define	I2C_CR1_POS					11
#define	I2C_CR1_PEC					12
#define	I2C_CR1_ALERT				13
#define	I2C_CR1_SWRST				15

/*
 * I2C_CR2 bit position definitions
 */
#define	I2C_CR2_FREQ				0
#define	I2C_CR2_ITERREN				8
#define	I2C_CR2_ITEVTEN				9
#define	I2C_CR2_ITBUFEN				10
#define	I2C_CR2_DMAEN				11
#define	I2C_CR2_LAST				12

/*
 * I2C_SR1 bit position definitions
 */
#define	I2C_SR1_SB					0
#define	I2C_SR1_ADDR				1
#define	I2C_SR1_BTF					2
#define	I2C_SR1_ADD10				3
#define	I2C_SR1_STOPF				4
#define	I2C_SR1_RXNE				6
#define	I2C_SR1_TXE					7
#define	I2C_SR1_BERR				8
#define	I2C_SR1_ARLO				9
#define	I2C_SR1_AF					10
#define	I2C_SR1_OVR					11
#define	I2C_SR1_PECERR				12
#define	I2C_SR1_TIMEOUT				14
#define	I2C_SR1_SMBALERT			15

/*
 * I2C_SR2 bit position definitions
 */
#define	I2C_SR2_MSL					0
#define	I2C_SR2_BUSY				1
#define	I2C_SR2_TRA					2
#define	I2C_SR2_GENCALL				4
#define	I2C_SR2_SMBDEFAULT			5
#define	I2C_SR2_SMBHOST				6
#define	I2C_SR2_DUALF				7
#define	I2C_SR2_PEC					8

/*
 * I2C_CCR bit position definitions
 */
#define	I2C_CCR_CCR					0
#define	I2C_CCR_DUTY				14
#define	I2C_CCR_FS					15

/*
 * USART_SR bit position definitions
 */
#define	USART_SR_PE					0
#define	USART_SR_FE					1
#define	USART_SR_NF					2
#define	USART_SR_ORE				3
#define	USART_SR_IDLE				4
#define	USART_SR_RXNE				5
#define	USART_SR_TC					6
#define	USART_SR_TXE				7
#define	USART_SR_LBD				8
#define	USART_SR_CTS				9

/*
 * USART_BRR bit position definitions
 */
#define USART_BRR_DIV_FRACTION		0
#define USART_BRR_DIV_MANTISSA		4

/*
 * USART_CR1 bit position definitions
 */
#define USART_CR1_SBK				0
#define USART_CR1_RWU				1
#define USART_CR1_RE				2
#define USART_CR1_TE				3
#define USART_CR1_IDLEIE			4
#define USART_CR1_RXNEIE			5
#define USART_CR1_TCIE				6
#define USART_CR1_TXEIE				7
#define USART_CR1_PEIE				8
#define USART_CR1_PS				9
#define USART_CR1_PCE				10
#define USART_CR1_WAKE				11
#define USART_CR1_M					12
#define USART_CR1_UE				13
#define USART_CR1_OVER8				15

/*
 * USART_CR2 bit position definitions
 */
#define USART_CR2_ADD				0
#define USART_CR2_LBDL				5
#define USART_CR2_LBDIE				6
#define USART_CR2_LBCL				8
#define USART_CR2_CPHA				9
#define USART_CR2_CPOL				10
#define USART_CR2_CLKEN				11
#define USART_CR2_STOP				12
#define USART_CR2_LINEN				14

/*
 * USART_CR3 bit position definitions
 */
#define USART_CR3_EIE				0
#define USART_CR3_IREN				1
#define USART_CR3_IRLP				2
#define USART_CR3_HDSEL				3
#define USART_CR3_NACK				4
#define USART_CR3_SCEN				5
#define USART_CR3_DMAR				6
#define USART_CR3_DMAT				7
#define USART_CR3_RTSE				8
#define USART_CR3_CTSE				9
#define USART_CR3_CTSIE				10
#define USART_CR3_ONEBIT			11

/*
 * USART_GTPR bit position definitions
 */
#define	USART_GTPR_PSC				0
#define	USART_GTPR_GT				8

/*
 * RCC_CFGR bit position definitions
 */
#define RCC_CFGR_SW					0
#define RCC_CFGR_SWS				2
#define RCC_CFGR_HPRE				4
#define RCC_CFGR_PPRE1				10
#define RCC_CFGR_PPRE2				13
#define RCC_CFGR_RTCPRE				16
#define RCC_CFGR_MCO1				21
#define RCC_CFGR_I2SSRC				23
#define RCC_CFGR_MCO1PRE			24
#define RCC_CFGR_MCO2PRE			27
#define RCC_CFGR_MCO2				30

/************************************* X. Other Macros **************************************/
/*
 * GPIOx Reset Macros
 */
#define GPIOA_REG_RESET()			do{ (RCC->AHB1RSTR |= (1<<0)); (RCC->AHB1RSTR &= ~(1<<0)); }while(0)
#define GPIOB_REG_RESET()			do{ (RCC->AHB1RSTR |= (1<<1)); (RCC->AHB1RSTR &= ~(1<<1)); }while(0)
#define GPIOC_REG_RESET()			do{ (RCC->AHB1RSTR |= (1<<2)); (RCC->AHB1RSTR &= ~(1<<2)); }while(0)
#define GPIOD_REG_RESET()			do{ (RCC->AHB1RSTR |= (1<<3)); (RCC->AHB1RSTR &= ~(1<<3)); }while(0)
#define GPIOE_REG_RESET()			do{ (RCC->AHB1RSTR |= (1<<4)); (RCC->AHB1RSTR &= ~(1<<4)); }while(0)
#define GPIOF_REG_RESET()			do{ (RCC->AHB1RSTR |= (1<<5)); (RCC->AHB1RSTR &= ~(1<<5)); }while(0)
#define GPIOG_REG_RESET()			do{ (RCC->AHB1RSTR |= (1<<6)); (RCC->AHB1RSTR &= ~(1<<6)); }while(0)
#define GPIOH_REG_RESET()		    do{ (RCC->AHB1RSTR |= (1<<7)); (RCC->AHB1RSTR &= ~(1<<7)); }while(0)
#define GPIOI_REG_RESET()			do{ (RCC->AHB1RSTR |= (1<<8)); (RCC->AHB1RSTR &= ~(1<<8)); }while(0)

/*
 * SPIx Reset Macros
 */
#define SPI1_REG_RESET()			do{ (RCC->APB2RSTR |= (1<<12)); (RCC->APB2RSTR &= ~(1<<12)); }while(0)
#define SPI2_REG_RESET()			do{ (RCC->APB1RSTR |= (1<<14)); (RCC->APB1RSTR &= ~(1<<14)); }while(0)
#define SPI3_REG_RESET()			do{ (RCC->APB1RSTR |= (1<<15)); (RCC->APB1RSTR &= ~(1<<15)); }while(0)

/*
 * I2Cx Reset Macros
 */
#define I2C1_REG_RESET()			do{ (RCC->APB1RSTR |= (1<<21)); (RCC->APB1RSTR &= ~(1<<21)); }while(0)
#define I2C2_REG_RESET()			do{ (RCC->APB1RSTR |= (1<<22)); (RCC->APB1RSTR &= ~(1<<22)); }while(0)
#define I2C3_REG_RESET()			do{ (RCC->APB1RSTR |= (1<<23)); (RCC->APB1RSTR &= ~(1<<23)); }while(0)

/*
 * USARTx Reset Macros
 */
#define USART1_REG_RESET()			do{ (RCC->APB2RSTR |= (1<<4)); (RCC->APB2RSTR &= ~(1<<4)); }while(0)
#define USART2_REG_RESET()			do{ (RCC->APB1RSTR |= (1<<17)); (RCC->APB1RSTR &= ~(1<<17)); }while(0)
#define USART3_REG_RESET()			do{ (RCC->APB1RSTR |= (1<<18)); (RCC->APB1RSTR &= ~(1<<18)); }while(0)
#define UART4_REG_RESET()			do{ (RCC->APB1RSTR |= (1<<19)); (RCC->APB1RSTR &= ~(1<<19)); }while(0)
#define UART5_REG_RESET()			do{ (RCC->APB1RSTR |= (1<<20)); (RCC->APB1RSTR &= ~(1<<20)); }while(0)
#define USART6_REG_RESET()			do{ (RCC->APB2RSTR |= (1<<5)); (RCC->APB2RSTR &= ~(1<<5)); }while(0)

/*
 * GPIO Port Code Macros
 */

#define GPIO_BASEADDR_TO_CODE(x)	((x == GPIOA) ? 0 :\
									(x == GPIOB) ? 1 :\
									(x == GPIOC) ? 2 :\
									(x == GPIOD) ? 3 :\
									(x == GPIOE) ? 4 :\
									(x == GPIOF) ? 5 :\
									(x == GPIOG) ? 6 :\
									(x == GPIOH) ? 7 :\
									(x == GPIOI) ? 8 :0)
//Generic Macros
#define	ENABLE					1
#define DISABLE					0
#define SET						ENABLE
#define RESET					DISABLE
#define GPIO_PIN_SET			SET
#define GPIO_PIN_RESET			RESET
#define FLAG_SET				SET
#define FLAG_RESET				RESET



#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx_i2c_driver.h"
#include "stm32f407xx_usart_driver.h"
#include "stm32f407xx_rcc_driver.h"

#endif /* INC_STM32F407XX_H_ */
