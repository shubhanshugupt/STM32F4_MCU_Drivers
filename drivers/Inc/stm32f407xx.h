/*
 * stm32f407xx.h
 *
 *  Created on: Jan 29, 2021
 *      Author: shubh
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#define __vo volatile
#include <stdint.h>
#include <stddef.h>


/*		Some generic macros		*/
#define ENABLE				1
#define	DISABLE				0
#define	SET					ENABLE
#define	RESET				DISABLE
#define FLAG_SET			SET
#define FLAG_RESET			RESET

/*		IRQ number for STM32F407x MCU	*/
#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI115_10	40

#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2			36
#define IRQ_NO_SPI3			51



#define NVIC_IRQ_PRI0		0
#define NVIC_IRQ_PRI1		1
#define NVIC_IRQ_PRI2		2
#define NVIC_IRQ_PRI3		3
#define NVIC_IRQ_PRI4		4
#define NVIC_IRQ_PRI5		5
#define NVIC_IRQ_PRI6		6
#define NVIC_IRQ_PRI7		7
#define NVIC_IRQ_PRI8		8
#define NVIC_IRQ_PRI9		9
#define NVIC_IRQ_PRI10		10
#define NVIC_IRQ_PRI11		11
#define NVIC_IRQ_PRI12		12
#define NVIC_IRQ_PRI13		13
#define NVIC_IRQ_PRI14		14
#define NVIC_IRQ_PRI15		15



/*------------------------Processor Specific Details------------------------------ */

/* NVIC ISER register addresses */
#define NVIC_ISER0			( (__vo uint32_t*) 0xE000E100 )
#define NVIC_ISER1			( (__vo uint32_t*) 0xE000E104 )
#define NVIC_ISER2			( (__vo uint32_t*) 0xE000E108 )
//We only need these 3 out of 8 registers, as there are only 80 interrupts covered by the MCU manufacturer.

/* NVIC ICER register addresses */
#define NVIC_ICER0			( (__vo uint32_t*) 0XE000E180 )
#define NVIC_ICER1			( (__vo uint32_t*) 0XE000E184 )
#define NVIC_ICER2			( (__vo uint32_t*) 0XE000E188 )

/* NVIC IPR register address */
#define NVIC_IPR_BASEADDR	( (__vo uint32_t*) 0xE000E400 )

#define PRIORITY_BITS_IMPLEMENTED	4


/* base addresses of the various address memory locations */
#define FLASH_BASEADDR		0x80000000U
#define SRAM1_BASEADDR		0x20000000U
#define SRAM2_BASEADDR		0x2001C000U
#define ROM					0x1FFF0000U
#define SRAM				SRAM1_BASEADDR			// the default RAM is SRAM1


/* Bus peripheral base addresses */
#define PERIPH_BASEADDR		0x40000000U
#define APB1PERIPH_BASEADDR	PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR	0x40010000U
#define AHB1PERIPH_BASEADDR	0x40020000U
#define AHB2PERIPH_BASEADDR	0x50000000U


/* Base addresses of peripherals connected to AHB1 bus */
#define RCC_BASEADDR		(AHB1PERIPH_BASEADDR + 0x3800)
#define GPIOA_BASEADDR		(AHB1PERIPH_BASEADDR + 0x0000)	//BaseAddress of bus + offset
#define GPIOB_BASEADDR		(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR		(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR		(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR		(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR		(AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR		(AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR		(AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR		(AHB1PERIPH_BASEADDR + 0x2000)


/* Base addresses of peripherals connected to APB1 bus */
#define I2C1_BASEADDR		(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR		(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR		(APB1PERIPH_BASEADDR + 0x5C00)

#define SPI2_BASEADDR		(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR		(APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR		(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR		(APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR		(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR		(APB1PERIPH_BASEADDR + 0x5000)


/* Base addresses of peripherals connected to APB2 bus */
#define EXTI_BASEADDR		(APB2PERIPH_BASEADDR + 0x3C00)
#define SPI1_BASEADDR		(APB2PERIPH_BASEADDR + 0x3000)
#define SYSCFG_BASEADDR		(APB2PERIPH_BASEADDR + 0x3800)
#define USART1_BASEADDR		(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR		(APB2PERIPH_BASEADDR + 0x1400)


/*------------Peripheral register definition structures------------*/

typedef struct
{
	__vo uint32_t MODER;	//Mode register				Address offset: 0x00
	__vo uint32_t OTYPER;	//Output type register		Address offset: 0x04
	__vo uint32_t OSPEEDR;	//Output speed register		Address offset: 0x08
	__vo uint32_t PUPDR;	//PullUp PullDown register	Address offset: 0x0C
	__vo uint32_t IDR;		//Input data register		Address offset: 0x10
	__vo uint32_t ODR;		//OUtput data register		Address offset: 0x14
	__vo uint32_t BSRR;		//Bit set/reset register	Address offset: 0x18
	__vo uint32_t LCKR;		//Config lock register		Address offset: 0x1C
	__vo uint32_t AFRL;		//Alternate func register	Address offset: 0x20
	__vo uint32_t AFRH;		//Alternate func register	Address offset: 0x24

}GPIO_RegDef_t;


typedef struct
{
	__vo uint32_t	CR;				//Address offset: 0x00
	__vo uint32_t	PLLCFGR;		//Address offset: 0x04
	__vo uint32_t	CFGR;			//Address offset: 0x08
	__vo uint32_t	CIR;			//Address offset: 0x0C
	__vo uint32_t	AHB1RSTR;		//Address offset: 0x10
	__vo uint32_t	AHB2RSTR;		//Address offset: 0x14
	__vo uint32_t	AHB3RSTR;		//Address offset: 0x18
	uint32_t 		Reserved1;		//Address offset: 0x1C
	__vo uint32_t	APB1RSTR;		//Address offset: 0x20
	__vo uint32_t	APB2RSTR;		//Address offset: 0x24
	uint32_t		Reserved2[2];	//Address offset: 0x28
	__vo uint32_t	AHB1ENR;		//Address offset: 0x30
	__vo uint32_t	AHB2ENR;		//Address offset: 0x34
	__vo uint32_t	AHB3ENR;		//Address offset: 0x38
	uint32_t		Reserved3;		//Address offset: 0x3C
	__vo uint32_t	APB1ENR;		//Address offset: 0x40
	__vo uint32_t	APB2ENR;		//Address offset: 0x44
	uint32_t		Reserved4[2];	//Address offset: 0x48
	__vo uint32_t	AHB1LPENR;		//Address offset: 0x50
	__vo uint32_t	AHB2LPENR;		//Address offset: 0x54
	__vo uint32_t	AHB3LPENR;		//Address offset: 0x58
	uint32_t		Reserved5;		//Address offset: 0x5C
	__vo uint32_t	APB1LPENR;		//Address offset: 0x60
	__vo uint32_t	APB2LPENR;		//Address offset: 0x64
	uint32_t		Reserved6[2];	//Address offset: 0x68
	__vo uint32_t	BDCR;			//Address offset: 0x70
	__vo uint32_t	CSR;			//Address offset: 0x74
	uint32_t		Reserved7[2];	//Address offset: 0x78
	__vo uint32_t	SSCGR;			//Address offset: 0x80
	__vo uint32_t	PLLI2SCFGR;		//Address offset: 0x84
}RCC_RegDef_t;


typedef struct
{
	// Each register has 23 bits available to configure
	__vo uint32_t	IMR;			//Address offset: 0x00
	__vo uint32_t	EMR;			//Address offset: 0x04
	__vo uint32_t	RTSR;			//Address offset: 0x08
	__vo uint32_t	FTSR;			//Address offset: 0x0C
	__vo uint32_t	SWIER;			//Address offset: 0x10
	__vo uint32_t	PR;				//Address offset: 0x14
}EXTI_RegDef_t;


typedef struct
{
	__vo uint32_t	MEMRMP;			//Address offset: 0x00
	__vo uint32_t	PMC;			//Address offset: 0x04
	__vo uint32_t	EXTICR[4];		//Address offset: 0x08-0x14
	uint32_t	Reserved[2];		//Address offset: 0x18-0x1C
	__vo uint32_t	CMPCR;			//Address offset: 0x20
}SYSCFG_RegDef_t;


typedef struct
{
	__vo uint32_t	CR1;			//Address offset: 0x00
	__vo uint32_t	CR2;			//Address offset: 0x04
	__vo uint32_t	SR;				//Address offset: 0x08
	__vo uint32_t	DR;				//Address offset: 0x0C
	__vo uint32_t	CRCPR;			//Address offset: 0x10
	__vo uint32_t	RXCRCR;			//Address offset: 0x14
	__vo uint32_t	TXCRCR;			//Address offset: 0x18
	__vo uint32_t	I2SCFGR;		//Address offset: 0x1C
	__vo uint32_t	I2SPR;			//Address offset: 0x20

}SPI_RegDef_t;


/* Peripheral definitions */
#define GPIOA	( (GPIO_RegDef_t*) GPIOA_BASEADDR )
#define GPIOB	( (GPIO_RegDef_t*) GPIOB_BASEADDR )
#define GPIOC	( (GPIO_RegDef_t*) GPIOC_BASEADDR )
#define GPIOD	( (GPIO_RegDef_t*) GPIOD_BASEADDR )
#define GPIOE	( (GPIO_RegDef_t*) GPIOE_BASEADDR )
#define GPIOF	( (GPIO_RegDef_t*) GPIOF_BASEADDR )
#define GPIOG	( (GPIO_RegDef_t*) GPIOG_BASEADDR )
#define GPIOH	( (GPIO_RegDef_t*) GPIOH_BASEADDR )
#define GPIOI	( (GPIO_RegDef_t*) GPIOI_BASEADDR )

#define RCC		( (RCC_RegDef_t*) RCC_BASEADDR )

#define EXTI	( (EXTI_RegDef_t*) EXTI_BASEADDR )

#define SYSCFG	( (SYSCFG_RegDef_t*) SYSCFG_BASEADDR )

#define SPI1	( (SPI_RegDef_t*) SPI1_BASEADDR )
#define SPI2	( (SPI_RegDef_t*) SPI2_BASEADDR )
#define SPI3	( (SPI_RegDef_t*) SPI3_BASEADDR )


/* Clock Enable/Disable Macros for GPIOx Peripherals */
#define GPIOA_CLK_EN()	( RCC->AHB1ENR |= (1 << 0) )
#define GPIOA_CLK_DI()	( RCC->AHB1ENR &= ~(1 << 0) )

#define GPIOB_CLK_EN()	( RCC->AHB1ENR |= (1 << 1) )
#define GPIOB_CLK_DI()	( RCC->AHB1ENR &= ~(1 << 1) )

#define GPIOC_CLK_EN()	( RCC->AHB1ENR |= (1 << 2) )
#define GPIOC_CLK_DI()	( RCC->AHB1ENR &= ~(1 << 2) )

#define GPIOD_CLK_EN()	( RCC->AHB1ENR |= (1 << 3) )
#define GPIOD_CLK_DI()	( RCC->AHB1ENR &= ~(1 << 3) )

#define GPIOE_CLK_EN()	( RCC->AHB1ENR |= (1 << 4) )
#define GPIOE_CLK_DI()	( RCC->AHB1ENR &= ~(1 << 4) )

#define GPIOF_CLK_EN()	( RCC->AHB1ENR |= (1 << 5) )
#define GPIOF_CLK_DI()	( RCC->AHB1ENR &= ~(1 << 5) )

#define GPIOG_CLK_EN()	( RCC->AHB1ENR |= (1 << 6) )
#define GPIOG_CLK_DI()	( RCC->AHB1ENR &= ~(1 << 6) )

#define GPIOH_CLK_EN()	( RCC->AHB1ENR |= (1 << 7) )
#define GPIOH_CLK_DI()	( RCC->AHB1ENR &= ~(1 << 7) )

#define GPIOI_CLK_EN()	( RCC->AHB1ENR |= (1 << 8) )
#define GPIOI_CLK_DI()	( RCC->AHB1ENR &= ~(1 << 8) )


/* Clock Enable/Disable macros for I2Cx peripherals */
#define I2C1_CLK_EN()	( RCC->APB1ENR |= (1 << 21) )
#define I2C1_CLK_DI()	( RCC->APB1ENR &= ~(1 << 21) )


/* Clock Enable/Disable macros for SYSCFG peripheral */
#define SYSCFG_CLK_EN()	( RCC->APB2ENR |= (1 << 14) )
#define SYSCFG_CLK_DI()	( RCC->APB2ENR &= ~(1 << 14) )


/* Clock Enable/Disable macros for SPIx peripherals */
#define SPI1_CLK_EN()	( RCC->APB2ENR |= (1 << 12) )
#define SPI1_CLK_DI()	( RCC->APB2ENR &= ~(1 << 12) )

#define SPI2_CLK_EN()	( RCC->APB1ENR |= (1 << 14) )
#define SPI2_CLK_DI()	( RCC->APB1ENR &= ~(1 << 14) )

#define SPI3_CLK_EN()	( RCC->APB1ENR |= (1 << 15) )
#define SPI3_CLK_DI()	( RCC->APB1ENR &= ~(1 << 15) )


/* GPIOx peripheral register reset macros */
#define GPIOA_RESET()	do {( RCC->AHB1RSTR |= (1 << 0) );	( RCC->AHB1RSTR &= ~(1 << 0) ); }while(0)
#define GPIOB_RESET()	do {( RCC->AHB1RSTR |= (1 << 1) );	( RCC->AHB1RSTR &= ~(1 << 1) ); }while(0)
#define GPIOC_RESET()	do {( RCC->AHB1RSTR |= (1 << 2) );	( RCC->AHB1RSTR &= ~(1 << 2) ); }while(0)
#define GPIOD_RESET()	do {( RCC->AHB1RSTR |= (1 << 3) );	( RCC->AHB1RSTR &= ~(1 << 3) ); }while(0)
#define GPIOE_RESET()	do {( RCC->AHB1RSTR |= (1 << 4) );	( RCC->AHB1RSTR &= ~(1 << 4) ); }while(0)
#define GPIOF_RESET()	do {( RCC->AHB1RSTR |= (1 << 5) );	( RCC->AHB1RSTR &= ~(1 << 5) ); }while(0)
#define GPIOG_RESET()	do {( RCC->AHB1RSTR |= (1 << 6) );	( RCC->AHB1RSTR &= ~(1 << 6) ); }while(0)
#define GPIOH_RESET()	do {( RCC->AHB1RSTR |= (1 << 7) );	( RCC->AHB1RSTR &= ~(1 << 7) ); }while(0)
#define GPIOI_RESET()	do {( RCC->AHB1RSTR |= (1 << 8) );	( RCC->AHB1RSTR &= ~(1 << 8) ); }while(0)


/*	SPIx peripheral register reset macros	*/
#define SPI1_RESET()	do {( RCC->APB2RSTR |= (1 << 12) ); ( RCC->APB2RSTR &= ~(1 <<12) ); }while(0)
#define SPI2_RESET()	do {( RCC->APB1RSTR |= (1 << 14) ); ( RCC->APB1RSTR &= ~(1 <<14) ); }while(0)
#define SPI3_RESET()	do {( RCC->APB1RSTR |= (1 << 15) ); ( RCC->APB1RSTR &= ~(1 <<15) ); }while(0)


/*-----------------BIT POSITION DEFINITIONS for SPI PERIPHERAL--------------------------*/
#define SPI_CR1_CPHA			0
#define SPI_CR1_CPOL			1
#define SPI_CR1_MSTR			2
#define SPI_CR1_BR				3
#define SPI_CR1_SPE				6
#define SPI_CR1_LSBFIRST		7
#define SPI_CR1_SSI				8
#define SPI_CR1_SSM				9
#define SPI_CR1_RXONLY			10
#define SPI_CR1_DFF				11
#define SPI_CR1_CRCNEXT			12
#define SPI_CR1_CRCEN			13
#define SPI_CR1_BIDIOE			14
#define SPI_CR1_BIDIMODE		15

#define SPI_CR2_RXDMAEN			0
#define SPI_CR2_TXDMAEN			1
#define SPI_CR2_SSOE			2
#define SPI_CR2_FRF				4
#define SPI_CR2_ERRIE			5
#define SPI_CR2_RXNEIE			6
#define SPI_CR2_TXEIE			7

#define SPI_SR_RXNE				0
#define SPI_SR_TXE				1
#define SPI_SR_CHSIDE			2
#define SPI_SR_UDR				3
#define SPI_SR_CRCERR			4
#define SPI_SR_MODF				5
#define SPI_SR_OVR				6
#define SPI_SR_BSY				7
#define SPI_SR_FRE				8



#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"

#endif /* INC_STM32F407XX_H_ */
