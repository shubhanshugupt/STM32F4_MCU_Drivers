/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: 15-Feb-2021
 *      Author: shubh
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h"

typedef struct
{
	uint8_t	SPI_DeviceMode;				// Possible values from @SPI_DEVICE_MODE
	uint8_t	SPI_BusConfig;				// Possible values from @SPI_BUS_CONFIGS
	uint8_t	SPI_DFF;					// Possible values from @SPI_DATA_FRAME_FORMAT
	uint8_t	SPI_CPHA;					// Possible values from @SPI_CPHA
	uint8_t	SPI_CPOL;					// Possible values from @SPI_CPOL
	uint8_t	SPI_SSM;					// Possible values from @SPI_SSM
	uint8_t	SPI_SCLKSpeed;				// Possible values from SPI_SCLK_SPEED
}SPI_PinConfig_t;

typedef struct
{
	SPI_RegDef_t *pSPIx;
	SPI_PinConfig_t SPI_PinConfig;
}SPI_Handle_t;


/*--------------------------SPI Device Modes--------------------------------*/
/*	@SPI_DEVICE_MODE	*/
/*	SPI_CR1 bit2: MSTR	*/
#define	SPI_MODE_SLAVE			0			//Default value
#define	SPI_MODE_MASTER			1

/*------------------------SPI Bus Configurations-----------------------------*/
/*	@SPI_BUS_CONFIGS	*/
/*	SPI_CR1 bit15,14,10	*/
#define	SPI_BUS_FULL_DUP		1
#define SPI_BUS_HALF_DUP		2
#define	SPI_BUS_SIMPLEX_RX		3
#define	SPI_BUS_SIMPLEX_TX		4

/*-----------------------SPI Data Frame Format------------------------------*/
/*	@SPI_DATA_FRAME_FORMAT	*/
/*	SPI_CR1 bit11	*/
#define SPI_DFF_8BIT			0			//Default value
#define	SPI_DFF_16BIT			1

/*-------------------------SPI SCLK Phase-----------------------------------*/
/*	@SPI_CPHA	*/
/*	SPI_CR1 bit0	*/
#define	SPI_CPHA_FIRST_EDGE		0			//Default value
#define	SPI_CPHA_SECOND_EDGE	1

/*------------------------SPI SCLK Polarity---------------------------------*/
/*	@SPI_CPOL	*/
/*	SPI_CR1 bit1	*/
#define SPI_CPOL_LOW			0			//Default value
#define SPI_CPOL_HIGH			1

/*----------------------SPI Slave Select Mgmt-------------------------------*/
/*	@SPI_SSM	*/
/*	SPI_CR1 bit8,9	*/
#define SPI_SSM_DI				0			//Default value
#define	SPI_SSM_EN				1

/*-------------------------SPI SCLK Speed------------------------------------*/
/*	@SPI_SCLK_SPEED	*/
/*	SPI_CR1 bit 3,4,5	*/
#define SPI_SCLK_BY_2			0			//Default value
#define SPI_SCLK_BY_4			1
#define SPI_SCLK_BY_8			2
#define SPI_SCLK_BY_16			3
#define SPI_SCLK_BY_32			4
#define SPI_SCLK_BY_64			5
#define SPI_SCLK_BY_128			6
#define SPI_SCLK_BY_256			7


/*--------------------------SPI Flags----------------------------------------*/
#define SPI_RXNE_FLAG			(1 << SPI_SR_RXNE)
#define SPI_TXE_FLAG			(1 << SPI_SR_TXE)
#define SPI_BUSY_FLAG			(1 << SPI_SR_BSY)


/*-----------------------------------------------------------------------------
  					APIs supported by this Driver
 ----------------------------------------------------------------------------*/

//	Peripheral Clock Setup
void SPI_PClkCtrl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

//	SPI Init and DeInit
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

//	Data Send and Receive
void SPI_DataSend(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t DataLen);
void SPI_DataReceive(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t DataLen);

//	IRQ Configuration and ISR Handling
void SPI_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);



#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
