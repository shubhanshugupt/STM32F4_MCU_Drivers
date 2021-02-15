/*
 * stm32f4xx_spi_driver.h
 *
 *  Created on: 15-Feb-2021
 *      Author: shubh
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h"

typedef struct
{
	uint8_t	SPI_DeviceMode;
	uint8_t	SPI_BusConfig;
	uint8_t	SPI_DFF;
	uint8_t	SPI_CPHA;
	uint8_t	SPI_CPOL;
	uint8_t	SPI_SSM;
	uint8_t	SPI_SCLKSpeed;
}SPI_PinConfig_t;

typedef struct
{
	SPI_RegDef_t *pSPIx;
	SPI_PinConfig_t SPI_PinConfig;
}SPI_Handle_t;


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
