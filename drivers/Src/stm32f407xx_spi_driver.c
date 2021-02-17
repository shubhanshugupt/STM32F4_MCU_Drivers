/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: 17-Feb-2021
 *      Author: shubh
 */

#include "stm32f407xx_gpio_driver.h"


//	Peripheral Clock Setup
void SPI_PClkCtrl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (pSPIx == SPI1)
		{
			SPI1_CLK_EN();
		}
		else if (pSPIx == SPI2)
		{
			SPI2_CLK_EN();
		}
		else if (pSPIx == SPI3)
		{
			SPI3_CLK_EN();
		}
	}
	else
	{
		if (pSPIx == SPI1)
		{
			SPI1_CLK_DI();
		}
		else if (pSPIx == SPI2)
		{
			SPI2_CLK_DI();
		}
		else if (pSPIx == SPI3)
		{
			SPI3_CLK_DI();
		}
	}
}

//	SPI Init and DeInit
void SPI_Init(SPI_Handle_t *pSPIHandle)
{

}
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{

}

//	Data Send and Receive
void SPI_DataSend(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t DataLen)
{

}
void SPI_DataReceive(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t DataLen)
{

}

//	IRQ Configuration and ISR Handling
void SPI_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

}
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{

}
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{

}

