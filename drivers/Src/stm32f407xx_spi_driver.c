/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: 17-Feb-2021
 *      Author: shubh
 */

#include "stm32f407xx_spi_driver.h"


//	Peripheral Clock Setup
/*----------------------------------------------------------------------------------------
 * @fn			-	SPI_PClkCtrl
 *
 * @brief		-	This function Enables or Disables the Peripheral clock for SPI
 *
 * @param[in]	-	base address of the SPI peripheral
 * @param[in]	-	ENABLE or DISABLE macros, defined in MCU header file "stm32f407xx.h"
 *
 * @return		-	none
 *
 * @note		-	none
 -----------------------------------------------------------------------------------------*/
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
/*----------------------------------------------------------------------------------------
 * @fn			-	SPI_Init
 *
 * @brief		-	This function initializes the peripheral, ie, configures all the registers
 *
 * @param[in]	-	pointer to the handle structure
 * @param[in]	-	none
 *
 * @return		-	none
 *
 * @note		-	none
 -----------------------------------------------------------------------------------------*/
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	uint32_t tempreg = 0;

	//1. Configure the device modes
	tempreg |= ( (pSPIHandle->SPI_PinConfig.SPI_DeviceMode) << SPI_CR1_MSTR );

	//2. Configure the BUS Config
	if (pSPIHandle->SPI_PinConfig.SPI_BusConfig == SPI_BUS_FULL_DUP)
	{
		//Use unidirectional mode, ie, Clear BIDIMODE
		tempreg &= ~(1 << SPI_CR1_BIDIMODE );
	}
	else if(pSPIHandle->SPI_PinConfig.SPI_BusConfig == SPI_BUS_HALF_DUP)
	{
		//Set BIDIMODE
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPI_PinConfig.SPI_BusConfig == SPI_BUS_SIMPLEX_RX)
	{
		//Clear BIDIMODE
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		//RXONLY bit must be set
		tempreg |= (1 << SPI_CR1_RXONLY);
	}

	//3. Configure the Data Frame Format
	tempreg |= ( (pSPIHandle->SPI_PinConfig.SPI_DFF) << SPI_CR1_DFF);

	//4. Configure the Clock Phase
	tempreg |= ( (pSPIHandle->SPI_PinConfig.SPI_CPHA) << SPI_CR1_CPHA);

	//5. Configure the Clock Polarity
	tempreg |= ( (pSPIHandle->SPI_PinConfig.SPI_CPOL) << SPI_CR1_CPOL);

	//6. Configure Slave Select Mode
	tempreg |= ( (pSPIHandle->SPI_PinConfig.SPI_SSM) << SPI_CR1_SSM);

	//7. Configure Serial Clock Speed
	tempreg |= ( (pSPIHandle->SPI_PinConfig.SPI_SCLKSpeed) << SPI_CR1_BR);

	pSPIHandle->pSPIx->CR1 = tempreg;
}

/*----------------------------------------------------------------------------------------
 * @fn			-	SPI_DeInit
 *
 * @brief		-	It resets the SPI peripheral registers
 *
 * @param[in]	-	Address of the peripheral
 * @param[in]	-	none
 *
 * @return		-	none
 *
 * @note		-	We have a special register in RCC peripheral, called AHB1 peripheral reset register
 * 					This will reset all the peripheral registers
 -----------------------------------------------------------------------------------------*/
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{

}

//	Data Send and Receive
/*----------------------------------------------------------------------------------------
 * @fn			-	SPI_DataSend
 *
 * @brief		-
 *
 * @param[in]	-	Address of the peripheral
 * @param[in]	-	none
 *
 * @return		-	none
 *
 * @note		-
 -----------------------------------------------------------------------------------------*/
void SPI_DataSend(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t DataLen)
{

}

/*----------------------------------------------------------------------------------------
 * @fn			-	SPI_DataSend
 *
 * @brief		-
 *
 * @param[in]	-	Address of the peripheral
 * @param[in]	-	none
 *
 * @return		-	none
 *
 * @note		-
 -----------------------------------------------------------------------------------------*/
void SPI_DataReceive(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t DataLen)
{

}

//	IRQ Configuration and ISR Handling
/*----------------------------------------------------------------------------------------
 * @fn			-	SPI_DataSend
 *
 * @brief		-
 *
 * @param[in]	-	Address of the peripheral
 * @param[in]	-	none
 *
 * @return		-	none
 *
 * @note		-
 -----------------------------------------------------------------------------------------*/
void SPI_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

}

/*----------------------------------------------------------------------------------------
 * @fn			-	SPI_DataSend
 *
 * @brief		-
 *
 * @param[in]	-	Address of the peripheral
 * @param[in]	-	none
 *
 * @return		-	none
 *
 * @note		-
 -----------------------------------------------------------------------------------------*/
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{

}

/*----------------------------------------------------------------------------------------
 * @fn			-	SPI_DataSend
 *
 * @brief		-
 *
 * @param[in]	-	Address of the peripheral
 * @param[in]	-	none
 *
 * @return		-	none
 *
 * @note		-
 -----------------------------------------------------------------------------------------*/
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{

}

