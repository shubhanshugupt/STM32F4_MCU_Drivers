/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: 17-Feb-2021
 *      Author: shubh
 */

#include "stm32f407xx_spi_driver.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);


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

	//0. Enable the Peripheral clock				(Updated on 19-Feb-2021)
	SPI_PClkCtrl(pSPIHandle->pSPIx, ENABLE);

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
 * @note		-	We have a special register in RCC peripheral, called APB1 and APB2 peripheral reset register
 * 					This will reset all the peripheral registers
 -----------------------------------------------------------------------------------------*/
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if (pSPIx == SPI1)
	{
		SPI1_RESET();
	}
	else if (pSPIx == SPI2)
	{
		SPI2_RESET();
	}
	else if (pSPIx == SPI3)
	{
		SPI3_RESET();
	}
}

uint8_t	FlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if ( pSPIx->SR & FlagName )
	{
		return FLAG_SET;
	}
	else
	{
		return FLAG_RESET;
	}
}

//	Data Send and Receive
/*----------------------------------------------------------------------------------------
 * @fn			-	SPI_DataSend
 *
 * @brief		-
 *
 * @param[in]	-	Address of the peripheral
 * @param[in]	-	Pointer to TX buffer
 * @param[in]	-	Data length in bytes
 *
 * @return		-	none
 *
 * @note		- This is a blocking call
 -----------------------------------------------------------------------------------------*/
void SPI_DataSend(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t DataLen)
{
	while (DataLen > 0)
	{
		//1. Wait until TX buffer is empty
		while ( ( FlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET) );

		//2. Check the DFF bit in CR1 register
		if ( ( pSPIx->CR1 & (1 << SPI_CR1_DFF) ) )
		{
			// 16 bit data format
			//1. Load the data in DR register
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			//2. Decrease DataLen twice
			DataLen--;
			DataLen--;
			//3. Increment TX buffer
			(uint16_t*)pTxBuffer++;
		}
		else
		{
			// 8 bit data format
			//1. Load the data in DR register
			pSPIx->DR = *(pTxBuffer);
			//2. Decrease DataLen
			DataLen--;
			//3. Increment TX buffer
			pTxBuffer++;
		}
	}
}

/*----------------------------------------------------------------------------------------
 * @fn			-	SPI_DataReceive
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
	while (DataLen > 0)
	{
		//1. Wait until Rx buffer is available
		while (FlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

		//2. Check the DFF bit in CR1 register
		if ( ( pSPIx->CR1 & (1 << SPI_CR1_DFF) ) )
		{
			// 16 bit data format
			//1. Load the data from DR to buffer
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			//2. Decrease DataLen twice
			DataLen--;
			DataLen--;
			//3. Increment RX Buffer
			(uint16_t*)pRxBuffer++;
		}
		else
		{
			// 8 bit data format
			//1. Load the data from DR to buffer
			*(pRxBuffer) = pSPIx->DR;
			//2. Decrease DataLen
			DataLen--;
			//3. Increment RX Buffer
			pRxBuffer++;
		}
	}
}


//	Data Send and Receive by Interrupt
uint8_t SPI_DataSendByIntr(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t DataLen)
{
	uint8_t state = pSPIHandle->TxState;
	if (state != SPI_BUSY_IN_TX)
	{
		//1. Save the TX buffer and DataLen information in some global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = DataLen;

		//2. Mark SPI state as busy during transmission,
		//		so that no other program will take over the same SPI peripheral
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		//3. Enable TXEIE bit to get the interrupt when TXE flag is set
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
	}
	return state;
}

uint8_t SPI_DataReceiveByIntr(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t DataLen)
{
	uint8_t state = pSPIHandle->RxState;
	if (state != SPI_BUSY_IN_RX)
	{
		//1. Save the TX buffer and DataLen information in some global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = DataLen;

		//2. Mark SPI state as busy during transmission,
		//		so that no other program will take over the same SPI peripheral
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		//3. Enable TXEIE bit to get the interrupt when TXE flag is set
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
	}
	return state;

}


//	IRQ Configuration and ISR Handling
/*----------------------------------------------------------------------------------------
 * @fn			-	SPI_IRQConfig
 *
 * @brief		-	It configures an Interrupt, ie enable or disable
 *
 * @param[in]	-	IRQ number of the interrupt
 * @param[in]	-	ENABLE or DISABLE macros
 *
 * @return		-	none
 *
 * @note		-
 -----------------------------------------------------------------------------------------*/
void SPI_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	uint8_t shift = IRQNumber%32;
	if (EnorDi == ENABLE)
	{
		if (IRQNumber < 32)
		{
			*NVIC_ISER0 |= ( 1 << shift );

		}else if (IRQNumber > 31 && IRQNumber < 64)
		{
			*NVIC_ISER1 |= (1 << shift);

		}else if (IRQNumber > 63 && IRQNumber < 96)
		{
			*NVIC_ISER2 |= (1 << shift);

		}
	}else
	{
		if (IRQNumber < 32)
		{
			*NVIC_ICER0 |= ( 1 << shift );

		}else if (IRQNumber > 31 && IRQNumber < 64)
		{
			*NVIC_ICER1 |= (1 << shift);

		}else if (IRQNumber > 63 && IRQNumber < 96)
		{
			*NVIC_ICER2 |= (1 << shift);
		}
	}
}

/*----------------------------------------------------------------------------------------
 * @fn			-	SPI_IRQPriorityConfig
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
	uint8_t RegNo	=	IRQNumber/4;
	uint8_t section	=	IRQNumber%4;
	uint8_t shift	=	( 8*section ) + ( 8 - PRIORITY_BITS_IMPLEMENTED);

	*(NVIC_IPR_BASEADDR + RegNo) = ( IRQPriority << shift );
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
	uint8_t temp1, temp2;
	//1. Check for TXE
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

	if (temp1 && temp2)
	{
		// Handle TX interrupt
		spi_txe_interrupt_handle(pSPIHandle);
	}

	//2. Check for RXNE
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

	if (temp1 && temp2)
	{
		// Handle RX interrupt
		spi_rxne_interrupt_handle(pSPIHandle);
	}

	//3. Check for OVR flag
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

	if (temp1 && temp2)
	{
		//Handle overrun interrupt
		spi_ovr_err_interrupt_handle(pSPIHandle);
	}
}

/*----------------------------------------------------------------------------------------
 * @fn			-	SPI_PCtrl
 *
 * @brief		-
 *
 * @param[in]	-	Address of the peripheral
 * @param[in]	-	ENABLE or DISABLE macros, defined in MCU header file "stm32f407xx.h"
 *
 * @return		-	none
 *
 * @note		-
 -----------------------------------------------------------------------------------------*/
void SPI_PCtrl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

/*----------------------------------------------------------------------------------------
 * @fn			-	SPI_SSIConfig
 *
 * @brief		-	Configures the SSI bit in CR1 register, when Software Slave mode is enabled.
 *
 * @param[in]	-	Address of the peripheral
 * @param[in]	-	ENABLE or DISABLE macros, defined in MCU header file "stm32f407xx.h"
 *
 * @return		-	none
 *
 * @note		-
 -----------------------------------------------------------------------------------------*/
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

/*----------------------------------------------------------------------------------------
 * @fn			-	SPI_SSOEConfig
 *
 * @brief		-	Configures the SSOE bit in CR2 register, when Software Slave mode is Disabled, ie, NSS is working in hardware mode.
 *
 * @param[in]	-	Address of the peripheral
 * @param[in]	-	ENABLE or DISABLE macros, defined in MCU header file "stm32f407xx.h"
 *
 * @return		-	none
 *
 * @note		-
 -----------------------------------------------------------------------------------------*/
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}
	else
	{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}



/*-------------------------------Some Helper Functions------------------------------------*/
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	// Check the DFF bit in CR1 register
	if ( ( pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF) ) )
	{
		// 16 bit data format
		//1. Load the data in DR register
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		//2. Decrease DataLen twice
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		//3. Increment TX buffer
		(uint16_t*)pSPIHandle->pTxBuffer++;
	}
	else
	{
		// 8 bit data format
		//1. Load the data in DR register
		pSPIHandle->pSPIx->DR = *(pSPIHandle->pTxBuffer);
		//2. Decrease DataLen
		pSPIHandle->TxLen--;
		//3. Increment TX buffer
		pSPIHandle->pTxBuffer++;
	}
	if (! pSPIHandle->TxLen)
	{
		// TxLen is zero.
		// Close the SPI communication and inform TX is over
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallBack(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}

}

static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	// Check the DFF bit in CR1 register
	if ( ( pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF) ) )
	{
		// 16 bit data format
		//1. Load the data from DR register to Rx Buffer
		*((uint16_t*)pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
		//2. Decrease DataLen twice
		pSPIHandle->RxLen--;
		pSPIHandle->RxLen--;
		//3. Increment TX buffer
		(uint16_t*)pSPIHandle->pRxBuffer++;
	}
	else
	{
		// 8 bit data format
		//1. Load the data from DR register to Rx buffer
		*(pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
		//2. Decrease DataLen
		pSPIHandle->RxLen--;
		//3. Increment TX buffer
		pSPIHandle->pRxBuffer++;
	}
	if (! pSPIHandle->RxLen)
	{
		// RxLen is zero.
		// Close the SPI communication and inform RX is over
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallBack(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}
}

static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;
	//1. Clear the OVR flag
	// To clear, read the data register, then read the Status register
	if (pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	//Since temp variable is not used, it will give warning.
	//Therefore use this:
	(void)temp;

	//2. Inform the Application
	SPI_ApplicationEventCallBack(pSPIHandle, SPI_EVENT_OVR_ERR);

}

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);

	// Reset the buffers
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);

	// Reset the buffers
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}

void ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	//Since temp variable is not used, it will give warning.
	//Therefore use this:
	(void)temp;
}



__attribute__((weak)) void SPI_ApplicationEventCallBack(SPI_Handle_t *pSPIHandle, uint8_t AppEvent)
{
	// this is a weak implementation, the application can override this function
}
