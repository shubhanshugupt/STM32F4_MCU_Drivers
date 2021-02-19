/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: 01-Feb-2021
 *      Author: shubh
 */

#include "stm32f407xx_gpio_driver.h"



/* This function returns a code (0-7) for a given GPIOx peripheral */

uint8_t CONV_GPIOx_TO_PORTCODE(GPIO_RegDef_t *pGPIOx)
{
	uint8_t portcode = 0;
	if (pGPIOx == GPIOA)
	{
		portcode = 0;
	}else if (pGPIOx == GPIOB)
	{
		portcode = 1;
	}else if (pGPIOx == GPIOC)
	{
		portcode = 2;
	}else if (pGPIOx == GPIOD)
	{
		portcode = 3;
	}else if (pGPIOx == GPIOE)
	{
		portcode = 4;
	}else if (pGPIOx == GPIOF)
	{
		portcode = 5;
	}else if (pGPIOx == GPIOG)
	{
		portcode = 6;
	}else if (pGPIOx == GPIOH)
	{
		portcode = 7;
	}
	return portcode;
}

//	Enabling the GPIO / Peripheral Clock Setup

/*----------------------------------------------------------------------------------------
 * @fn			-	GPIO_PClkCtrl
 *
 * @brief		-	This function Enables or Disables the Peripheral clock for a GPIO port
 *
 * @param[in]	-	base address of the GPIO peripheral
 * @param[in]	-	ENABLE or DISABLE macros, defined in MCU header file "stm32f407xx.h"
 *
 * @return		-	none
 *
 * @note		-	none
 -----------------------------------------------------------------------------------------*/
void GPIO_PClkCtrl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_CLK_EN();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_CLK_EN();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_CLK_EN();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_CLK_EN();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_CLK_EN();
		}
		else if (pGPIOx == GPIOF)
		{
			GPIOF_CLK_EN();
		}
		else if (pGPIOx == GPIOG)
		{
			GPIOG_CLK_EN();
		}
		else if (pGPIOx == GPIOH)
		{
			GPIOH_CLK_EN();
		}
		else if (pGPIOx == GPIOI)
		{
			GPIOI_CLK_EN();
		}
	}
	else
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_CLK_DI();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_CLK_DI();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_CLK_DI();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_CLK_DI();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_CLK_DI();
		}
		else if (pGPIOx == GPIOF)
		{
			GPIOF_CLK_DI();
		}
		else if (pGPIOx == GPIOG)
		{
			GPIOG_CLK_DI();
		}
		else if (pGPIOx == GPIOH)
		{
			GPIOH_CLK_DI();
		}
		else if (pGPIOx == GPIOI)
		{
			GPIOI_CLK_DI();
		}
	}

}

/*----------------------------------------------------------------------------------------
 * @fn			-	GPIO_Init
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
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	//0. Enable the Peripheral clock							//Updated on 19-Feb-2021
	GPIO_PClkCtrl(pGPIOHandle->pGPIOx, ENABLE);

	uint32_t	temp = 0;
	//1. Configure the mode of GPIO pin
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		//If the given Pin mode <= 3, it is non-interrupt mode, else interrupt mode
		//Left shifting 2 bit MODE value for specific PIN number
		temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2*(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)) );

		// Clear the register values for specific pin
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
		// set the MODE register for the specific pin
		pGPIOHandle->pGPIOx->MODER |= temp;
	}
	else
	{
		//The interrupt mode
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			//1. Configure FTSR (Falling Trigger Selection Register)
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Also reset the RTSR
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			//1. Configure RTSR (Rising Trigger Selection Register)
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Also reset the FTSR
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			//1. Configure FTSR and RTSR both
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}

		//2. Configure the PORT selection in SYSCFG_EXTICR
		uint8_t temp1, temp2;
		temp1 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)/4;			//Finds the EXTI register to be configured, each register has 4 EXTIx
		temp2 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)%4;			//Finds the position of EXTIx in the EXTI register

		SYSCFG_CLK_EN();

		uint8_t portcode = CONV_GPIOx_TO_PORTCODE(pGPIOHandle->pGPIOx);

		SYSCFG->EXTICR[temp1] &= ~(0xF << 4*temp2);
		SYSCFG->EXTICR[temp1] |= (portcode << 4*temp2);

		//3. Enable the EXTI interrupt delivery using IMR (interrupt management register)
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);


	}
	temp = 0;

	//2. Configure the speed of the GPIO pin
	//Left shifting 2 bit SPEED value for specific PIN number
	temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );

	// Clear the register values for specific pin
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	// set the MODE register for the specific pin
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	temp = 0;

	//3. Configure the Pull-up Pull-down setting
	temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );

	// Clear the register values for specific pin
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	// set the MODE register for the specific pin
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;

	//4. Configure the Output type
	temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinOpType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );

	// Clear the register values for specific pin
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	// set the MODE register for the specific pin
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp = 0;

	//5. Configure the Alternate functionality
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		uint8_t temp1;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFuncMode << (4 * temp1) );
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber < 8)
			{
				pGPIOHandle->pGPIOx->AFRL &= ~(0xF << (4 * temp1) );
				pGPIOHandle->pGPIOx->AFRL |= temp;
			}
			else
			{
				pGPIOHandle->pGPIOx->AFRH &= ~(0xF << (4 * temp1) );
				pGPIOHandle->pGPIOx->AFRH |= temp;
			}
	}
	temp = 0;

}

/*----------------------------------------------------------------------------------------
 * @fn			-	GPIO_DeInit
 *
 * @brief		-	It resets the GPIO peripheral registers
 *
 * @param[in]	-	Address of the peripheral
 * @param[in]	-	none
 *
 * @return		-	none
 *
 * @note		-	We have a special register in RCC peripheral, called AHB1 peripheral reset register
 * 					This will reset all the peripheral registers
 -----------------------------------------------------------------------------------------*/
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if (pGPIOx == GPIOA)
	{
		GPIOA_RESET();
	}
	else if (pGPIOx == GPIOB)
	{
		GPIOB_RESET();
	}
	else if (pGPIOx == GPIOC)
	{
		GPIOC_RESET();
	}
	else if (pGPIOx == GPIOD)
	{
		GPIOD_RESET();
	}
	else if (pGPIOx == GPIOE)
	{
		GPIOE_RESET();
	}
	else if (pGPIOx == GPIOF)
	{
		GPIOF_RESET();
	}
	else if (pGPIOx == GPIOG)
	{
		GPIOG_RESET();
	}
	else if (pGPIOx == GPIOH)
	{
		GPIOH_RESET();
	}
	else if (pGPIOx == GPIOI)
	{
		GPIOI_RESET();
	}
}

//GPIO Read and Write
/*----------------------------------------------------------------------------------------
 * @fn			-	GPIO_ReadfromIPPin
 *
 * @brief		-	This function will read the value from a GPIO pin, 1 or 0
 *
 * @param[in]	-	Address of the peripheral
 * @param[in]	-	Pin number which is to be read
 *
 * @return		-	returns the high or low state of the pin, 1 or 0
 *
 * @note		-	none
 -----------------------------------------------------------------------------------------*/
uint8_t GPIO_ReadfromIPPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t) ( ( pGPIOx->IDR >> PinNumber ) & 0x00000001 );
	return value;
}

/*----------------------------------------------------------------------------------------
 * @fn			-	GPIO_ReadfromIPPort
 *
 * @brief		-	It reads from the Input port of a GPIO peripheral, ie, 16 pins
 *
 * @param[in]	-	Address of the peripheral
 * @param[in]	-	none
 *
 * @return		-	16 bit data from 16 pins
 *
 * @note		-	none
 -----------------------------------------------------------------------------------------*/
uint16_t GPIO_ReadfromIPPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t) ( pGPIOx->IDR ) ;
	return value;
}

/*----------------------------------------------------------------------------------------
 * @fn			-	GPIO_WriteToOPPin
 *
 * @brief		-	It sets/resets specific pin of a GPIOx peripheral
 *
 * @param[in]	-	Address of GPIO peripheral
 * @param[in]	-	Pin number to be read
 * @param[in]	-	SET or RESET macros (defined in MCU header file "stm32f407xx.h"
 *
 * @return		-	none
 *
 * @note		-	none
 -----------------------------------------------------------------------------------------*/
void GPIO_WriteToOPPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value)
{
	if (value == SET)						// SET =	1
	{
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else if (value == RESET)				// RESET =	0
	{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

/*----------------------------------------------------------------------------------------
 * @fn			-	GPIO_WriteToOPPort
 *
 * @brief		-	It writes to the complete port of a GPIO peripheral
 *
 * @param[in]	-	Address of the GPIO
 * @param[in]	-	16bit value to be written
 *
 * @return		-	none
 *
 * @note		-	none
 -----------------------------------------------------------------------------------------*/
void GPIO_WriteToOPPort(GPIO_RegDef_t *pGPIOx, uint16_t value)
{
	pGPIOx->ODR = (value);
}

/*----------------------------------------------------------------------------------------
 * @fn			-	GPIO_ToggleOPPin
 *
 * @brief		-	THis function toggles the state of a pin (HIGH <--> LOW)
 *
 * @param[in]	-	Address of GPIO peripheral
 * @param[in]	-	Pin number to be toggled
 *
 * @return		-	none
 *
 * @note		-	none
 -----------------------------------------------------------------------------------------*/
void GPIO_ToggleOPPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR = pGPIOx->ODR ^ (0x1 << PinNumber);			//Using bitwise EXOR (^) to toggle a bit value
}

//GPIO Interrupt Handling
/*----------------------------------------------------------------------------------------
 * @fn			-	GPIO_IRQConfig
 *
 * @brief		-	It configures an Interrupt, ie enable or disable
 *
 * @param[in]	-	IRQ number of the interrupt
 * @param[in]	-	ENABLE or DISABLE macros
 *
 * @return		-	none
 *
 * @note		-	none
 -----------------------------------------------------------------------------------------*/
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi)
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


//GPIO Interrupt Handling
/*----------------------------------------------------------------------------------------
 * @fn			-	GPIO_IRQPriorityConfig
 *
 * @brief		-	It configures the priority of an Interrupt
 *
 * @param[in]	-	IRQ number of the interrupt
 * @param[in]	-	Priority of the interrupt
 *
 * @return		-	none
 *
 * @note		-	none
 -----------------------------------------------------------------------------------------*/
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t RegNo	=	IRQNumber/4;
	uint8_t section	=	IRQNumber%4;
	uint8_t shift	=	( 8*section ) + ( 8 - PRIORITY_BITS_IMPLEMENTED);

	*(NVIC_IPR_BASEADDR + 4*RegNo) = ( IRQPriority << shift );

}


/*----------------------------------------------------------------------------------------
 * @fn			-	GPIO_IRQHandling
 *
 * @brief		-	This will read the pin where the interrupt was triggered
 *
 * @param[in]	-	Pin number
 *
 * @return		-	none
 *
 * @note		-	none
 -----------------------------------------------------------------------------------------*/
void GPIO_IRQHandling(uint8_t PinNumber)
{
	//clear the EXTI PR register corresponding to the pin number
	if (EXTI->PR & (1 << PinNumber))
	{
		EXTI->PR |= (1 << PinNumber);
	}

}
