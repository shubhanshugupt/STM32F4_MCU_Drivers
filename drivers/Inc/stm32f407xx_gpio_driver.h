/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: 01-Feb-2021
 *      Author: shubh
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"

typedef struct
{
	uint8_t GPIO_PinNumber;			// Possible values from @GPIO_PIN_NUMBER
	uint8_t GPIO_PinMode;			// Possible values from @GPIO_PIN_MODES
	uint8_t GPIO_PinSpeed;			// Possible values from @GPIO_PIN_SPEEDS
	uint8_t GPIO_PinPuPdControl;	// Possible values from @GPIO_PIN_PUPD_CTRL
	uint8_t GPIO_PinOpType;			// Possible values from @GPIO_PIN_OP_TYPE
	uint8_t GPIO_PinAltFuncMode;
}GPIO_PinConfig_t;

typedef struct
{
	GPIO_RegDef_t *pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;
}GPIO_Handle_t;


/*------------------GPIO PIN Numbers--------------------------------------------*/
/* 	@GPIO_PIN_NUMBER	*/
#define GPIO_PIN_0			0
#define GPIO_PIN_1			1
#define GPIO_PIN_2			2
#define GPIO_PIN_3			3
#define GPIO_PIN_4			4
#define GPIO_PIN_5			5
#define GPIO_PIN_6			6
#define GPIO_PIN_7			7
#define GPIO_PIN_8			8
#define GPIO_PIN_9			9
#define GPIO_PIN_10			10
#define GPIO_PIN_11			11
#define GPIO_PIN_12			12
#define GPIO_PIN_13			13
#define GPIO_PIN_14			14
#define GPIO_PIN_15			15

/*------------------GPIO PIN possible modes-------------------------------------*/
/*	@GPIO_PIN_MODES	 */

#define GPIO_MODE_IN		0
#define	GPIO_MODE_OUT		1
#define GPIO_MODE_ALTFN		2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_IT_FT		4		//InpuT Falling-edge Trigger (ITFT)
#define GPIO_MODE_IT_RT		5		//InpuT Rising-edge Trigger (ITRT)
#define GPIO_MODE_IT_RFT	6		//InpuT Rising-Falling-edge Trigger (ITRFT)

/*--------------GPIO PIN possible output type------------------------------------*/
/*	@GPIO_PIN_OP_TYPE	*/

#define GPIO_OP_TYPE_PP		0		//Output type push pull
#define GPIO_OP_TYPE_OD		1		//Output type Open Drain

/*--------------GPIO PIN possible  output speed---------------------------------*/
/*	@GPIO_PIN_SPEEDS	*/

#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define	GPIO_SPEED_HIGH		2
#define GPIO_SPEED_VERYHIGH	3

/*-------------GPIO PIN pull-up pull-down configuration---------------------------*/
/*	@GPIO_PIN_PUPD_CTRL	*/
#define	GPIO_NO_PUPD		0		//No pull-up pull-down
#define GPIO_PIN_PU			1		//Pull up
#define GPIO_PIN_PD			2		//Pull down




/*-----------------------------------------------------------------------------
  					APIs supported by this Driver
 ----------------------------------------------------------------------------*/

//	Enabling the GPIO / Peripheral Clock Setup
void GPIO_PClkCtrl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

//GPIO Read and Write
uint8_t GPIO_ReadfromIPPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadfromIPPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOPPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value);
void GPIO_WriteToOPPort(GPIO_RegDef_t *pGPIOx, uint16_t value);
void GPIO_ToggleOPPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

//GPIO Interrupt Handling
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);


//Convert GPIO peripheral to a portcode number
uint8_t CONV_GPIOx_TO_PORTCODE(GPIO_RegDef_t *pGPIOx);


#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
