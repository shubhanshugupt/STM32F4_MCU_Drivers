/*
 * 004button_interrupt2.c
 *
 *  Created on: 03-Mar-2021
 *      Author: shubh
 */


#include <stdint.h>
#include <string.h>
#include "stm32f407xx.h"

#define HIGH 	1
#define LOW		0


int main(void)
{
	//Create and initialize the Handle structure for GPIO LED
	GPIO_Handle_t GPIOLed;
	memset(&GPIOLed, 0, sizeof(GPIOLed));

	GPIOLed.pGPIOx = GPIOD;
	GPIOLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GPIOLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIOLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MEDIUM;
	GPIOLed.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_PP;
	GPIOLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	//GPIO_PClkCtrl(GPIOD, ENABLE);

	GPIO_Init(&GPIOLed);

	//Create and Initialize the Handle structure for GPIO Button
	GPIO_Handle_t GPIOButton;
	memset(&GPIOButton, 0, sizeof(GPIOButton));

	GPIOButton.pGPIOx = GPIOD;
	GPIOButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
	GPIOButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GPIOButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GPIOButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	//GPIO_PClkCtrl(GPIOA, ENABLE);

	GPIO_Init(&GPIOButton);


	//IRQ Configurations on th Button PIN
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRI15);
	GPIO_IRQConfig(IRQ_NO_EXTI9_5, ENABLE);		//EXTI0 since PIN0 is used for button

	while(1);

	return 0;
}

// Implementing the ISR

void EXTI9_5_IRQHandler(void)
{
	GPIO_IRQHandling(GPIO_PIN_5);			//We reset the bit in EXTI PR register, since the interrupt is now triggered
	GPIO_ToggleOPPin(GPIOD, GPIO_PIN_12);

}
