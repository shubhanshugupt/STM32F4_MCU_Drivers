/*
 * 002led_button.c
 *
 * This program toggles OnBoard LED connected to GPIOD PIN 12,
 * using an OnBoard PushButton connected to GPIOA PIN 0.
 * This program is created to understand how to read and write on a GPIO PIN
 *
 *  Created on: 10-Feb-2021
 *      Author: shubh
 */


#include <stdint.h>
#include "stm32f407xx.h"

#define HIGH 	1
#define LOW		0

void delay(void)
{
	for(uint32_t i=0; i<=100000; i++);
}

int main(void)
{
	uint8_t old	=	LOW;
	uint8_t new	=	LOW;

	//Create and initialize the Handle structure for GPIO LED
	GPIO_Handle_t GPIOLed;
	GPIOLed.pGPIOx = GPIOD;
	GPIOLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GPIOLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIOLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MEDIUM;
	GPIOLed.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_PP;
	GPIOLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PClkCtrl(GPIOD, ENABLE);

	GPIO_Init(&GPIOLed);

	//Create and Initialize the Handle structure for GPIO Button
	GPIO_Handle_t GPIOButton;
	GPIOButton.pGPIOx = GPIOA;
	GPIOButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
	GPIOButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GPIOButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PClkCtrl(GPIOA, ENABLE);

	GPIO_Init(&GPIOButton);

	/*
    while(1)
    {
    	if (GPIO_ReadfromIPPin(GPIOA, GPIO_PIN_0) == HIGH)
    	{
    		delay();
    		GPIO_ToggleOPPin(GPIOD, GPIO_PIN_12);
    	}
    }
    */

    while(1)
    {
    	new = GPIO_ReadfromIPPin(GPIOA, GPIO_PIN_0);
    	if ((new == HIGH) && (old == LOW))
    	{
    		GPIO_ToggleOPPin(GPIOD, GPIO_PIN_12);
    	}
    	old = new;
    }
}


