/*
 * 001led_toggle.c
 *
 * This program toggles OnBoard LED connected to GPIOD PIN 12, with a constant delay function
 * This program is created to understand how to write on a GPIO PIN
 *
 *  Created on: 09-Feb-2021
 *      Author: shubh
 */


#include <stdint.h>
#include "stm32f407xx.h"

void delay(void)
{
	for(uint32_t i=0; i<=5000000; i++);
}

int main(void)
{
	GPIO_Handle_t GPIOLed;
	GPIOLed.pGPIOx = GPIOD;
	GPIOLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GPIOLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIOLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MEDIUM;
	GPIOLed.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_PP;
	GPIOLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PClkCtrl(GPIOD, ENABLE);

	GPIO_Init(&GPIOLed);

    while(1)
    {
    	GPIO_ToggleOPPin(GPIOD, GPIO_PIN_12);
    	delay();
    }
}
