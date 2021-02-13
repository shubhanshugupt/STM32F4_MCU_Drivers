/*
 * 003led_freq.c
 *
 * I don't know why I wrote this program
 *
 *  Created on: 12-Feb-2021
 *      Author: shubh
 */


#include <stdint.h>
#include "stm32f407xx.h"


int main(void)
{
	GPIO_Handle_t GPIOLed;

	GPIOLed.pGPIOx = GPIOD;
	GPIOLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GPIOLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIOLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	GPIOLed.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_PP;
	GPIOLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PClkCtrl(GPIOD, ENABLE);

	GPIO_Init(&GPIOLed);

    while(1)
    {
    	GPIO_ToggleOPPin(GPIOD, GPIO_PIN_12);

    }
}
