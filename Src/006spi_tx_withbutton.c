/*
 * 006spi_tx_withbutton.c
 *
 *  Created on: 02-Mar-2021
 *      Author: shubh
 */

#include <string.h>
#include "stm32f407xx.h"

/*
 * -----PIN CONFIGURATION-----
 * Alternate Functionality: AF5
 * SPI_NSS	->	PB12
 * SPI_SCLK	->	PB13
 * SPI_MISO	->	PB14
 * SPI_MOSI	->	PB15
 */

/*------Function to initialize GPIO PIN as SPI PIN-------*/
void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPI2Pins;

	SPI2Pins.pGPIOx = GPIOB;
	SPI2Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPI2Pins.GPIO_PinConfig.GPIO_PinAltFuncMode = AF5;
	SPI2Pins.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_PP;
	SPI2Pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPI2Pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	// Enable SCLK
	SPI2Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GPIO_Init(&SPI2Pins);

	// Enable NSS
	SPI2Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GPIO_Init(&SPI2Pins);

	// Enable MOSI
	SPI2Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;
	GPIO_Init(&SPI2Pins);

	// Enable MISO
	SPI2Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15;
	GPIO_Init(&SPI2Pins);
}

void GPIOButton(void)
{
	//Create and Initialize the Handle structure for GPIO Button
	GPIO_Handle_t GPIOButton;
	GPIOButton.pGPIOx = GPIOA;
	GPIOButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
	GPIOButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GPIOButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GPIOButton);
}


/*------Function to initialize SPI2 Peripheral--------*/
void SPI2Init(void)
{
	SPI_Handle_t SPI2Handle;

	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPI_PinConfig.SPI_DeviceMode	= SPI_MODE_MASTER;
	SPI2Handle.SPI_PinConfig.SPI_BusConfig	= SPI_BUS_FULL_DUP;
	SPI2Handle.SPI_PinConfig.SPI_DFF		= SPI_DFF_8BIT;
	SPI2Handle.SPI_PinConfig.SPI_CPHA		= SPI_CPHA_FIRST_EDGE;
	SPI2Handle.SPI_PinConfig.SPI_CPOL		= SPI_CPOL_LOW;
	SPI2Handle.SPI_PinConfig.SPI_SSM		= SPI_SSM_EN;
	SPI2Handle.SPI_PinConfig.SPI_SCLKSpeed	= SPI_SCLK_BY_2;

	SPI_Init(&SPI2Handle);
}


int main(void)
{
	char Data[] = "Namaste";
	// Initialize GPIO PIN to behave as SPI PIN
	SPI2_GPIOInits();

	// Initialize GPIO PIN for button
	GPIOButton();

	// Initialize SPI peripheral parameters
	SPI2Init();

	// Since SSM is enabled, SSI bit needs to be kept high.
	SPI_SSIConfig(SPI2, ENABLE);

	// Enable the SPI2 peripheral
	SPI_PCtrl(SPI2, ENABLE);

	// Wait for the button to be pressed
	while( GPIO_ReadfromIPPin(GPIOA, GPIO_PIN_0) == RESET );

	SPI_DataSend(SPI2, (uint8_t*)Data, strlen(Data));

	//Wait till SPI is busy
	while( FlagStatus(SPI2, SPI_BUSY_FLAG) );

	//Disable the SPI2 peripheral
	SPI_PCtrl(SPI2, DISABLE);

	while(1);

	return 0;
}
