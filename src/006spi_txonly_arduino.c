/*
 * 006spi_txonly_arduino.c
 *
 *  Created on: Oct 3, 2025
 *      Author: gnanaprakash.g
 */

#include<string.h>
#include "stm32f411xx.h"
#include "stm32f4xx_gpio_driver.h"
/*
 * PA5 --> SPI1_SCLK
 * PA6 --> SPI1_MISO
 * PA7 --> SPI1_MOSI
 * PA4 --> SPI1_NSS
 *
 * Alternate function - AF05
 *
 */

void delay()
{
	for(uint32_t i=0;i<500000/2;i++);
}


void GPIO_ButtonInit()
{
	GPIO_Handle_t GpioBtn;

	GpioBtn.pGPIOx = GPIOC;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_10;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinOPTyper = GPIO_OP_TYPE_PP;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOC,ENABLE);

	GPIO_Init(&GpioBtn);
}

void SPI1_GPIOInits(void)
{
    GPIO_Handle_t SPIPins;

    SPIPins.pGPIOx = GPIOA;
    SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
    SPIPins.GPIO_PinConfig.GPIO_PinOPTyper = GPIO_OP_TYPE_PP;
    SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

    //SCLK
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
    GPIO_Init(&SPIPins);

    //MISO
    //SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
    //GPIO_Init(&SPIPins);

    //MOSI
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
    GPIO_Init(&SPIPins);

    //NSS
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
    GPIO_Init(&SPIPins);
}


void SPI1_Inits()
{
	SPI_Handle_t SPI1handle;
	SPI1handle.pSPIx = SPI1;
	SPI1handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI1handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI1handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8 ;
	SPI1handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS ;
	SPI1handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI1handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI1handle.SPIConfig.SPI_SSM = SPI_SSM_DI;//Hardware slave management enabled

	SPI_Init(&SPI1handle);

	SPI_PeriClockControl(SPI1,ENABLE);

}


int main(void)
{

	char user_data[] = "Hello world";

	GPIO_ButtonInit();
	//this function is used to initialize the GPIO pins to behave as SPI1 pins
	SPI1_GPIOInits();

	//this function is used to initialize the SPI1 peripheral parameters
	SPI1_Inits();


	SPI_SSOEConfig(SPI2,ENABLE);

	while(1)
	{
        //wait till button is pressed
		while(! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_10) );

		delay();

	    //enable the SPI1 peripheral
        SPI_PeripheralControl(SPI1,ENABLE);

        //first send length information to arduino
        uint8_t dataLen = strlen(user_data);
	    SPI_SendData(SPI1,&dataLen,1);

        //to send data
	    SPI_SendData(SPI1,(uint8_t*)user_data,strlen(user_data));

	    //lets confirm SPI is not busy
	    while(SPI_GetFlagStatus(SPI1, SPI_BSY_FLAG));


	    //Disable the SPI1 peripheral
	    SPI_PeripheralControl(SPI1,DISABLE);

	}


	return 0;
}
