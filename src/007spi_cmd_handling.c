/*
 * 007spi_cmd_handling.c
 *
 *  Created on: Oct 3, 2025
 *      Author: gnanaprakash.g
 */


#include<string.h>
#include "stm32f411xx.h"
#include "stm32f4xx_gpio_driver.h"


//command modes
#define COMMAND_LED_CTRL    	0x50
#define COMMAND_SENSOR_READ   	0x50
#define COMMAND_LED_READ    	0x50
#define COMMAND_PRINT        	0x50
#define COMMAND_ID_READ     	0x50

#define LED_ON        1
#define LED_OFF       0

//arduino analog pins
#define ANALOG_PIN0     0
#define ANALOG_PIN1     1
#define ANALOG_PIN2     2
#define ANALOG_PIN3     3
#define ANALOG_PIN4     4

//arduino led
#define LED_PIN         9

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
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
    GPIO_Init(&SPIPins);

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

uint8_t SPI_VerifyResponse(uint8_t ackByte)
{
	if(ackByte == 0xF5){
		//ack
		return 1;
	}else
	{
		//nack
		return 0;
	}
}

int main(void)
{

	uint8_t  dummy_byte = 0xff;

	uint8_t dummy_read;

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

        //1.CMD_LED_CTRL  <pin no(1)>  <value(1)>

       uint8_t commandcode = COMMAND_LED_CTRL;
       uint8_t ackByte;
       uint8_t args[2];

       //send command
       SPI_SendData(SPI1,&commandcode,1);

       //do dummy read to clear off the RXNE
       SPI_ReceiveData(SPI1,&dummy_read,1);



       //send some dummy bits (1 byte) to fetch response from the slave
       SPI_SendData(SPI1,&dummy_byte,1);

       //read the ack byte received
       SPI_ReceiveData(SPI1,&ackByte,1);

       if(SPI_VerifyResponse(ackByte))
       {
    	   //send arguments
    	   args[0] = LED_PIN;
    	   args[1] = LED_ON;
    	   SPI_SendData(SPI1,args,2);
       }

       //2.CMD_SENSOR_READ
       //wait till button is pressed
       	while(! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_10) );

       	delay();

       	commandcode = COMMAND_SENSOR_READ;

        SPI_SendData(SPI1,&commandcode,1);

        //do dummy read to clear off the RXNE
        SPI_ReceiveData(SPI1,&dummy_read,1);

        //send some dummy bits (1 byte) to fetch response from the slave
        SPI_SendData(SPI1,&dummy_byte,1);

        //read the ack byte received
        SPI_ReceiveData(SPI1,&ackByte,1);

        if(SPI_VerifyResponse(ackByte))
        {
         //send arguments
         args[0] = ANALOG_PIN0;

         SPI_SendData(SPI1,args,1);


         //do dummy read to clear off the RXNE
         SPI_ReceiveData(SPI1,&dummy_read,1);

         //insert some delay so that slave can ready with the data
         delay();

          //send some dummy bits (1 byte) to fetch response from the slave
          SPI_SendData(SPI1,&dummy_byte,1);

         uint8_t analog_read;
         SPI_ReceiveData(SPI1,&analog_read,1);

        }

	    //lets confirm SPI is not busy
	    while(SPI_GetFlagStatus(SPI1, SPI_BSY_FLAG));


	    //Disable the SPI1 peripheral
	    SPI_PeripheralControl(SPI1,DISABLE);

	}


	return 0;
}
