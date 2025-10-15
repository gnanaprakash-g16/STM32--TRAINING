/*
 * 005spi_tx_testing.c
 *
 *  Created on: Sep 30, 2025
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
    //SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
    //GPIO_Init(&SPIPins);
}


void SPI1_Inits()
{
	SPI_Handle_t SPI1handle;
	SPI1handle.pSPIx = SPI1;
	SPI1handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI1handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI1handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2 ;
	SPI1handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS ;
	SPI1handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI1handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI1handle.SPIConfig.SPI_SSM = SPI_SSM_EN;

	SPI_Init(&SPI1handle);

	SPI_PeriClockControl(SPI1,ENABLE);

}


int main(void)
{

	char user_data[] = "Hello world";
	//this function is used to initialize the GPIO pins to behave as SPI1 pins
	SPI1_GPIOInits();

	//this function is used to initialize the SPI1 peripheral parameters
	SPI1_Inits();

	//this makes NSS Signal internally high and avoids MODF error
	SPI_SSIConfig(SPI1,ENABLE);

	//enable the SPI1 peripheral
    SPI_PeripheralControl(SPI1,ENABLE);


	SPI_SendData(SPI1,(uint8_t*)user_data,strlen(user_data));

	//Disable the SPI1 peripheral
	SPI_PeripheralControl(SPI1,DISABLE);


	return 0;
}



//
//#include<string.h>
//#include "stm32f411xx.h"
//#include "stm32f4xx_gpio_driver.h"
///*
// * PB13 -->  SPI2_SCLK
// * PB14 -->  SPI2_MISO
// * PB15 -->  SPI2_MOSI
// * PB12 -->  SPI2_NSS
// *
// * Alternate function - AF05
// *
// */
//
//void SPI2_GPIOInits(void)
//{
//    GPIO_Handle_t SPIPins;
//
//    SPIPins.pGPIOx = GPIOB;
//    SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
//    SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
//    SPIPins.GPIO_PinConfig.GPIO_PinOPTyper = GPIO_OP_TYPE_PP;
//    SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
//    SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
//
//    //SCLK
//    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_10;
//    GPIO_Init(&SPIPins);
//
//    //MISO
//    //SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
//    //GPIO_Init(&SPIPins);
//
//    //MOSI
//    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
//    GPIO_Init(&SPIPins);
//
//    //NSS
//    //SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
//    //GPIO_Init(&SPIPins);
//}
//
//
//void SPI2_Inits()
//{
//	SPI_Handle_t SPI2handle;
//	SPI2handle.pSPIx = SPI2;
//	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
//	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
//	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2 ;
//	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS ;
//	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
//	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
//	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_EN;
//
//	SPI_Init(&SPI2handle);
//
//	SPI_PeriClockControl(SPI2,ENABLE);
//
//}
//
//
//int main(void)
//{
//
//	char user_data[] = "Hello world";
//	//this function is used to initialize the GPIO pins to behave as SPI2 pins
//	SPI2_GPIOInits();
//
//	//this function is used to initialize the SPI1 peripheral parameters
//	SPI2_Inits();
//
//	//this makes NSS Signal internally high and avoids MODF error
//	SPI_SSIConfig(SPI2,ENABLE);
//
//	//enable the SPI2 peripheral
//    SPI_PeripheralControl(SPI2,ENABLE);
//
//
//	SPI_SendData(SPI2,(uint8_t*)user_data,strlen(user_data));
//
//	//Disable the SPI2 peripheral
//	SPI_PeripheralControl(SPI2,DISABLE);
//
//
//	return 0;
//}


//
//#include<string.h>
//#include "stm32f411xx.h"
//#include "stm32f4xx_gpio_driver.h"
///*
// * PB0 -->  SPI5_SCLK
// * PE5 -->  SPI5_MISO
// * PB8 -->  SPI5_MOSI
// * PB1 -->   SPI5_NSS
// *
// * Alternate function - AF05
// *
// */
//
//void SPI5_GPIOInits(void)
//{
//    GPIO_Handle_t SPIPins;
//
//    SPIPins.pGPIOx = GPIOB;
//    SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
//    SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 6;
//    SPIPins.GPIO_PinConfig.GPIO_PinOPTyper = GPIO_OP_TYPE_PP;
//    SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
//    SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
//
//    //SCLK
//    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
//    GPIO_Init(&SPIPins);
//
//    //MISO
//    //SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
//    //GPIO_Init(&SPIPins);
//
//    //MOSI
//    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
//    GPIO_Init(&SPIPins);
//
//    //NSS
//    //SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
//    //GPIO_Init(&SPIPins);
//}
//
//
//void SPI5_Inits()
//{
//	SPI_Handle_t SPI5handle;
//	SPI5handle.pSPIx = SPI5;
//	SPI5handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
//	SPI5handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
//	SPI5handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2 ;
//	SPI5handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS ;
//	SPI5handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
//	SPI5handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
//	SPI5handle.SPIConfig.SPI_SSM = SPI_SSM_EN;
//
//	SPI_Init(&SPI5handle);
//
//	SPI_PeriClockControl(SPI5,ENABLE);
//
//}
//
//
//int main(void)
//{
//
//	char user_data[] = "Hello world";
//	//this function is used to initialize the GPIO pins to behave as SPI2 pins
//	SPI5_GPIOInits();
//
//	//this function is used to initialize the SPI1 peripheral parameters
//	SPI5_Inits();
//
//	//this makes NSS Signal internally high and avoids MODF error
//	SPI_SSIConfig(SPI5,ENABLE);
//
//	//enable the SPI2 peripheral
//    SPI_PeripheralControl(SPI5,ENABLE);
//
//
//	SPI_SendData(SPI5,(uint8_t*)user_data,strlen(user_data));
//
//	//Disable the SPI2 peripheral
//	SPI_PeripheralControl(SPI5,DISABLE);
//
//
//	return 0;
//}

