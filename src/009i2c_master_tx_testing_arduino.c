/*
 * 009i2c_master_tx_testing_arduino.c
 *
 *  Created on: Oct 10, 2025
 *      Author: gnanaprakash.g
 */


#include<stdio.h>
#include<string.h>
#include "stm32f411xx.h"
#include"stm32f4xx_gpio_driver.h"
#include"stm32f411xx_i2c_driver.h"

#define MY_ADDR      0x61
#define SLAVE_ADDR   0x68


void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}


I2C_Handle_t I2CHandle;
/*
 * PB6 - I2C1_SCL
 * PB7 - I2C1_SDA
 */

//some data
uint8_t some_Data[] = "We are testing I2C master Tx\n";


void I2C1_GPIOInits(void)
{
	GPIO_Handle_t I2CPins;

	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinOPTyper = GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;


	//SCL
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&I2CPins);

	//SDA
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&I2CPins);
}

void I2C1_Inits(void)
{
     I2CHandle.pI2Cx = 	I2C1;
     I2CHandle.I2C_Config.I2C_ACKControle = I2C_ACK_ENABLE;
     I2CHandle.I2C_Config.I2C_DeviceAddress = MY_ADDR;
     I2CHandle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
     I2CHandle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

     I2C_Init(&I2CHandle);
}

void GPIO_ButtonInit()
{
	GPIO_Handle_t GpioBtn;

	GpioBtn.pGPIOx = GPIOC;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinOPTyper = GPIO_OP_TYPE_PP;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOC,ENABLE);

	GPIO_Init(&GpioBtn);
}


int main()
{
	GPIO_ButtonInit();

	I2C1_GPIOInits();//I2C Pin Inits

	I2C1_Inits();//I2C peripheral configuration

	//Enable the I2C peripheral
	I2C_PeripheralControl(I2C1,ENABLE);


	while(1)
	{
        //wait till button is pressed
		while(! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_13) );

		delay();



	//send some data to the slave
	I2C_MasterSendData(&I2CHandle,some_Data,strlen((char*)some_Data),SLAVE_ADDR);

	}
}

