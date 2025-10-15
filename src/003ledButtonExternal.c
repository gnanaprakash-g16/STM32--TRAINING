/*
 * 003ledButtonExternal.c
 *
 *  Created on: Sep 24, 2025
 *      Author: gnanaprakash.g
 */


#include "stm32f411xx.h"
#include "stm32f4xx_gpio_driver.h"


void delay()
{
	for(uint32_t i=0;i<500000/2;i++);
}
int main(void)
{
	GPIO_Handle_t GpioLed,GpioBtn;

	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_10;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPTyper = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA,ENABLE);

	GPIO_Init(&GpioLed);

	GpioBtn.pGPIOx = GPIOC;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_10;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinOPTyper = GPIO_OP_TYPE_PP;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOC,ENABLE);

	GPIO_Init(&GpioBtn);

	while(1)
	{
		if(!GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_NO_10))
			//why inverter is used here?
			//Because,in nucleo push button state will be go high,when button is not pressed,
			//so it toggles when button is not pressed,to avoid that inverter is used
		{
			delay();
			GPIO_ToggleOutputPin(GPIOA,GPIO_PIN_NO_10);

		}
	}





}
