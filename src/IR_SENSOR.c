/*
 * IR_SENSOR.c
 *
 *  Created on: Oct 14, 2025
 *      Author: gnanaprakash.g
 */


#include "stm32f411xx.h"
#include "stm32f4xx_gpio_driver.h"


void delay()
{
	for(uint32_t i=0;i<500000/2;i++);
}



int main()
{

	GPIO_Handle_t GpioLed,GpioIR;

	GPIO_PeriClockControl(GPIOA,ENABLE);
	GPIO_PeriClockControl(GPIOC,ENABLE);

	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPTyper = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;



	GPIO_Init(&GpioLed);

	GpioIR.pGPIOx = GPIOC;
	GpioIR.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
	GpioIR.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioIR.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioIR.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;



	GPIO_Init(&GpioIR);

	while(1)
	{
		if(((GPIOC->IDR) & 1) ==  0)
		{
			GPIOA->ODR |= (1<<5);
		}else
		{
			GPIOA->ODR &= ~(1<<5);
		}
		delay();
	}
}
