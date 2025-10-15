/*
 * 004button_interrupt.c
 *
 *  Created on: Sep 25, 2025
 *      Author: gnanaprakash.g
 */


#include <string.h>
#include "stm32f411xx.h"
#include "stm32f4xx_gpio_driver.h"


void delay()
{
	for(uint32_t i=0;i<500000/2;i++);
}
int main(void)
{
	GPIO_Handle_t GpioLed,GpioBtn;
    memset(&GpioLed,0,sizeof(GpioLed));
    memset(&GpioBtn,0,sizeof(GpioBtn));


	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPTyper = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA,ENABLE);

	GPIO_Init(&GpioLed);

	GpioBtn.pGPIOx = GPIOB;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOB,ENABLE);

	GPIO_Init(&GpioBtn);

	SYSCFG_PCLK_EN();

	//IRQ Configuration
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10, ENABLE);
	//GPIO_IRQPriorityConfig(IRQ_NO_EXTI4, NVIC_IRQ_PRI15);



	while(1){}

}


void EXTI15_10_IRQHandler(void)
{
	delay();
     GPIO_IRQHandling(GPIO_PIN_NO_13);
     GPIO_ToggleOutputPin(GPIOA,GPIO_PIN_NO_5);
}
