/*
 * stm32f4xx_gpio_driver.c
 *
 *  Created on: Sep 23, 2025
 *      Author: gnanaprakash.g
 */

#include "stm32f4xx_gpio_driver.h"

//Peripheral Clock Setup

/*********************************************
 * @fn                - GPIO_PeriClockCOntrol
 * brief              - This function enables or disables peripheral clock for given GPIO Port
 * @param[in]         - Base Address of GPIO Port
 * @param[in]         - Enable or Disable
 * @param[in]
 *
 *
 * @return            - 0 or 1
 *
 * @note              - None
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx,uint8_t EnOrDi)
{
	  if(EnOrDi == ENABLE)
	  {
		  if(pGPIOx == GPIOA)
		  {
			  GPIOA_PCLK_EN();
		  }
		  else if(pGPIOx == GPIOB)
		  {
			  GPIOB_PCLK_EN();
		  }
		  else if(pGPIOx == GPIOC)
		  {
		  	  GPIOC_PCLK_EN();
		  }
		  else if(pGPIOx == GPIOD)
		  {
		  	  GPIOD_PCLK_EN();
		  }
		  else if(pGPIOx == GPIOE)
		  {
		  	  GPIOE_PCLK_EN();
		  }
		  else if(pGPIOx == GPIOH)
		  {
		  	  GPIOH_PCLK_EN();
		  }
	  }
	  else
	  {
		  if(pGPIOx == GPIOA)
		  {
			  GPIOA_PCLK_DI();
		  }
		  else if(pGPIOx == GPIOB)
		  {
		      GPIOB_PCLK_DI();
		  }
		  else if(pGPIOx == GPIOC)
		  {
		  	  GPIOC_PCLK_DI();
		  }
		  else if(pGPIOx == GPIOD)
		  {
		  	  GPIOD_PCLK_DI();
		  }
		  else if(pGPIOx == GPIOE)
		  {
		  	  GPIOE_PCLK_DI();
		  }
		  else if(pGPIOx == GPIOH)
		  {
		  	  GPIOH_PCLK_DI();
		  }
	  }
}


//Init and DeInit
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
        uint32_t temp = 0;

        //enable the peripheral clock
        GPIO_PeriClockControl(pGPIOHandle->pGPIOx,ENABLE);

        //1.Configure the mode of GPIO Pin
        if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
        {
            //non interrupt mode
        	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode<<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
        	pGPIOHandle->pGPIOx->MODER &= ~(0x3<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        	pGPIOHandle->pGPIOx->MODER |= temp;
        }
        else
        {
        	//interrupt mode
        	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_IT_FT)
        	{
        		//1.Configure the FTSR
        		EXTI->FTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        		//2.Clear the corresponding RTSR bit
        		EXTI->RTSR &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

        	}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_IT_RT)
        	{
        		//1.Configure the RTSR
        		EXTI->RTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        		//2.Clear the corresponding FTSR bit
        		EXTI->FTSR &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

        	}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_IT_RFT)
        	{
        		//1.Configure both the FTSR & RTSR
        		EXTI->FTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        		EXTI->RTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        	}

                //2.Configure the GPIO Port selection in SYSCF_EXTICR
        	    uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
        	    uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
        	    uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
        	    SYSCFG_PCLK_EN();
        	    SYSCFG->EXTICR[temp1] &= ~(0xF << (temp2 * 4));  // clear old bits
        	    SYSCFG->EXTICR[temp1] |=  (portcode << (temp2 * 4));  // set new port

        	    //Enable  the EXTI interrupt  Delivery using IMR
        	    EXTI->IMR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        }









        temp = 0;

        //2.Configure the Speed
        temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed<<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
        pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        pGPIOHandle->pGPIOx->OSPEEDR |= temp;

        temp = 0;

        //3.Configure the Pull up/Pull Down
        temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl<<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
        pGPIOHandle->pGPIOx->PUPDR &= ~(0x3<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        pGPIOHandle->pGPIOx->PUPDR |= temp;

        temp =0;

        //4.Configure the output type
        temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPTyper<<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
        pGPIOHandle->pGPIOx->OTYPER &= ~(0x1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        pGPIOHandle->pGPIOx->OTYPER |= temp;

        //5.Configure the alternate function
       if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
       {
           //configure the alt function registers
    	   uint8_t temp1,temp2;
    	   temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
    	   temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
    	   pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF<<(4*temp2));
    	   pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode<<(4*temp2));

       }
}


void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	          if(pGPIOx == GPIOA)
			  {
				  GPIOA_REG_RESET();
			  }
			  else if(pGPIOx == GPIOB)
			  {
				  GPIOB_REG_RESET();
			  }
			  else if(pGPIOx == GPIOC)
			  {
			  	  GPIOC_REG_RESET();
			  }
			  else if(pGPIOx == GPIOD)
			  {
			  	  GPIOD_REG_RESET();
			  }
			  else if(pGPIOx == GPIOE)
			  {
			  	  GPIOE_REG_RESET();
			  }
			  else if(pGPIOx == GPIOH)
			  {
			  	  GPIOH_REG_RESET();
			  }
}


//Data Read and Write
uint8_t  GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber)
{
         uint8_t value;
         value = (uint8_t)(pGPIOx->IDR >> PinNumber) & 0x00000001;
         return value;
}


uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
         uint16_t value;
         value = (uint16_t)pGPIOx->IDR;
         return value;
}


void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber,uint8_t Value)
{
         if(Value == GPIO_PIN_SET)
         {
        	 pGPIOx->ODR |= (1<< PinNumber);
         }
         else
         {
        	 pGPIOx->ODR &= ~(1<< PinNumber);
         }
}


void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint8_t Value)
{
	     pGPIOx->ODR = Value;
}


void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber)
{
         pGPIOx->ODR ^= (1<<PinNumber);
}


//IRQ Configuration and ISR Handling
void GPIO_IRQInterruptConfig(uint8_t IRQNumber , uint8_t EnOrDi)
{
         if(EnOrDi == ENABLE)
         {
        	 if(IRQNumber <= 31)
        	 {
        		 //Program ISER0 register
        		 *NVIC_ISER0 |= (1 << IRQNumber);
        	 }
        	 else if(IRQNumber > 31 && IRQNumber <64)
        	 {
        		 //Program ISER1 register
        		 *NVIC_ISER0 |= (1 << (IRQNumber % 32));
        	 }
        	 else if(IRQNumber > 64 && IRQNumber <96)
        	 {
        		 //Program ISER2 register
        		 *NVIC_ISER0 |= (1 << (IRQNumber % 64));
        	 }
         }
         else
         {
        	 if(IRQNumber <= 31)
        	 {
        	     //Program ICER0 register
        		 *NVIC_ICER0 |= (1 << IRQNumber);
        	 }
        	 else if(IRQNumber > 31 && IRQNumber <64)
        	 {
        	     //Program ICER1 register
        		 *NVIC_ICER1 |= (1 << IRQNumber);
        	 }
        	 else if(IRQNumber > 64 && IRQNumber <96)
        	 {
        	     //Program ICER2 register
        		 *NVIC_ICER2 |= (1 << IRQNumber);
        	 }
         }
}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	//1.first lets find out ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_Section = IRQNumber % 4;

	uint8_t shift_Amount = (8 * iprx_Section ) + (8 - PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_Amount);

}


void GPIO_IRQHandling(uint8_t PinNumber)
{
     //clear the EXTI PENDING REGISTER corresponding to the pin number
	if(EXTI->PR & (1<<PinNumber))
	{
		//clear the register
		EXTI->PR |= (1<<PinNumber);
	}
}






















