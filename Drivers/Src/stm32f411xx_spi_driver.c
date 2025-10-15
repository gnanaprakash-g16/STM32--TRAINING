/*
 * stm32f411xx_spi_driver.c
 *
 *  Created on: Sep 29, 2025
 *      Author: gnanaprakash.g
 */

#include "stm32f411xx.h"
#include "stm32f411xx_spi_driver.h"

static void spi_txe_interrupt_handle();
static void spi_rxne_interrupt_handle();
static void spi_ovr_err_interrupt_handle();



void SPI_PeriClockControl(SPI_RegDef_t *pSPIx,uint8_t EnOrDi)
{
	  if(EnOrDi == ENABLE)
		  {
			  if(pSPIx == SPI1)
			  {
				  SPI1_PCLK_EN();
			  }
			  else if(pSPIx == SPI2)
			  {
				  SPI2_PCLK_EN();
			  }
			  else if(pSPIx == SPI3)
			  {
			  	  SPI3_PCLK_EN();
			  }
			  else if(pSPIx == SPI4)
			  {
			  	  SPI4_PCLK_EN();
			  }
			  else if(pSPIx == SPI5)
			  {
			  	  SPI5_PCLK_EN();
			  }
		  }
		  else
		  {
			  if(pSPIx == SPI1)
			  {
			     SPI1_PCLK_DI();
			  }
			  else if(pSPIx == SPI2)
			  {
			     SPI2_PCLK_DI();
			  }
			  else if(pSPIx == SPI3)
			  {
			 	 SPI3_PCLK_DI();
			  }
			  else if(pSPIx == SPI4)
			  {
			 	 SPI4_PCLK_DI();
			  }
			  else if(pSPIx == SPI5)
			  {
			     SPI5_PCLK_DI();
			  }
		  }
}


void SPI_Init(SPI_Handle_t *pSPIHandle)
{

	//peripheral clock enable
	SPI_PeriClockControl(pSPIHandle->pSPIx,ENABLE);


	//first configure the SPI_CR1 register

	uint32_t temp = 0 ;

	//1.configure the device mode
	temp |= pSPIHandle->SPIConfig.SPI_DeviceMode << 2 ;

	//2.configure the bus config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//bidirectional mode should be cleared
		temp &= ~(1<<15);
	}

	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//bidirectional mode should be set
		temp |= (1<<15);
	}

	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//bidirectional mode should be cleared
		temp &= ~(1<<15);
		//RXONLY bit must be set
		temp |= (1 << 10);
	}

	//3.Configure the spi serial clock speed (baud rate)
	temp |= pSPIHandle->SPIConfig.SPI_SclkSpeed << 3;

	//4.Configure the DFF
	temp |= pSPIHandle->SPIConfig.SPI_DFF << 11 ;

	//5.Configure the CPOL
	temp |= pSPIHandle->SPIConfig.SPI_CPOL << 1 ;

	//6.Configure the CPHA
	temp |= pSPIHandle->SPIConfig.SPI_CPHA << 0 ;

	// Software slave management (SSM = bit 9)
	temp |= (pSPIHandle->SPIConfig.SPI_SSM << 9);


	pSPIHandle->pSPIx->SPI_CR1 = temp;

}

void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer ,uint32_t len)
{
	while(len)
	{
		//1.wait until TXE is set
		 while(SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG) == FLAG_RESET);

		 //2.check the DFF bit in CR1
		 if(pSPIx->SPI_CR1 & (1<<SPI_CR1_DFF))
		 {
			 //16 bit DFF
			 //1.Load the data in to DR
			 pSPIx->SPI_DR = *((uint16_t*)pTxBuffer);
			 len--;
			 len--;
			 (uint16_t*)pTxBuffer++;
		 }else
		 {
			 //8 bit DFF
			 pSPIx->SPI_DR = *pTxBuffer;
			 len--;
			 pTxBuffer++;
		 }

	}
}

void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer ,uint32_t len)
{
	while(len)
	{
		//1.wait until RXNE is set
		 while(SPI_GetFlagStatus(pSPIx,SPI_RXNE_FLAG) == FLAG_RESET);

		 //2.check the DFF bit in CR1
		 if(pSPIx->SPI_CR1 & (1<<SPI_CR1_DFF))
		 {
			 //16 bit DFF
			 //1.Load the data from DR to Rxbuffer address
			 *((uint16_t*)pRxBuffer) =  pSPIx->SPI_DR;
			 len--;
			 len--;
			 (uint16_t*)pRxBuffer++;
		 }else
		 {
			 //8 bit DFF
			 *(pRxBuffer) =  pSPIx->SPI_DR;
			 len--;
			 pRxBuffer++;
		 }

	}
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx , uint32_t FlagName)
{
	if(pSPIx->SPI_SR & FlagName) return FLAG_SET;

	return FLAG_RESET;
}

void SPI_DeInit(SPI_RegDef_t *pSPIx)
{

}


void SPI_PeripheralControl(SPI_RegDef_t *pSPIx,uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->SPI_CR1 |= (1<<SPI_CR1_SPE);
	}else
	{
		pSPIx->SPI_CR1 &= ~(1<<SPI_CR1_SPE);
	}
}


void SPI_SSIConfig(SPI_RegDef_t *pSPIx,uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
		{
			pSPIx->SPI_CR1 |= (1<<SPI_CR1_SSI);
		}else
		{
			pSPIx->SPI_CR1 &= ~(1<<SPI_CR1_SSI);
		}
}


void SPI_SSOEConfig(SPI_RegDef_t *pSPIx,uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
		{
			pSPIx->SPI_CR2 |= (1<<SPI_CR2_SSOE);
		}else
		{
			pSPIx->SPI_CR2 &= ~(1<<SPI_CR2_SSOE);
		}
}


uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pTxBuffer ,uint32_t len)
{
	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX)
	{
		//1.Save the TX buffer address and Len information in some global variables;
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = len;
		//2.Mark the SPI state as busy in transmission so that
		//no other code can take over same SPI Peripheral until transmission is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		//3.Enable the TXEIE control bit to get interrupt whenver TXE Flag is set in SR
		pSPIHandle->pSPIx->SPI_CR2 |= (1 << SPI_CR2_TXEIE);
	}

	return state;
}


uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle , uint8_t *pRxBuffer ,uint32_t len)
{
	uint8_t state = pSPIHandle->RxState;

		if(state != SPI_BUSY_IN_RX)
		{
			//1.Save the TX buffer address and Len information in some global variables;
			pSPIHandle->pRxBuffer = pRxBuffer;
			pSPIHandle->RxLen = len;
			//2.Mark the SPI state as busy in transmission so that
			//no other code can take over same SPI Peripheral until transmission is over
			pSPIHandle->RxState = SPI_BUSY_IN_RX;

			//3.Enable the TXEIE control bit to get interrupt whenver TXE Flag is set in SR
			pSPIHandle->pSPIx->SPI_CR2 |= (1 << SPI_CR2_RXNEIE);
		}
		return state;
}

//IRQ Configuration and ISR Handling
void SPI_IRQInterruptConfig(uint8_t IRQNumber , uint8_t EnOrDi)
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

void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	//1.first lets find out ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_Section = IRQNumber % 4;

	uint8_t shift_Amount = (8 * iprx_Section ) + (8 - PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_Amount);

}



void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp1,temp2;
	//lets check for TXE
	temp1 = pSPIHandle->pSPIx->SPI_SR & (1<<SPI_SR_TXE);
	temp2 = pSPIHandle->pSPIx->SPI_CR2 & (1<<SPI_CR2_TXEIE);

	if( temp1 && temp2)
	{
		//handle TXE
		spi_txe_interrupt_handle(pSPIHandle);
	}

	//check for RXNE
	temp1 = pSPIHandle->pSPIx->SPI_SR & (1<<SPI_SR_RXNE);
	temp2 = pSPIHandle->pSPIx->SPI_CR2 & (1<<SPI_CR2_RXNEIE);

	if( temp1 && temp2)
	{
	      //handle RXNE
		spi_rxne_interrupt_handle(pSPIHandle);
	}

	//check for ovr flag
	temp1 = pSPIHandle->pSPIx->SPI_SR & (1<<SPI_SR_OVR);
	temp2 = pSPIHandle->pSPIx->SPI_CR2 & (1<<SPI_CR2_ERRIE);

	if( temp1 && temp2)
	{
		  //handle RXNE
		  spi_ovr_err_interrupt_handle(pSPIHandle);
	}

}



static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	if ( pSPIHandle->pSPIx->SPI_CR1 & (1<<SPI_CR1_DFF))
	{
		//16 bit DFF
		//1.load data into DR
		pSPIHandle->pSPIx->SPI_DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		 (uint16_t*)pSPIHandle->pTxBuffer++;
	}else
	{
	     //8 bit DFF
	     pSPIHandle->pSPIx->SPI_DR = *pSPIHandle->pTxBuffer;
	     pSPIHandle->TxLen--;
	     pSPIHandle->pTxBuffer++;
    }

	if(! pSPIHandle->TxLen)
	{
//Txlen is zero,so close spi transmission and inform the application that TX is over

		//this prevents interrupts from setting up of TXE flag
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_CMPLT);
	}
}


static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	if ( pSPIHandle->pSPIx->SPI_CR1 & (1<<SPI_CR1_DFF))
	{
		//16 bit DFF
		//1.load data into DR
		*(uint16_t *)pSPIHandle->pRxBuffer =pSPIHandle->pSPIx->SPI_DR;
		pSPIHandle->RxLen -= 2;
		pSPIHandle->pRxBuffer--;
		pSPIHandle->pRxBuffer--;
	}else
	{
	     //8 bit DFF
	     *(pSPIHandle->pRxBuffer) =(uint8_t)pSPIHandle->pSPIx->SPI_DR;
	     pSPIHandle->RxLen--;
	     pSPIHandle->pRxBuffer--;
    }

	if(! pSPIHandle->RxLen)
	{
        //reception  is complete
		//lets turn off the rxneie interrupt
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_RX_CMPLT);
	}
}


static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	 uint8_t temp;
	 //1.clear the over flag
	 if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	 {
		 temp = pSPIHandle->pSPIx->SPI_DR;
		 temp = pSPIHandle->pSPIx->SPI_SR;

	 }
	 (void)temp;
	 //2.inform the application
	 SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_OVR_ERR);
}


void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->SPI_CR2  &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;

}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->SPI_CR2  &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}


void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->SPI_DR;
	temp = pSPIx->SPI_SR;
	(void)temp;

}


__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv)
{

}
















































