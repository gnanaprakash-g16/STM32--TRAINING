/*
 * stm32f4xx_i2c_driver.c
 *
 *  Created on: Oct 7, 2025
 *      Author: gnanaprakash.g
 */
#include "stm32f411xx_i2c_driver.h"

uint16_t AHB_PreScaler[8] = {2,4,8,16,63,128,256,512};
uint8_t APB1_PreScaler[4] = {2,4,8,16};

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressesPhaseWrite(I2C_RegDef_t *pI2Cx , uint8_t SlaveAddr);
static void I2C_ExecuteAddressesPhaseRead(I2C_RegDef_t *pI2Cx , uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle);

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->I2C_CR1 |= (1 << I2C_CR1_START);
}

static void I2C_ExecuteAddressesPhaseWrite(I2C_RegDef_t *pI2Cx , uint8_t SlaveAddr)
{
      SlaveAddr = SlaveAddr << 1;
      SlaveAddr &= ~(1);
      pI2Cx->I2C_DR = SlaveAddr;
}

static void I2C_ExecuteAddressesPhaseRead(I2C_RegDef_t *pI2Cx , uint8_t SlaveAddr)
{
      SlaveAddr = SlaveAddr << 1;
      SlaveAddr |= 1;
      pI2Cx->I2C_DR = SlaveAddr;
}

static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle)
{
    uint32_t dummy_Read;
    //check for device mode
    if(pI2CHandle->pI2Cx->I2C_SR2 & ( 1<< I2C_SR2_MSL))
    {
    	//device is in master mode
    	if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
    	{
    		if(pI2CHandle->RxSize == 1)
    		{
    			//first disable the ack
    			I2C_ManageAck(pI2CHandle->pI2Cx,DISABLE);

    			//clear the ADDR flag( read SR1, read SR2)
                dummy_Read = pI2CHandle->pI2Cx->I2C_SR1;
                dummy_Read = pI2CHandle->pI2Cx->I2C_SR2;
                (void)dummy_Read;
    		}
    	}else
    	{
    		//clear the ADDR flag( read SR1, read SR2)
    		dummy_Read = pI2CHandle->pI2Cx->I2C_SR1;
    		dummy_Read = pI2CHandle->pI2Cx->I2C_SR2;
    		(void)dummy_Read;
    	}
    }else
    {
    	//device is in slave mode
		//clear the ADDR flag( read SR1, read SR2)
		dummy_Read = pI2CHandle->pI2Cx->I2C_SR1;
		dummy_Read = pI2CHandle->pI2Cx->I2C_SR2;
		(void)dummy_Read;
    }

}

static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->I2C_CR1 |= (1 << I2C_CR1_STOP);
}

uint32_t RCC_GetPLLOutputClock()
{
	return 0;
}

uint32_t RCC_GetPCLKValue()
{
    uint32_t pclk1,SystemClK,temp,ahbp,apb;

    uint8_t clksrc;

    clksrc = ((RCC->CFGR>>2) & 0x03);

    if(clksrc == 0)
    {
    	SystemClK = 16000000;
    }else if(clksrc == 1)
    {
    	SystemClK = 8000000;
    }else if(clksrc == 2)
    {
    	SystemClK = RCC_GetPLLOutputClock();
    }
    //ahb bus
    temp = ((RCC->CFGR >> 4) & 0xF);

    if(temp <8)
    {
    	ahbp = 1;
    }else
    {
    	ahbp = AHB_PreScaler[temp-8];
    }

    //apb bus
    temp = ((RCC->CFGR >> 10) & 0x7);
    if(temp <4)
    {
       apb = 1;
    }else
    {
       apb = APB1_PreScaler[temp-4];
    }

    pclk1 = (SystemClK/ahbp)/apb;


    return pclk1;
}

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx,uint8_t EnOrDi)
{
     if(EnOrDi == ENABLE)
     {
    	 pI2Cx->I2C_CR1 |= (1 << I2C_CR1_PE);
     }else
     {
    	 pI2Cx->I2C_CR1 &= ~(1 << 0);
     }
}

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx,uint8_t EnOrDi)
{
	  if(EnOrDi == ENABLE)
		  {
			  if(pI2Cx == I2C1)
			  {
				  I2C1_PCLK_EN();
			  }
			  else if(pI2Cx == I2C2)
			  {
				  I2C2_PCLK_EN();
			  }
			  else if(pI2Cx == I2C3)
			  {
			  	  I2C3_PCLK_EN();
			  }
		  }
		  else
		  {
			  if(pI2Cx == I2C1)
			  {
			     I2C1_PCLK_DI();
			  }
			  else if(pI2Cx == I2C2)
			  {
			     I2C2_PCLK_DI();
			  }
			  else if(pI2Cx == I2C3)
			  {
			 	 I2C3_PCLK_DI();
			  }
		  }
}

void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	  uint32_t temp=0;


	  //enable the clock for i2C peripheral
	  I2C_PeriClockControl(pI2CHandle->pI2Cx,ENABLE);


	  temp |= (pI2CHandle->I2C_Config.I2C_ACKControle <<10) ;
	  pI2CHandle->pI2Cx->I2C_CR1 = temp;

	  //configure the FREQ field of cr2
	  temp = 0;
	  temp |= RCC_GetPCLKValue()/1000000U;
	  pI2CHandle->pI2Cx->I2C_CR2 = (temp & 0x3F);


	  //Program the device own address
	  temp|= pI2CHandle->I2C_Config.I2C_DeviceAddress<<1;
	  temp |= ( 1 << 14);
	  pI2CHandle->pI2Cx->I2C_OAR1 = temp;


	  //CCR calculation
	  uint16_t ccr_value = 0;
	  temp = 0;
	  if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	  {
		  //mode is standard mode
		  ccr_value = (RCC_GetPCLKValue() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		  temp |= (ccr_value & 0xFFF);
	  }else
	  {
		  //mode is fast mode
		  temp |= (1<<15);
		  temp |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);
		  if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		  {
			  ccr_value = (RCC_GetPCLKValue() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		  }
		  else
		  {
			  ccr_value = (RCC_GetPCLKValue() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		  }
		  temp |= (ccr_value & 0xFFF);
	  }
	  pI2CHandle->pI2Cx->I2C_CCR = temp;

	  //TRISE Configuration
	  if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	  {
		  //mode is standard mode
		  temp = (RCC_GetPCLKValue() / 1000000U)+1;//1000ns=1microsecond=>10^-6=>/1000000
	  }else
	  {
		  //mode is fast mode
		  temp = ((RCC_GetPCLKValue()*300) / 1000000000U)+1;//maximum time is 300ns
	  }

	  pI2CHandle->pI2Cx->I2C_TRISE = (temp & 0x3F);

}

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx , uint32_t FlagName)
{
	if(pI2Cx->I2C_SR1 & FlagName) return FLAG_SET;

	return FLAG_RESET;
}

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer,uint32_t Len,uint8_t SlaveAddr,uint8_t Sr)
{
	// 1. Generate the start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. confirm that start generation is completed by checking the SB flag in SR1
	//Until SB is cleared SCL will be streched low
	while( !I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_SB));

	//3.Send the addresses of slave with r/w bit set to  write(0)(total 8 bits)
	I2C_ExecuteAddressesPhaseWrite(pI2CHandle->pI2Cx,SlaveAddr);

    //4.Confirm the address phase is completed by checking the ADDR flag in the SR1
	while( !I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_ADDR));

	//5.Clear the ADDR flag according to its software sequence
	//until ADDR is cleared SCL will be stretched (pulled low)
	I2C_ClearADDRFlag(pI2CHandle->pI2Cx);


	//6.Send the data until len becomes 0
	while(Len > 0)
	{
		while( !I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_TXE));
		pI2CHandle->pI2Cx->I2C_DR = *pTxbuffer;
		pTxbuffer++;
		Len--;
	}

	//7.when len becomes 0 wait for TXE=1& BTF=1 before generating STOP Condition
    //TXE=1,BT=1,means that both SR & DR are empty & next transmission should begin
	//when BTF=1 SCL will be stretched to LOW
	while( !I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_TXE));
	while( !I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_BTF));

	//8. Generate STOP condition and master need not to wait for the completion of stop condition
	//Generating STOP,automatically clears the BTF

	if(Sr == I2C_NO_SR) I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
}

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle,uint8_t *pRxbuffer,uint32_t Len,uint8_t SlaveAddr,uint8_t Sr)
{
    //1.Generate the START Condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. confirm that start generation is completed by checking the SB flag in SR1
		//Until SB is cleared SCL will be streched low
    while( !I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_SB));

	//3.Send the addresses of slave with r/w bit set to read(1)(total 8 bits)
	I2C_ExecuteAddressesPhaseRead(pI2CHandle->pI2Cx,SlaveAddr);

	//4.Wait until address phase is completed, by checking the ADDR flag in the  SR1
	while( !I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_ADDR));

	//procedure to read only 1 byte from slave
	if(Len == 1)
	{
         //Disable Acking
		I2C_ManageAck(pI2CHandle->pI2Cx,I2C_ACK_DISABLE);

		//Clear the ADDR FLag
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

		//wait until RXNE becomes  1
		while( !I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_RXNE));

		//generates STOP Condition
		if(Sr == I2C_NO_SR) I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		//read data in to buffer
		*pRxbuffer =pI2CHandle->pI2Cx->I2C_DR;
	}

	if( Len > 1)
	{
		//Clear the ADDR FLag
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

	    //wait until RXNE becomes  1
		while( !I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_RXNE));

		//read the dat until Len becomes zero
		for ( uint32_t i=Len;i > 0 ; i--)
		{

			//wait until RXNE becomes  1
		    while( !I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_RXNE));

		if(i==2)//last 2 bytes remaining
		{
		   //Disable Acking
			I2C_ManageAck(pI2CHandle->pI2Cx,I2C_ACK_DISABLE);

			//generates STOP Condiition
			if(Sr == I2C_NO_SR) I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		}

		//read the data from data register in to buffer
		*pRxbuffer =pI2CHandle->pI2Cx->I2C_DR;

		//increment the buffer address
		pRxbuffer++;
	}
	}

	//re-enable ACKing
	if(pI2CHandle->I2C_Config.I2C_ACKControle == I2C_ACK_ENABLE)
	{
		I2C_ManageAck(pI2CHandle->pI2Cx,I2C_ACK_ENABLE);
	}
}

void I2C_ManageAck(I2C_RegDef_t *pI2Cx , uint8_t EnOrDi)
{
       if(EnOrDi == I2C_ACK_ENABLE)
       {
    	   pI2Cx->I2C_CR1 |=  ( 1<< I2C_CR1_ACK);
       }else
       {
    	   pI2Cx->I2C_CR1 &=  ~( 1<< I2C_CR1_ACK);
       }
}

//IRQ Configuration and ISR Handling
void I2C_IRQInterruptConfig(uint8_t IRQNumber , uint8_t EnOrDi)
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



void I2C_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	//1.first lets find out ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_Section = IRQNumber % 4;

	uint8_t shift_Amount = (8 * iprx_Section ) + (8 - PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_Amount);

}

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer,uint32_t Len,uint8_t SlaveAddr,uint8_t Sr)
{
     uint8_t busystate = pI2CHandle->TxRxState;

     if((busystate != I2C_BUSY_IN_TX ) && (busystate != I2C_BUSY_IN_RX))
     {
    	 pI2CHandle->pTxBuffer = pTxbuffer;
    	 pI2CHandle->TxLen = Len;
    	 pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
    	 pI2CHandle->DevAddr = SlaveAddr;
    	 pI2CHandle->Sr = Sr;

    	 //Implement code to generate START Condition
    	 I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

    	 //Implement the code to enable ITBUFEN Control Bit
    	 pI2CHandle->pI2Cx->I2C_CR2 |= ( 1<< I2C_CR2_ITBUFEN);

    	 //Implement the code to enable ITBUFEN Control Bit
    	 pI2CHandle->pI2Cx->I2C_CR2 |= ( 1<< I2C_CR2_ITEVTEN);

    	 //Implement the code to enable ITBUFEN Control Bit
    	 pI2CHandle->pI2Cx->I2C_CR2 |= ( 1<< I2C_CR2_ITERREN);

     }

     return busystate;
}


uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxbuffer,uint32_t Len,uint8_t SlaveAddr,uint8_t Sr)
{

}

static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle)
{
	if(pI2CHandle->TxLen > 0)
	{
	    //1.Load the data in to DR
		pI2CHandle->pI2Cx->I2C_DR = *(pI2CHandle->pTxBuffer);

		//2.decrement the TxLen
		pI2CHandle->TxLen--;

		//3.Increment the buffer address
	    pI2CHandle->pTxBuffer++;
		}
}


static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle)
{
	//we have to do the data reception
				if(pI2CHandle->RxSize == 1)
				{
					*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->I2C_DR;
					pI2CHandle->RxLen--;
				}
				if(pI2CHandle->RxSize > 1)
				{
	                 if(pI2CHandle->RxLen == 2)
	                 {
	                	 //clear the ack bit
	                	 I2C_ManageAck(pI2CHandle->pI2Cx,DISABLE);
	                 }
	                 //read DR
	                 *pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->I2C_DR;
	                 pI2CHandle->pRxBuffer++;
	                 pI2CHandle->RxLen--;
				}
				if(pI2CHandle->RxLen == 0)
				{
					//close the I2C data reception

					//1.generate the stop condition
					if(pI2CHandle->Sr == I2C_NO_SR) I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

					//2. CLose the I2C rx
					I2C_CloseReceiveData(pI2CHandle);

					//3. Notify the application
					I2C_ApplicationEventCallBack(pI2CHandle,I2C_EV_RX_CMPLT);
				}
}

void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
     //Interrupt handling for both master and slave mode of device

	uint32_t temp1, temp2, temp3;

	temp1 = pI2CHandle->pI2Cx->I2C_CR2 & ( 1<< I2C_CR2_ITEVTEN);
	temp2 = pI2CHandle->pI2Cx->I2C_CR2 & ( 1<< I2C_CR2_ITEVTEN);

	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & ( 1<< I2C_SR1_SB);

	//1.Handle For Interrupt generated by SB event
	//Note : SB Flag is only applicable in  master mode
	if(temp1 && temp3)
	{
		//SB flag is set
		//This interrupt is generated because of SB event
		//This block will not be executed in slave mode because for slave SB is always zero
		//In this block lets execute the address block
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			I2C_ExecuteAddressesPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}else
		{
			I2C_ExecuteAddressesPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
	}

	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & ( 1 << I2C_SR1_ADD);

	//2. Handle for Interrupt generated by ADDR event
	//Note : When master mode : Address is sent
	//       When Slave mode  : Address matched with own address
	if( temp1 && temp3)
	{
		//ADDR flag is set
		//interrupt is generated because of ADDR event
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);
	}

	temp3 = pI2CHandle->pI2Cx->I2C_SR1 &  (1 << I2C_SR1_BTF);

	//3.Handle for interrupt generated by BTF(Byte Transfer Finished) event
	if(temp1 && temp3)
	{
		//btf flag is set
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			//make sure that TXE is also set
			if(pI2CHandle->pI2Cx->I2C_SR1 & ( 1<< I2C_SR1_TXE))
			{
				//if BTF & TXE = 1 , both Shift register and DR is empty

				if(pI2CHandle->TxLen == 0)
				{
				//1.generate the STOP condition
				if(pI2CHandle->Sr == I2C_NO_SR) I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

				//2.reset all member elements of handle structure
				I2C_CloseSendData(pI2CHandle);

				//3.notify the aplication about transmission complete
				I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_TX_CMPLT);
				}
			}
		}
	}

	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_STOPF);
	//4. Handle for interrupt generated by STOPF event
	// Note : Stop detection flag is applicable only slave mode,for master this flag will never be set
	//the below code block will not be executed by master since STOPF will not set in master mode

	if(temp1 && temp3)
	{
       //STOPF flag is set
	   //clear the STOPF (i.e 1) read SR1 ,write to CR1)

		pI2CHandle->pI2Cx->I2C_CR1 |= 0x0000;

		//notify application that stop is detected
		I2C_ApplicationEventCallBack(pI2CHandle,I2C_EV_STOP);
	}

	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_TXE);
	//5. Handle for interrupt generated by TXE event
	if(temp1 && temp2 && temp3)
	{
		//check for device mode
		if(pI2CHandle->pI2Cx->I2C_SR2 & ( 1 << I2C_SR2_MSL))
		{
	     //TXE flag is set
		//We have to do the data transmission
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
			{
				I2C_MasterHandleTXEInterrupt(pI2CHandle);
			}
	}

	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_RXNE);
	//6. Handle for interrupt generated by RXNE event
	if(temp1 && temp2 && temp3)
	{
		//check for device mode
	   if(pI2CHandle->pI2Cx->I2C_SR2 & ( 1 << I2C_SR2_MSL))
	   {
		 // The device is master
		 //RXNE flag is set
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			I2C_MasterHandleRXNEInterrupt(pI2CHandle);
			}
		}
	}
}
}

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->I2C_CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->I2C_CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	if(pI2CHandle->I2C_Config.I2C_ACKControle == I2C_ACK_ENABLE)
	{
		I2C_ManageAck(pI2CHandle->pI2Cx,ENABLE);
	}

}

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->I2C_CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->I2C_CR2 &= ~( 1 << I2C_CR2_ITEVTEN);


	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;
}















