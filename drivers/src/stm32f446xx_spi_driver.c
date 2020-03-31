/*
 * stm32f446xx_spi_driver.c
 *
 *  Created on: 21-Mar-2020
 *      Author: siddk
 */

#include "stm32f446xx_spi_driver.h"
#include "stddef.h"
/******************************************************************************
 * @fn		- SPI_PeriClockControl
 *
 * @brief	-
 *
 * @param1	-
 * @param2	-
 * @param3	-
 *
 * @return	-
 *
*/

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);


void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi){

	if (EnorDi == ENABLE){
		if (pSPIx == SPI1){
			SPI1_PCLK_EN();
		}
		else if(pSPIx == SPI2){
			SPI2_PCLK_EN();
		}
		else if(pSPIx == SPI3){
			SPI3_PCLK_EN();
		}
		else if(pSPIx == SPI4){
			SPI4_PCLK_EN();
		}

	}
	else{
		if (pSPIx == SPI1){
			SPI1_PCLK_DI();
		}
		else if(pSPIx == SPI2){
			SPI2_PCLK_DI();
		}
		else if(pSPIx == SPI3){
			SPI3_PCLK_DI();
		}
		else if(pSPIx == SPI4){
			SPI4_PCLK_DI();
		}

	}
}

/******************************************************************************
 * @fn		- SPI_Init
 *
 * @brief	-
 *
 * @param1	-
 * @param2	-
 * @param3	-
 *
 * @return	-
 *
*/

/* Init and Deinit */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
//First we configure SPI_CR1 Register
	uint32_t tempreg=0;
//1. Configure Device mode
tempreg |= (pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR);

//2. Configure the BUS_CONFIG
if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD){
	//BIDI (Bi-directional mode should be cleared
	tempreg &= ~(1<< SPI_CR1_BIDIMODE);

} else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD){
	// BIDI mode should be set
	tempreg |= 1<< SPI_CR1_BIDIMODE;

} else if(pSPIHandle-> SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY){
	//BIDI mode should be cleared
	tempreg &= ~(1<< SPI_CR1_BIDIMODE);
	//RXONLY bit must be set
	tempreg |= (1<< SPI_CR1_RXONLY);
}
//3. SPI clock
	tempreg |= (pSPIHandle ->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR );

//4. Configure the DFF
	tempreg |= (pSPIHandle ->SPIConfig.SPI_DFF << SPI_CR1_DFF );

//5. Configure CPOL
	tempreg |= (pSPIHandle ->SPIConfig.SPI_CPOL << SPI_CR1_CPOL );

//6. Configure CPHA
	tempreg |= (pSPIHandle ->SPIConfig.SPI_CPHA << SPI_CR1_CPHA );

	pSPIHandle->pSPIx->CR1= tempreg;
}
void SPI_DeInit(SPI_RegDef_t *pSPIx);



/*
 * Function to tell if the flag is set or not
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName){
	if(pSPIx->SR & FlagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
}
/*
 * Data Send and receive
 */

/******************************************************************************
 * @fn		- SPI_SendData
 *
 * @brief	-
 *
 * @param1	-
 * @param2	-
 * @param3	-
 *
 * @return	- none
 *
 * @note- This is a blocking call (while loops)
 *
*/

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len){

	while(Len>0){
		//1. Wait until TXE is SET
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		//2. Check the DFF Bit in CR1
		if((pSPIx -> CR1 & ( 1 << SPI_CR1_DFF)) ){
			//16 bit DFF
			pSPIx -> DR = *((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
		}else{
			//8 Bit DFF
			pSPIx -> DR = *(pTxBuffer);
			Len--;
			pTxBuffer++;
		}
	}
}

/******************************************************************************
 * @fn		- SPI_ReceiveData
 *
 * @brief	-
 *
 * @param1	-
 * @param2	-
 * @param3	-
 *
 * @return	-
 *
*/

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len){

	while(Len>0){
		//1. Wait until RXNE is SET
		while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

		//2. Check the DFF Bit in CR1
		if((pSPIx -> CR1 & ( 1 << SPI_CR1_DFF)) ){
			//16 bit DFF
			*((uint16_t*)pRxBuffer)= pSPIx -> DR ;
			Len--;
			Len--;
			(uint16_t*)pRxBuffer++;
		}else{
			//8 Bit DFF
			*(pRxBuffer)= pSPIx -> DR ;
			Len--;
			pRxBuffer++;
		}
	}
}

/*
 * IRQ  Configuration and ISR Handling
 */

/******************************************************************************
 * @fn		- SPI_IRQInterruptConfig
 *
 * @brief	-
 *
 * @param1	-
 * @param2	-
 * @param3	-
 *
 * @return	-
 *
*/

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= ( 1 << IRQNumber );

		}else if(IRQNumber > 31 && IRQNumber < 64 ) //32 to 63
		{
			//program ISER1 register
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			//program ISER2 register //64 to 95
			*NVIC_ISER2 |= ( 1 << (IRQNumber % 64) );
		}
	}else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= ( 1 << IRQNumber );
		}else if(IRQNumber > 31 && IRQNumber < 64 )
		{
			//program ICER1 register
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			//program ICER2 register
			*NVIC_ICER2 |= ( 1 << (IRQNumber % 64) );
		}
	}

}

/******************************************************************************
 * @fn		- SPI_IRQPriorityConfig
 *
 * @brief	-
 *
 * @param1	-
 * @param2	-
 * @param3	-
 *
 * @return	-
 *
*/

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){

	uint8_t iprx= IRQNumber/4 ;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASEADDR + iprx ) |= (IRQPriority << shift_amount );

}

/******************************************************************************
 * @fn		- SPI_IRQHandling
 *
 * @brief	-
 *
 * @param1	-
 * @param2	-
 * @param3	-
 *
 * @return	-
 *
*/

void SPI_IRQHandling(SPI_Handle_t *pHandle){
	uint8_t temp1, temp2;
	//first lets check for TXE
	temp1 = pHandle -> pSPIx ->SR & (1 << SPI_SR_TXE);
	temp2 = pHandle -> pSPIx-> CR2 & (1 << SPI_CR2_TXEIE);

	if(temp1 && temp2)
	{
		//Handle TXE
		spi_txe_interrupt_handle(pHandle);

	}

	// Check for RXNE
	temp1 = pHandle -> pSPIx ->SR & (1 << SPI_SR_RXNE);
	temp2 = pHandle -> pSPIx-> CR2 & (1 << SPI_CR2_RXNEIE);

	if(temp1 && temp2)
	{
		//Handle RXNE
		spi_rxne_interrupt_handle(pHandle);
	}

	// Check for OVR
	temp1 = pHandle -> pSPIx ->SR & (1 << SPI_SR_OVR);
	temp2 = pHandle -> pSPIx-> CR2 & (1 << SPI_CR2_ERRIE);

	if(temp1 && temp2)
	{
		//Handle OVR
		spi_ovr_err_interrupt_handle(pHandle);
	}
}
/*
 * Other Peripheral control APIs
 */

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi){
	if(EnOrDi == ENABLE){
		pSPIx->CR1 |= (1<< SPI_CR1_SPE);
	}else{
		pSPIx->CR1 &= ~(1<< SPI_CR1_SPE);
	}
}

/*
 * Send Data in Interrupt Mode
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len){
	uint8_t state= pSPIHandle->TxState;
	if(state != SPI_BUSY_IN_TX){
		//1. Save the TX buffer Address and Len information in global variables
		pSPIHandle ->pTxBuffer = pTxBuffer;
		pSPIHandle ->TxLen = Len;

		//2. Mark the SPI state as busy in transmission so that no
		//other code can take over same SPI Peripheral until transmission is over
		pSPIHandle ->TxState= SPI_BUSY_IN_TX;
		//3. Enable the TXEIE Control Bit to get Interrupt whenever TXE flag is set in SR
		pSPIHandle ->pSPIx -> CR2 |= (1 << SPI_CR2_TXEIE);
	}
	//4. Data transmission will be handled by the ISR Code
	return state;
}
/*
 * Receive Data in Interrupt Mode
 */
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len){

	uint8_t state= pSPIHandle->RxState;
	if(state != SPI_BUSY_IN_RX){
		//1. Save the TX buffer Address and Len information in global variables
		pSPIHandle ->pRxBuffer = pRxBuffer;
		pSPIHandle ->RxLen = Len;

		//2. Mark the SPI state as busy in transmission so that no
		//other code can take over same SPI Peripheral until transmission is over
		pSPIHandle ->RxState= SPI_BUSY_IN_RX;
		//3. Enable the TXEIE Control Bit to get Interrupt whenever TXE flag is set in SR
		pSPIHandle ->pSPIx -> CR2 |= (1 << SPI_CR2_RXNEIE);
	}
	//4. Data transmission will be handled by the ISR Code
	return state;

}

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi){
	if(EnOrDi == ENABLE){
		pSPIx->CR1 |= (1<< SPI_CR1_SSI);
	}else{
		pSPIx->CR1 &= ~(1<< SPI_CR1_SSI);
	}
}

// Make SSOE 1 To enable NSS
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi){
	if(EnOrDi == ENABLE){
		pSPIx->CR1 |= (1<< SPI_CR2_SSOE);
	}else{
		pSPIx->CR1 &= ~(1<< SPI_CR2_SSOE);
	}
}

//Some helper function definitions
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle){
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer= NULL;
	pSPIHandle->TxLen=0;
	pSPIHandle->TxState=SPI_READY;

}
void SPI_CloseReception(SPI_Handle_t *pSPIHandle){
	pSPIHandle ->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer= NULL;
	pSPIHandle->RxLen=0;
	pSPIHandle->RxState=SPI_READY;

}

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle){
	//2. Check the DFF Bit in CR1
	if((pSPIHandle-> pSPIx -> CR1 & ( 1 << SPI_CR1_DFF)) ){
		//16 bit DFF
		pSPIHandle-> pSPIx -> DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle-> TxLen--;
		pSPIHandle-> TxLen--;
		(uint16_t*)pSPIHandle-> pTxBuffer++;
	}else{
		//8 Bit DFF
		pSPIHandle-> pSPIx -> DR = *(pSPIHandle-> pTxBuffer);
		pSPIHandle-> TxLen--;
		pSPIHandle-> pTxBuffer++;
	}
	if (! pSPIHandle ->TxLen){

		//close the SPI comm
		//And inform the app that TX is over
		SPI_CloseTransmission(pSPIHandle);


	}
}
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle){
//2. Check the DFF Bit in CR1
	if((pSPIHandle-> pSPIx -> CR1 & ( 1 << 11)) ){
		//16 bit DFF
		*((uint16_t*)pSPIHandle->pRxBuffer)= (uint16_t)pSPIHandle-> pSPIx -> DR ;
		pSPIHandle-> RxLen--;
		pSPIHandle-> RxLen--;
		pSPIHandle-> pRxBuffer--;
		pSPIHandle-> pRxBuffer--;
	}else{
		//8 Bit DFF
		pSPIHandle-> pSPIx -> DR = *(pSPIHandle-> pRxBuffer);
		pSPIHandle-> RxLen--;
		pSPIHandle-> pRxBuffer--;
	}
	if (! pSPIHandle ->RxLen){

		//close the SPI comm
		//And inform the app that TX is over

		SPI_CloseReception(pSPIHandle);

	}
}
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle){

	uint8_t temp;
	//1. Clear the ovr flag
	if(pSPIHandle -> TxState != SPI_BUSY_IN_TX){
		temp= pSPIHandle -> pSPIx ->DR;
		temp= pSPIHandle -> pSPIx ->SR;
	}
	(void)temp;
	//2. Inform the Application
}

