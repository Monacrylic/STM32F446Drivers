/*
 * stm32f446xx_gpio_driver.c
 *
 *  Created on: 16-Mar-2020
 *      Author: Siddharth Kothari
 */



#include "stm32f446xx_gpio_driver.h"

/*
 * Peripheral clock setup
 * Function- GPIOPeriClockControl()
 *
 * Brief- Enables or Disables the peripheral clock for GPIOx.
 *
 * Param1- Base Address of GPIO Peripheral
 *
 * Param2- Enable or Disable Macros
 *
 * Return- none
 *
 * Note- none
 *
 *
 */


void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi){
if (EnorDi == ENABLE){
	if (pGPIOx == GPIOA){
		GPIOA_PCLK_EN();
	}
	else if(pGPIOx == GPIOB){
		GPIOB_PCLK_EN();
	}
	else if(pGPIOx == GPIOC){
		GPIOC_PCLK_EN();
	}
	else if(pGPIOx == GPIOD){
		GPIOD_PCLK_EN();
	}
	else if(pGPIOx == GPIOE){
		GPIOE_PCLK_EN();
	}
	else if(pGPIOx == GPIOF){
		GPIOF_PCLK_EN();
	}
	else if(pGPIOx == GPIOG){
		GPIOG_PCLK_EN();
	}
	else if(pGPIOx == GPIOH){
		GPIOH_PCLK_EN();
	}
}
else{
if (pGPIOx == GPIOA){
		GPIOA_PCLK_DI();
	}
	else if(pGPIOx == GPIOB){
		GPIOB_PCLK_DI();
	}
	else if(pGPIOx == GPIOC){
		GPIOC_PCLK_DI();
	}
	else if(pGPIOx == GPIOD){
		GPIOD_PCLK_DI();
	}
	else if(pGPIOx == GPIOE){
		GPIOE_PCLK_DI();
	}
	else if(pGPIOx == GPIOF){
		GPIOF_PCLK_DI();
	}
	else if(pGPIOx == GPIOG){
		GPIOG_PCLK_DI();
	}
	else if(pGPIOx == GPIOH){
		GPIOH_PCLK_DI();
	}
}

}

/* Init and Deinit */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){
	uint32_t temp=0;

	//0. Enable the Peripheral clock

	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);
	// 1.Configure mode of GPIO PIn
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <=GPIO_MODE_ANALOG){
		temp= (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~ (0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) ;//clearing
		pGPIOHandle->pGPIOx->MODER |= temp; //setting
	}
	else
	{
		//interrupt mode
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT){
			//Configure the Falling trigger Selection Register- FTSR
			EXTI->FTSR |= (1<< pGPIOHandle ->GPIO_PinConfig.GPIO_PinNumber);
			//clear the corresponding RTSR
			EXTI->RTSR &= ~(1<< pGPIOHandle ->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT){
			//Configure the Rising Trigger Selection Register- RTSR
			EXTI->RTSR |= (1<< pGPIOHandle ->GPIO_PinConfig.GPIO_PinNumber);
			//clear the corresponding FTSR
			EXTI->FTSR &= ~(1<< pGPIOHandle ->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT){
			//configure both the registers FTSR and RTSR
			EXTI->RTSR |= (1<< pGPIOHandle ->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= (1<< pGPIOHandle ->GPIO_PinConfig.GPIO_PinNumber);
		}

		//2. Configure the GPIO Port Selection in SYSCFG_EXTICR
		uint8_t temp1= pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber /4;
		uint8_t temp2= pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber %4;
		uint8_t portcode= BASEADDR_TO_PORTCODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << (4 * temp2);
		//3. Enable the EXTI interrupt delivery using IMR
		EXTI->IMR |= (1<<pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);

	}

	temp=0;
	// 2.Configure the Speed
	temp= (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed <<(2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~ (0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) ;//clearing
	pGPIOHandle->pGPIOx->OSPEEDR |=temp;
	temp=0;


	// 3.Configure the PuPd setting
	temp= (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl <<(2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ));
	pGPIOHandle->pGPIOx->PUPDR &= ~ (0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) ;//clearing
	pGPIOHandle->pGPIOx->PUPDR |=temp;
	temp=0;
	// 4. Configure the OP type

	temp= (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType <<( pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ));
	pGPIOHandle->pGPIOx->OTYPER &= ~ (0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) ;//clearing
	pGPIOHandle->pGPIOx->OTYPER |=temp;
	temp=0;

	// 5. Configure the alternate functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN){
		uint8_t temp1, temp2;
		temp1 = pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber/8;
		temp2 = pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~ (0xF << (4* temp2)) ;//clearing
		pGPIOHandle-> pGPIOx->AFR[temp1] |=  (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4* temp2) );
	}
}
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){

	    if (pGPIOx == GPIOA){
			GPIOA_REG_RESET();
		}
		else if(pGPIOx == GPIOB){
			GPIOB_REG_RESET();
		}
		else if(pGPIOx == GPIOC){
			GPIOC_REG_RESET();
		}
		else if(pGPIOx == GPIOD){
			GPIOD_REG_RESET();
		}
		else if(pGPIOx == GPIOE){
			GPIOE_REG_RESET();
		}
		else if(pGPIOx == GPIOF){
			GPIOF_REG_RESET();
		}
		else if(pGPIOx == GPIOG){
			GPIOG_REG_RESET();
		}
		else if(pGPIOx == GPIOH){
			GPIOH_REG_RESET();
		}

}

/* Read and write*/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){

	uint8_t value;
	value= (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001 ) ;
	return value;
}


uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){

	uint16_t value;
	value= (uint16_t)(pGPIOx->IDR);
	return value;
}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value){

	pGPIOx->ODR= Value;

}
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value){

	if (Value== GPIO_PIN_SET){
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}

}
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){

	pGPIOx->ODR ^= (1 << PinNumber);
}

/* Interrupt handling */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber,  uint8_t EnorDi){
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

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){

	uint8_t iprx= IRQNumber/4 ;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASEADDR + iprx ) |= (IRQPriority << shift_amount );

}

void GPIO_IRQHandling(uint8_t PinNumber){
//1. Find out the IPR Register

	if(EXTI->PR & (1 << PinNumber)){
		//clear
		EXTI->PR |= (1 << PinNumber);
	}
}

