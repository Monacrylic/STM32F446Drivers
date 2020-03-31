/*
 * 003_ButtonInterrupt.c
 *
 *  Created on: 20-Mar-2020
 *      Author: siddk
 */
#include "stm32f446xx.h"
#include "string.h"

void delay(void){
	for(uint32_t i=0; i<=500000/2; i++);
	//200ms delay, clock is running at 16MHz
}


int main(void ){
	GPIO_Handle_t pushButton;
	GPIO_Handle_t led;

	memset(&led, 0, sizeof(led));
	memset(&pushButton, 0, sizeof(pushButton));

	led.pGPIOx= GPIOA;
	led.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_5;
	led.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	led.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	led.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	led.GPIO_PinConfig.GPIO_PinSpeed= GPIO_SPEED_FAST;
	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&led);

	pushButton.pGPIOx= GPIOC;
	pushButton.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_5;
	pushButton.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_IT_FT;
	pushButton.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	pushButton.GPIO_PinConfig.GPIO_PinSpeed= GPIO_SPEED_FAST;
	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&pushButton);

	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRI15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);

	while(1);

	return 0;
}

void EXTI9_5_IRQHandler(void){
	delay();
	GPIO_IRQHandling(GPIO_PIN_NO_5);
	GPIO_ToggleOutputPin(GPIOA , GPIO_PIN_NO_5);

}
