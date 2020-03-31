/*
 * 002_ledButton.c
 *
 *  Created on: 19-Mar-2020
 *      Author: siddk
 */
#include "stm32f446xx.h"


int main(void){

GPIO_Handle_t pushButton;
GPIO_Handle_t led;

led.pGPIOx= GPIOA;
led.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_5;
led.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
led.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
led.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
led.GPIO_PinConfig.GPIO_PinSpeed= GPIO_SPEED_FAST;
GPIO_PeriClockControl(GPIOA, ENABLE);
GPIO_Init(&led);

pushButton.pGPIOx= GPIOC;
pushButton.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_13;
pushButton.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_IN;
pushButton.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
pushButton.GPIO_PinConfig.GPIO_PinSpeed= GPIO_SPEED_FAST;
GPIO_PeriClockControl(GPIOC, ENABLE);
GPIO_Init(&pushButton);

while(1){
	GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NO_5, !(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13)));
}
	return 0;
}
