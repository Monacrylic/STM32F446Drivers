/*
 * 004_Spi_tx_testing.c
 *
 *  Created on: 22-Mar-2020
 *      Author: siddk
 */

#include "stm32f446xx.h"
#include "string.h"
/*
 * PB12- SPI2_NSS
 * PB13- SPI2_SCK
 * PB14- SPI2_MISO
 * PB15- SPI2_MOSI
 * Alternate functionality mode AF5
 */

void SPI2_GPIO_Inits(){
	GPIO_Handle_t SPIPins;
	SPIPins.pGPIOx= GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode=5;
	SPIPins.GPIO_PinConfig.GPIO_PinMode= GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType= GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl= GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed= GPIO_SPEED_FAST;

	//nss
	SPIPins.GPIO_PinConfig.GPIO_PinNumber= GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);

	//sck
	SPIPins.GPIO_PinConfig.GPIO_PinNumber= GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber= GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	//MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber= GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);

}

void SPI2_Inits(){
	SPI_Handle_t SPI2Handle;
	SPI2Handle.pSPIx=SPI2;
	SPI2Handle.SPIConfig.SPI_DeviceMode= SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_BusConfig= SPI_BUS_CONFIG_FD;
	SPI2Handle.SPIConfig.SPI_CPHA=SPI_CPHA_LOW;
	SPI2Handle.SPIConfig.SPI_CPOL= SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_SclkSpeed=SPI_SCLK_SPEED_DIV2; //Generates clock of 16/2 MHz
	SPI2Handle.SPIConfig.SPI_DFF= SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_EN;

	SPI_Init(&SPI2Handle);
}

int main(void){
	SPI_PeriClockControl(SPI2, ENABLE);
	SPI2_GPIO_Inits();
	SPI2_Inits();
	char user_data[]= "Hello World";

	//TO prevent MODF ERROR
	SPI_SSIConfig(SPI2, ENABLE);

	SPI_PeripheralControl(SPI2, ENABLE);
    SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

    while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));
    SPI_PeripheralControl(SPI2, DISABLE);

    while(1){

    }
	return 0;
}

