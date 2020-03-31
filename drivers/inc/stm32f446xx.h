/*
 * stm32f446xx.h
 *
 *  Created on: Mar 11, 2020
 *      Author: Siddharth Kothari
 */


#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_

#include <stdint.h>

#define __vo volatile
#define __weak __attribute__((weak))
/***********************START: PROCESSOR SPECIFIC DETAILS**************************/
/*
 * ARM CORTEX Mx Processor NVIC ISERx register Addresses
 */
#define NVIC_ISER0		((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1		((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2		((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3		((__vo uint32_t*)0xE000E10C)

/*
 * ARM CORTEX Mx Processor NVIC ICERx register Addresses
 */
#define NVIC_ICER0		((__vo uint32_t*)0XE000E180)
#define NVIC_ICER1		((__vo uint32_t*)0XE000E184)
#define NVIC_ICER2		((__vo uint32_t*)0XE000E188)
#define NVIC_ICER3		((__vo uint32_t*)0XE000E18C)

/*
 * ARM CORTEX Mx Priority Register Address
 */

#define NVIC_PR_BASEADDR	((__vo uint32_t*)0xE000E400)

/*
 * No of priority bits implemented
 */
#define NO_PR_BITS_IMPLEMENTED 4



/***********************END: PROCESSOR SPECIFIC DETAILS**************************/

#define FLASH_BASEADDR 0x08000000UL
#define SRAM1_BASEADDR 0x20000000UL
#define SRAM SRAM1_BASEADDR
#define SRAM2_BASEADDR 0x2001C000UL
#define ROM 0x1FFF0000UL

/*Define AHBX and APBx Base Addresses */

#define PERIPH_BASEADDR			0x40000000UL
#define APB1PERIPH_BASEADDR 	PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR		0x40010000UL
#define AHB1PERIPH_BASEADDR		0x40020000UL
#define AHB2PERIPH_BASEADDR     0x50000000UL

/* Define the base addresses of all peripherals hanging off AHB1 Bus */

#define GPIOA_BASEADDR 		(AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR 		(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR 		(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR 		(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR 		(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR 		(AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR 		(AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR 		(AHB1PERIPH_BASEADDR + 0x1C00)
#define RCC_BASEADDR 		(AHB1PERIPH_BASEADDR + 0x3800)
/* Define the base addresses of all peripherals hanging off APB1 bus*/

#define I2C1_BASEADDR		(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR		(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR		(APB1PERIPH_BASEADDR + 0x5C00)
#define SPI2_BASEADDR		(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR		(APB1PERIPH_BASEADDR + 0x3C00)
#define USART2_BASEADDR		(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR		(APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR		(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR		(APB1PERIPH_BASEADDR + 0x5000)

/* Define all the base addresses of peripherals hanging off APB2 bus */
 #define EXTI_BASEADDR			(APB2PERIPH_BASEADDR + 0x3C00)
 #define USART1_BASEADDR		(APB2PERIPH_BASEADDR + 0x1000)
 #define USART6_BASEADDR		(APB2PERIPH_BASEADDR + 0x1400)
 #define SPI1_BASEADDR			(APB2PERIPH_BASEADDR + 0x3000)
 #define SPI4_BASEADDR			(APB2PERIPH_BASEADDR + 0x3400)
 #define SYSCFG_BASEADDR		(APB2PERIPH_BASEADDR + 0x3800)

/* Peripheral register definition structures */

typedef struct{
	__vo uint32_t MODER;		/* GPIO port mode register  Address offset 0x00*/
	__vo uint32_t OTYPER;	/* GPIO port output type register  Address offset 0x04*/
	__vo uint32_t OSPEEDR;	/* GPIO port output speed register*/
	__vo uint32_t PUPDR;		/* GPIO port pull-up/pull-down register*/
	__vo uint32_t IDR;		/* GPIO port input data register*/
	__vo uint32_t ODR;
	__vo uint32_t BSRR;		/* GPIO port bit set/reset register */
	__vo uint32_t LCKR;		/* GPIO port configuration lock register*/
	__vo uint32_t AFR[2];	/* AFR[0] : alternate function low register, AFR[1] : GPIO alternate function high register*/
}GPIO_RegDef_t;

typedef struct{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;
	__vo uint32_t I2SCFGR;
	__vo uint32_t I2SPR;

}SPI_RegDef_t;

typedef struct{
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t BRR;
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t CR3;
	__vo uint32_t GTPR;

}USART_RegDef_t;

typedef struct{
	__vo uint32_t CR;          /* 0x00 */
	__vo uint32_t PLLCFGR;	   /* 0x04 */
	__vo uint32_t CFGR;			/* 0x08 */
	__vo uint32_t CIR;			/* 0x0C */
	__vo uint32_t AHB1RSTR;		/* 0x10 */
	__vo uint32_t AHB2RSTR;		/* 0x14 */
	__vo uint32_t AHB3RSTR;		/* 0x18 */
	uint32_t 	  RESERVED0;	/* 0x1C */
	__vo uint32_t APB1RSTR;		/* 0x20 */
	__vo uint32_t APB2RSTR;		/* 0x24 */
	uint32_t RESERVED1[2];		/* 0x28 0x2C */
	__vo uint32_t AHB1ENR;		/* 0x30 */
	__vo uint32_t AHB2ENR;		/* 0x34 */
	__vo uint32_t AHB3ENR;		/* 0x38 */
	uint32_t RESERVED3;			/* 0x3C */
	__vo uint32_t APB1ENR;		/* 0x40 */
	__vo uint32_t APB2ENR;		/* 0x44 */
	uint32_t RESERVED4[2];		/* 0x48 0x4C */
	__vo uint32_t AHB1LPENR;	/* 0x50 */
	__vo uint32_t AHB2LPENR;	/* 0x54 */
	__vo uint32_t AHB3LPENR;	/* 0x58 */
	uint32_t RESERVED5;			/* 0x5C */
	__vo uint32_t APB1LPENR;	/* 0x60  */
	__vo uint32_t APB2LPENR;	/* 0x64 */
	uint32_t RESERVED6[2];		/* 0x68 0x6C */
	__vo uint32_t BDCR;			/* 0x70 */
	__vo uint32_t CSR;			/* 0x74 */
	uint32_t RESERVED7[2];		/* 0x78 0x7C */
	__vo uint32_t SSCGR;		/* 0x80 */
	__vo uint32_t PLLI2SCFGR;	/* 0x84 */
	__vo uint32_t PLLSAICFGR;	/* 0x88 */
	__vo uint32_t DCKCFGR;		/* 0x8C */
	__vo uint32_t CKGATENR;		/* 0x90 */
	__vo uint32_t DCKCFGR2;		/* 0x94 */

}RCC_RegDef_t;


/*
 * EXTI register definition structure
 */
typedef struct{
	__vo uint32_t  IMR;
	__vo uint32_t  EMR;
	__vo uint32_t  RTSR;
	__vo uint32_t  FTSR;
	__vo uint32_t  SWIER;
	__vo uint32_t  PR;
}EXTI_RegDef_t;


/*
 * SYSCFG register definition structure
 */

typedef struct{
	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4];
	uint32_t RESERVED1[2];
	__vo uint32_t CMPCR;
	uint32_t RESERVED2[2];
	__vo uint32_t CFGR;

}SYSCFG_RegDef_t;


/* Peripheral Definitions */

#define RCC              ((RCC_RegDef_t*)RCC_BASEADDR)
#define SYSCFG           ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)
#define EXTI             ((EXTI_RegDef_t*)EXTI_BASEADDR)
#define GPIOA			 (GPIO_RegDef_t*)GPIOA_BASEADDR
#define GPIOB			 (GPIO_RegDef_t*)GPIOB_BASEADDR
#define GPIOC			 (GPIO_RegDef_t*)GPIOC_BASEADDR
#define GPIOD			 (GPIO_RegDef_t*)GPIOD_BASEADDR
#define GPIOE			 (GPIO_RegDef_t*)GPIOE_BASEADDR
#define GPIOF			 (GPIO_RegDef_t*)GPIOF_BASEADDR
#define GPIOG			 (GPIO_RegDef_t*)GPIOG_BASEADDR
#define GPIOH			 (GPIO_RegDef_t*)GPIOH_BASEADDR
#define SPI1			 ((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2			 ((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3			 ((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4			 ((SPI_RegDef_t*)SPI4_BASEADDR)
#define USART1			 ((USART_RegDef_t*)USART1_BASEADDR)
#define USART2			 ((USART_RegDef_t*)USART2_BASEADDR)
#define USART3			 ((USART_RegDef_t*)USART3_BASEADDR)
#define UART4			 ((USART_RegDef_t*)UART5_BASEADDR)
#define UART5			 ((USART_RegDef_t*)UART5_BASEADDR)
#define USART6			 ((USART_RegDef_t*)USART6_BASEADDR)
/* Clock Enable Macros for GPIOx peripherals*/


#define GPIOA_PCLK_EN()		 ((RCC -> AHB1ENR) |= (1<<0))
#define GPIOB_PCLK_EN()		 (RCC -> AHB1ENR|= (1<<1))
#define GPIOC_PCLK_EN()		 (RCC -> AHB1ENR|= (1<<2))
#define GPIOD_PCLK_EN()		 (RCC -> AHB1ENR|= (1<<3))
#define GPIOE_PCLK_EN()		 (RCC -> AHB1ENR|= (1<<4))
#define GPIOF_PCLK_EN()		 (RCC -> AHB1ENR|= (1<<5))
#define GPIOG_PCLK_EN()		 (RCC -> AHB1ENR|= (1<<6))
#define GPIOH_PCLK_EN()		 (RCC -> AHB1ENR|= (1<<7))


/* I2C Clock Enable Macros */

#define I2C1_PCLK_EN()		 (RCC-> APB1ENR|= (1<<21))
#define I2C2_PCLK_EN()		 (RCC-> APB1ENR|= (1<<22))
#define I2C3_PCLK_EN()		 (RCC-> APB1ENR|= (1<<23))

/* SPI Clock Enable Clock Macros */

#define SPI1_PCLK_EN()		 (RCC-> APB2ENR|= (1<<12))
#define SPI2_PCLK_EN()		 (RCC-> APB1ENR|= (1<<14))
#define SPI3_PCLK_EN()		 (RCC-> APB1ENR|= (1<<15))
#define SPI4_PCLK_EN()		 (RCC-> APB2ENR|= (1<<13))

/* Clock Enable Macros for USARTx Peripherals */

#define USART1_PCLK_EN()		 (RCC-> APB2ENR|= (1<<4))
#define USART6_PCLK_EN()		 (RCC-> APB2ENR|= (1<<5))
#define USART2_PCLK_EN()		 (RCC-> APB1ENR|= (1<<17))
#define USART3_PCLK_EN()		 (RCC-> APB1ENR|= (1<<18))
#define UART4_PCLK_EN()		 (RCC-> APB1ENR|= (1<<19))
#define UART5_PCLK_EN()		 (RCC-> APB1ENR|= (1<<20))

/* Clock Enable Macros for SYSCFG Peripherals */

#define SYSCFG_PCLK_EN()		 (RCC-> APB2ENR|= (1<<14))



/* Clock Enable Macros for ADC Peripherals */
#define ADC1_PCLK_EN()		 (RCC-> APB2ENR|= (1<<8))
#define ADC2_PCLK_EN()		 (RCC-> APB2ENR|= (1<<9))
#define ADC3_PCLK_EN()		 (RCC-> APB2ENR|= (1<<10))

/* CLOCK DISABLE MACROS*/

/* Clock disable macros for GPIOx Peripherals*/
#define GPIOA_PCLK_DI()		 (RCC-> AHB1ENR &= ~(1<<0))
#define GPIOB_PCLK_DI()		 (RCC-> AHB1ENR &= ~(1<<1))
#define GPIOC_PCLK_DI()		 (RCC-> AHB1ENR &= ~(1<<2))
#define GPIOD_PCLK_DI()		 (RCC-> AHB1ENR &= ~(1<<3))
#define GPIOE_PCLK_DI()		 (RCC-> AHB1ENR &= ~(1<<4))
#define GPIOF_PCLK_DI()		 (RCC-> AHB1ENR &= ~(1<<5))
#define GPIOG_PCLK_DI()		 (RCC-> AHB1ENR &= ~(1<<6))
#define GPIOH_PCLK_DI()		 (RCC-> AHB1ENR &= ~(1<<7))

/* Clock disable macros for I2Cx Peripherals*/

#define I2C1_PCLK_DI()		 (RCC-> APB1ENR &= ~(1<<21))
#define I2C2_PCLK_DI()		 (RCC-> APB1ENR &= ~(1<<22))
#define I2C3_PCLK_DI()		 (RCC-> APB1ENR &= ~(1<<23))

/* Clock disable macros for SPIx Peripherals*/

#define SPI1_PCLK_DI()		 (RCC-> APB2ENR &= ~(1<<12))
#define SPI2_PCLK_DI()		 (RCC-> APB1ENR &= ~(1<<14))
#define SPI3_PCLK_DI()		 (RCC-> APB1ENR &= ~(1<<15))
#define SPI4_PCLK_DI()		 (RCC-> APB2ENR &= ~(1<<13))

/* Clock disable macros for USARTx Peripherals*/

#define USART1_PCLK_DI()		 (RCC-> APB2ENR &= ~(1<<4))
#define USART6_PCLK_DI()		 (RCC-> APB2ENR &= ~(1<<5))
#define USART2_PCLK_DI()		 (RCC-> APB1ENR &= ~(1<<17))
#define USART3_PCLK_DI()		 (RCC-> APB1ENR &= ~(1<<18))
#define UART4_PCLK_DI()		 (RCC-> APB1ENR &= ~(1<<19))
#define UART5_PCLK_DI()		 (RCC-> APB1ENR &= ~(1<<20))
/* Clock disable macros for ADCx Peripherals*/

/* Some Generic Macros */

#define ENABLE				 1
#define DISABLE 			 0
#define SET 				 ENABLE
#define RESET 				 DISABLE
#define GPIO_PIN_SET         ENABLE
#define GPIO_PIN_RESET       DISABLE
#define FLAG_RESET			 RESET
#define FLAG_SET			 SET

/*Macros to reset GPIOX registers*/


#define GPIOA_REG_RESET()					do{(RCC-> AHB1RSTR |= (1<<0)); (RCC-> AHB1RSTR &= ~(1<<0)); }while(0)
#define GPIOB_REG_RESET()					do{(RCC-> AHB1RSTR |= (1<<1)); (RCC-> AHB1RSTR &= ~(1<<1)); }while(0)
#define GPIOC_REG_RESET()					do{(RCC-> AHB1RSTR |= (1<<2)); (RCC-> AHB1RSTR &= ~(1<<2)); }while(0)
#define GPIOD_REG_RESET()					do{(RCC-> AHB1RSTR |= (1<<3)); (RCC-> AHB1RSTR &= ~(1<<3)); }while(0)
#define GPIOE_REG_RESET()					do{(RCC-> AHB1RSTR |= (1<<4)); (RCC-> AHB1RSTR &= ~(1<<4)); }while(0)
#define GPIOF_REG_RESET()					do{(RCC-> AHB1RSTR |= (1<<5)); (RCC-> AHB1RSTR &= ~(1<<5)); }while(0)
#define GPIOG_REG_RESET()					do{(RCC-> AHB1RSTR |= (1<<6)); (RCC-> AHB1RSTR &= ~(1<<6)); }while(0)
#define GPIOH_REG_RESET()					do{(RCC-> AHB1RSTR |= (1<<7)); (RCC-> AHB1RSTR &= ~(1<<7)); }while(0)

/*
 * BASEADDRESS to PORTCODE for interrupts
 */
#define BASEADDR_TO_PORTCODE(x)				((x == GPIOA) ? 0 :\
											(x == GPIOB) ? 1 :\
											(x == GPIOC) ? 2 :\
											(x == GPIOD) ? 3 :\
											(x == GPIOE) ? 4 :\
											(x == GPIOF) ? 5 :\
											(x == GPIOG) ? 6 :0 ) //Google Ternary Operator

/*
 * IRQ (Interrupt Request) Numbers Macros
 */

#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI5_10		40
#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2			36
#define IRQ_NO_SPI3			51


/*
 * All possible Priority levels for interrupts
 */

#define NVIC_IRQ_PRI0		0
#define NVIC_IRQ_PRI1		1
#define NVIC_IRQ_PRI2		2
#define NVIC_IRQ_PRI3		3
#define NVIC_IRQ_PRI4		4
#define NVIC_IRQ_PRI5		5
#define NVIC_IRQ_PRI6		6
#define NVIC_IRQ_PRI7		7
#define NVIC_IRQ_PRI8		8
#define NVIC_IRQ_PRI9		9
#define NVIC_IRQ_PRI10		10
#define NVIC_IRQ_PRI11		11
#define NVIC_IRQ_PRI12		12
#define NVIC_IRQ_PRI13		13
#define NVIC_IRQ_PRI14		14
#define NVIC_IRQ_PRI15		15

/**********************************************************************
 * Bit Position definition Macros for SPI Peripheral
 *********************************************************************/

#define SPI_CR1_CPHA		0
#define SPI_CR1_CPOL		1
#define SPI_CR1_MSTR		2
#define SPI_CR1_BR			3
#define SPI_CR1_SPE			6
#define SPI_CR1_LSBFIRST    7
#define SPI_CR1_SSI			8
#define SPI_CR1_SSM			9
#define SPI_CR1_RXONLY      10
#define SPI_CR1_DFF			11
#define SPI_CR1_CRCNEXT		12
#define SPI_CR1_CRCEN		13
#define SPI_CR1_BIDIOE		14
#define SPI_CR1_BIDIMODE	15

#define SPI_CR2_RXDMAEN		0
#define SPI_CR2_TXDMAEN		1
#define SPI_CR2_SSOE		2
#define SPI_CR2_FRF			4
#define SPI_CR2_ERRIE		5
#define SPI_CR2_RXNEIE		6
#define SPI_CR2_TXEIE		7

#define SPI_SR_RXNE			0
#define SPI_SR_TXE			1
#define SPI_SR_CHSIDE		2
#define SPI_SR_UDR			3
#define SPI_SR_CRCERR		4
#define SPI_SR_MODF			5
#define SPI_SR_OVR			6
#define SPI_SR_BSY			7
#define SPI_SR_FRE			8

/**********************************************************************
 * Bit Position definition Macros for USART Peripheral
 *********************************************************************/

#define USART_CR1_SBK		0
#define USART_CR1_RWU		1
#define USART_CR1_RE		2
#define USART_CR1_TE		3
#define USART_CR1_IDLEIE	4
#define USART_CR1_RXNEIE	5
#define USART_CR1_TCIE		6
#define USART_CR1_TXEIE		7
#define USART_CR1_PEIE		8
#define USART_CR1_PS		9
#define USART_CR1_PCE		10
#define USART_CR1_WAKE		11
#define USART_CR1_M			12
#define USART_CR1_UE		13
#define USART_CR1_OVER8		15

#define USART_CR2_ADD		0
#define USART_CR2_LBDL		5
#define USART_CR2_LBDIE		6
#define USART_CR2_LBCL		8
#define USART_CR2_CPHA		9
#define USART_CR2_CPOL		10
#define USART_CR2_CLKEN		11
#define USART_CR2_STOP		12
#define USART_CR2_LINEN		14

#define USART_CR3_EIE		0
#define USART_CR3_IREN		1
#define USART_CR3_IRLP		2
#define USART_CR3_HDSEL		3
#define USART_CR3_NACK		4
#define USART_CR3_SCEN		5
#define USART_CR3_DMAR		6
#define USART_CR3_DMAT		7
#define USART_CR3_RTSE		8
#define USART_CR3_CTSE		9
#define USART_CR3_CTSIE		10
#define USART_CR3_ONEBIT	11

#define USART_SR_PE			0
#define USART_SR_FE			1
#define USART_SR_NF			2
#define USART_SR_ORE		3
#define USART_SR_IDLE		4
#define USART_SR_RXNE		5
#define USART_SR_TC			6
#define USART_SR_TXE		7
#define USART_SR_LBD		8
#define USART_SR_CTS		9

#include "stm32f446xx_usart_driver.h"
#include "stm32f446xx_gpio_driver.h"
#include "stm32f446xx_spi_driver.h"
#include "stm32f446xx_rcc_driver.h"

#endif /* INC_STM32F446XX_H_ */
