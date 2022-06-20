/*
 * stm32f401re.h
 *
 *  Created on: 05.07.2021
 *      Author: maxim
 */

#ifndef DRIVERS_INC_STM32F401RE_H_
#define DRIVERS_INC_STM32F401RE_H_

#include<stdint.h>

/**********************START:Processor Specific Details**********************
 * ARM Crotex Mx Processor NVIC ISERx register addresses
 */

#define NVIC_ISER0          ( (volatile uint32_t*)0xE000E100 )
#define NVIC_ISER1          ( (volatile uint32_t*)0xE000E104 )
#define NVIC_ISER2          ( (volatile uint32_t*)0xE000E108 )
#define NVIC_ISER3          ( (volatile uint32_t*)0xE000E10c )


/*
 * ARM Cortex Mx Processor NVIC ICERx register Addresses
 */
#define NVIC_ICER0 			((volatile uint32_t*)0XE000E180)
#define NVIC_ICER1			((volatile uint32_t*)0XE000E184)
#define NVIC_ICER2  		((volatile uint32_t*)0XE000E188)
#define NVIC_ICER3			((volatile uint32_t*)0XE000E18C)


/*
 * ARM Cortex Mx Processor Priority Register Address Calculation
 */
#define NVIC_PR_BASE_ADDR 	((volatile uint32_t*)0xE000E400)

/*
 * ARM Cortex Mx Processor number of priority bits implemented in Priority Register
 */
#define NO_PR_BITS_IMPLEMENTED  4










#define ENABLE 			1
#define DISABLE 		0
#define SET 			ENABLE
#define RESET 			DISABLE
#define GPIO_PIN_SET 	SET
#define GPIO_PIN_RESET	RESET

/*
 * base addresses of Flash and SRAM memories
 */

#define FLASH_BASEADDR						0x08000000U
#define SRAM1_BASEADDR						0x20000000U
#define SRAM 								SRAM1_BASEADDR

/*
 * AHBx and APBx Bus Peripheral base addresses
 */

#define PERIPH_BASEADDR						0x40000000U
#define APB1PERIPH_BASEADDR					PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR					0x40010000U
#define AHB1PERIPH_BASEADDR					0x40020000U
#define AHB2PERIPH_BASEADDR					0x50000000U

/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 * TODO: Complete for all other peripherals
 */

#define GPIOA_BASEADDR						(AHB1PERIPH_BASEADDR	+ 0x0000UL)  // BASEADDR AHB1 bus + OFFSET
#define GPIOB_BASEADDR						(AHB1PERIPH_BASEADDR	+ 0x0400UL)
#define GPIOC_BASEADDR						(AHB1PERIPH_BASEADDR + 0x0800UL)
#define GPIOD_BASEADDR						(AHB1PERIPH_BASEADDR + 0x0C00UL)
#define GPIOE_BASEADDR						(AHB1PERIPH_BASEADDR + 0x1000UL)
#define GPIOH_BASEADDR						(AHB1PERIPH_BASEADDR + 0x1C00UL)
#define RCC_BASEADDR						(AHB1PERIPH_BASEADDR + 0x3800UL)

/*
 * Base addresses of peripherals which are hanging on APB1 bus
 * TODO: Complete for all other peripherals
 */

#define I2C1_BASEADDR						(APB1PERIPH_BASEADDR + 0x5400UL)
#define I2C2_BASEADDR						(APB1PERIPH_BASEADDR + 0x5800UL)
#define I2C3_BASEADDR						(APB1PERIPH_BASEADDR + 0x5C00UL)

#define SPI2_BASEADDR						(APB1PERIPH_BASEADDR + 0x3800UL)
#define SPI3_BASEADDR						(APB1PERIPH_BASEADDR + 0x3C00UL)

#define USART2_BASEADDR						(APB1PERIPH_BASEADDR + 0x4400UL)

/*
 * Base addresses of peripherals which are hanging on APB2 bus
 * TODO: Complete for all other peripherals
 */

#define EXTI_BASEADDR						(APB2PERIPH_BASEADDR + 0x3C00UL)

#define SPI1_BASEADDR						(APB2PERIPH_BASEADDR + 0x3000UL)
#define SPI4_BASEADDR						(APB2PERIPH_BASEADDR + 0x3400UL)

#define SYSCFG_BASEADDR						(APB2PERIPH_BASEADDR + 0x3800UL)

#define USART1_BASEADDR						(APB2PERIPH_BASEADDR + 0x1000UL)
#define USART6_BASEADDR						(APB2PERIPH_BASEADDR + 0x1400UL)

/********************************************peripheral register definition structures********************************************/

/*
 * GPIO
 */

typedef struct
{
	volatile uint32_t MODER;				/*!< GPIO port mode register                                                                         Address offset : 0x00      */
	volatile uint32_t OTYPER;				/*!< GPIO port output type register                                                                  Address offset : 0x04      */
	volatile uint32_t OSPEEDR;				/*!< GPIO port output speed register                                                                 Address offset : 0x08      */
	volatile uint32_t PUPDR;				/*!< GPIO port pull-up/pull-down register                                                            Address offset : 0x0C      */
	volatile uint32_t IDR;					/*!< GPIO port input data register                                                                   Address offset : 0x10      */
	volatile uint32_t ODR;					/*!< GPIO port output data register                                                                  Address offset : 0x14      */
	volatile uint32_t BSRR;					/*!< GPIO port bit set/reset register                                                                Address offset : 0x18      */
	volatile uint32_t LCKR;					/*!< GPIO port configuration lock register                                                           Address offset : 0x1C      */
	volatile uint32_t AFR[2];				/*!< AFR[0] : GPIO alternate function low register AFR[1] : GPIO alternate function high register    Address offset : 0x20-0x24 */
}GPIO_RegDef_t;

/*
 * peripheral register definition structure for RCC
 */

typedef struct
{
	volatile uint32_t CR;					/*!< RCC clock control register                                                                        Address offset : 0x00*/
	volatile uint32_t PLLCFGR;				/*!< RCC PLL configuration register                                                                    Address offset : 0x04*/
	volatile uint32_t CFGR;					/*!< RCC clock configuration register                                                                  Address offset : 0x08*/
	volatile uint32_t CIR;					/*!< RCC clock interrupt register                                                                      Address offset : 0x0C*/
	volatile uint32_t AHB1RSTR;				/*!< RCC AHB1 peripheral reset register                                                                Address offset : 0x10*/
	volatile uint32_t AHB2RSTR;				/*!< RCC AHB2 peripheral reset register                                                                Address offset : 0x14*/
	volatile uint32_t RESERVED0;			/*!< -                                                                           					   Address offset : 0x18*/
	volatile uint32_t RESERVED1;			/*!< -                                                                                                 Address offset : 0x1C*/
	volatile uint32_t APB1RSTR;				/*!< RCC APB1 peripheral reset register                                                                Address offset : 0x20*/
	volatile uint32_t APB2RSTR;				/*!< RCC APB2 peripheral reset register                                                                Address offset : 0x24*/
	volatile uint32_t RESERVED2;			/*!< -                                                                                                 Address offset : 0x28*/
	volatile uint32_t RESERVED3;			/*!< -                                                                                                 Address offset : 0x2C*/
	volatile uint32_t AHB1ENR;				/*!< RCC AHB1 peripheral clock enable register                                                         Address offset : 0x30*/
	volatile uint32_t AHB2ENR;				/*!< RCC AHB2 peripheral clock enable register                                                         Address offset : 0x34*/
	volatile uint32_t RESERVED4;			/*!< -                                                                                                 Address offset : 0x38*/
	volatile uint32_t RESERVED5;			/*!< -                                                                                                 Address offset : 0x3C*/
	volatile uint32_t APB1ENR;				/*!< RCC APB1 peripheral clock enable register                                                         Address offset : 0x40*/
	volatile uint32_t APB2ENR;				/*!< RCC APB2 peripheral clock enable register                                                         Address offset : 0x44*/
	volatile uint32_t RESERVED6;			/*!< -                                                                                                 Address offset : 0x48*/
	volatile uint32_t RESERVED7;			/*!< -                                                                                                 Address offset : 0x4C*/
	volatile uint32_t AHB1LPENR;			/*!< RCC AHB1 peripheral clock enable in low power mode register                                       Address offset : 0x50*/
	volatile uint32_t AHB2LPENR;			/*!< RCC AHB2 peripheral clock enable in low power mode register                                       Address offset : 0x54*/
	volatile uint32_t RESERVED8;			/*!< -                                                                                                 Address offset : 0x58*/
	volatile uint32_t RESERVED9;			/*!< -                                                                                                 Address offset : 0x5C*/
	volatile uint32_t APB1LPENR;			/*!< RCC APB1 peripheral clock enable in low power mode register                                       Address offset : 0x60*/
	volatile uint32_t APB2LPENR;			/*!< RCC APB2 peripheral clock enabled in low power mode register                                      Address offset : 0x64*/
	volatile uint32_t RESERVED10;			/*!< -                                                                                                 Address offset : 0x68*/
	volatile uint32_t RESERVED11;			/*!< -                                                                                                 Address offset : 0x6C*/
	volatile uint32_t BDCR;					/*!< RCC Backup domain control register                                                                Address offset : 0x70*/
	volatile uint32_t CSR;					/*!< RCC clock control & status register                                                               Address offset : 0x74*/
	volatile uint32_t RESERVED12;			/*!< -                                                                                                 Address offset : 0x78*/
	volatile uint32_t RESERVED13;			/*!< -                                                                                                 Address offset : 0x7C*/
	volatile uint32_t SSCGR;				/*!< RCC spread spectrum clock generation register                                                     Address offset : 0x80*/
	volatile uint32_t PLLI2SCFGR;			/*!< RCC PLLI2S configuration register                                                                 Address offset : 0x84*/
	volatile uint32_t DCKCFGR;				/*!< RCC Dedicated Clocks Configuration Register                                                       Address offset : 0x8C*/

}RCC_RegDef_t;

/*
 * peripheral register definition structure for RCC
 */

typedef struct
{
	volatile uint32_t IMR;					/*!< Interrupt mask register                           		Address offset : 0x00*/
	volatile uint32_t EMR;					/*!< Event mask register                           			Address offset : 0x04*/
	volatile uint32_t RTSR;					/*!< Rising trigger selection register                      Address offset : 0x08*/
	volatile uint32_t FTSR;					/*!< Falling trigger selection register                     Address offset : 0x0C*/
	volatile uint32_t SWIER;				/*!< Software interrupt event register                      Address offset : 0x10*/
	volatile uint32_t PR;					/*!< Pending register                           			Address offset : 0x14*/

}EXTI_RegDef_t;

/*
 * peripheral register definition structure for SYSCFG
 */

typedef struct
{
	volatile uint32_t MEMRMP;					/*!< SYSCFG memory remap register                           Address offset : 0x00*/
	volatile uint32_t PMC;						/*!< SYSCFG peripheral mode configuration register      	Address offset : 0x04*/
	volatile uint32_t EXTICRx[4];				/*!< SYSCFG external interrupt configuration register 1     Address offset : 0x08-0x14*/
	volatile uint32_t CMPCR;					/*!< Compensation cell control register                     Address offset : 0x20*/

}SYSCFG_RegDef_t;

/*
 * peripheral definitions ( Peripheral base addresses typecasted to xxx_RegDef_t)
 */

#define GPIOA			((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB			((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC			((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD			((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE			((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOH			((GPIO_RegDef_t*)GPIOH_BASEADDR)

#define RCC				((RCC_RegDef_t*)RCC_BASEADDR)

#define SYSCFG			((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)
#define EXTI			((EXTI_RegDef_t*)EXTI_BASEADDR)

/*
 * Clock Enable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()	( RCC->AHB1ENR |= (0x1UL << 0U))
#define GPIOB_PCLK_EN()	( RCC->AHB1ENR |= (1 << 1) )
#define GPIOC_PCLK_EN()	( RCC->AHB1ENR |= (1 << 2) )
#define GPIOD_PCLK_EN()	( RCC->AHB1ENR |= (1 << 3) )
#define GPIOE_PCLK_EN()	( RCC->AHB1ENR |= (1 << 4) )
#define GPIOH_PCLK_EN()	( RCC->AHB1ENR |= (1 << 7) )

/*
 * Clock Disable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_DI()	( RCC->AHB1ENR &= ~(1 << 0) )
#define GPIOB_PCLK_DI()	( RCC->AHB1ENR &= ~(1 << 1) )
#define GPIOC_PCLK_DI()	( RCC->AHB1ENR &= ~(1 << 2) )
#define GPIOD_PCLK_DI()	( RCC->AHB1ENR &= ~(1 << 3) )
#define GPIOE_PCLK_DI()	( RCC->AHB1ENR &= ~(1 << 4) )
#define GPIOH_PCLK_DI()	( RCC->AHB1ENR &= ~(1 << 7) )

/*
 * Clock Enable Macros for I2Cx peripherals
 */

#define I2C1_PCLK_EN()	( RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()	( RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()	( RCC->APB1ENR |= (1 << 23))

/*
 * Clock Disable Macros for I2Cx peripherals
 */

#define I2C1_PCLK_DI()	( RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()	( RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()	( RCC->APB1ENR &= ~(1 << 23))

/*
 * Clock Enable Macros for SPIx peripherals
 */

#define SPI1_PCLK_EN()	( RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()	( RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()	( RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN()	( RCC->APB1ENR |= (1 << 13))

/*
 * Clock Disable Macros for SPIx peripherals
 */

#define SPI1_PCLK_DI()	( RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()	( RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()	( RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DI()	( RCC->APB1ENR &= ~(1 << 13))

/*
 * Clock Enable Macros for USARTx peripherals
 */

#define USART1_PCLK_EN()( RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()( RCC->APB1ENR |= (1 << 17))
#define USART6_PCLK_EN()( RCC->APB2ENR |= (1 << 5))

/*
 * Clock Disable Macros for USARTx peripherals
 */

#define USART1_PCLK_DI()( RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI()( RCC->APB1ENR &= ~(1 << 17))
#define USART6_PCLK_DI()( RCC->APB2ENR &= ~(1 << 5))

/*
 * Clock Enable Macros for SYSCFGx peripherals
 */

#define SYSCFG_PCLK_EN()( RCC->APB2ENR |= (1 << 14))

/*
 * Clock Disable Macros for SYSCFGx peripherals
 */

#define SYSCFG_PCLK_DI()( RCC->APB2ENR &= ~(1 << 14))

/*
 * Macros to reset GPIOx peripherals
 */

#define GPIOA_REG_RESET()			do{ ( RCC->AHB1RSTR |= (1 << 0)); ( RCC->AHB1RSTR &= ~(1 << 0));} while(0)
#define GPIOB_REG_RESET()			do{ ( RCC->AHB1RSTR |= (1 << 1)); ( RCC->AHB1RSTR &= ~(1 << 1));} while(0)
#define GPIOC_REG_RESET()			do{ ( RCC->AHB1RSTR |= (1 << 2)); ( RCC->AHB1RSTR &= ~(1 << 2));} while(0)
#define GPIOD_REG_RESET()			do{ ( RCC->AHB1RSTR |= (1 << 3)); ( RCC->AHB1RSTR &= ~(1 << 3));} while(0)
#define GPIOE_REG_RESET()			do{ ( RCC->AHB1RSTR |= (1 << 4)); ( RCC->AHB1RSTR &= ~(1 << 4));} while(0)
#define GPIOH_REG_RESET()			do{ ( RCC->AHB1RSTR |= (1 << 7)); ( RCC->AHB1RSTR &= ~(1 << 7));} while(0)

/*
 * returns port code for given GPIOx base adress
 */
#define GPIO_BASEADDR_TO_CODE(x)		((x == GPIOA) ? 0 :\
										(x == GPIOB) ? 1 :\
										(x == GPIOC) ? 2 :\
										(x == GPIOD) ? 3 :\
										(x == GPIOE) ? 4 :\
										(x == GPIOH) ? 7 :\
										0)

/*
 * IRQ(Interrupt Request) Numbers of STM32F407x MCU
 * TODO: You may complete this list for other peripherals
 */

#define IRQ_NO_EXTI0 		6
#define IRQ_NO_EXTI1 		7
#define IRQ_NO_EXTI2 		8
#define IRQ_NO_EXTI3 		9
#define IRQ_NO_EXTI4 		10
#define IRQ_NO_EXTI9_5 		23
#define IRQ_NO_EXTI15_10 	40
#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2         36
#define IRQ_NO_SPI3         51
#define IRQ_NO_SPI4			84
#define IRQ_NO_I2C1_EV     	31
#define IRQ_NO_I2C1_ER     	32
#define IRQ_NO_USART1	    37
#define IRQ_NO_USART2	    38
#define IRQ_NO_USART6	    71

/*
 * macros for all the possible priority levels
 */

#define NVIC_IRQ_PRI0    0
#define NVIC_IRQ_PRI15    15


#include "gpio_driver.h"

#endif /* DRIVERS_INC_STM32F401RE_H_ */
