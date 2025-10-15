#ifndef INC_STM32F411XX_H_
#define INC_STM32F411XX_H_

#include <stdint.h>
#include <string.h>

#define __vo volatile  //we can use this notation for volatile in the program

#define __weak  __attribute__((weak))

//ARM Cortex Mx Processor NVIC ISERx Register Addresses
#define NVIC_ISER0                        ((volatile uint32_t*)0xE000E100)
#define NVIC_ISER1                        ((volatile uint32_t*)0xE000E104)
#define NVIC_ISER2                        ((volatile uint32_t*)0xE000E108)
#define NVIC_ISER3                        ((volatile uint32_t*)0xE000E10C)

//ARM Cortex Mx Processor NVIC ICERx Register Addresses
#define NVIC_ICER0                        ((volatile uint32_t*)0XE000E180)
#define NVIC_ICER1                        ((volatile uint32_t*)0xE000E184)
#define NVIC_ICER2                        ((volatile uint32_t*)0xE000E188)
#define NVIC_ICER3                        ((volatile uint32_t*)0xE000E18C)

//ARM Cortex Mx Processor NVIC Priority  Register Addresses
#define NVIC_PR_BASE_ADDR                 ((volatile uint32_t*)0xE000E400)

#define PR_BITS_IMPLEMENTED                4

//Base Addresses of Memory
#define    FLASH_BASEADDR                     0x08000000U
#define    SRAM_BASEADDR                      0x20000000U
#define    ROM_BASEADDR                       0x1FFF0000U

//AHBx and APBx Bus Peripheral Base Addresses
#define    PERIPH_BASEADDR                    0x40000000U
#define    APB1PERIPH_BASEADDR                0x40000000U
#define    APB2PERIPH_BASEADDR                0x40010000U
#define    AHB1PERIPH_BASEADDR                0x40020000U
#define    AHB2PERIPH_BASEADDR                0x50000000U

//Base Addresses of AHB1 Peripherals
#define    GPIOA_BASEADDR                     0x40020000U
#define    GPIOB_BASEADDR                     0x40020400U
#define    GPIOC_BASEADDR                     0x40020800U
#define    GPIOD_BASEADDR                     0x40020C00U
#define    GPIOE_BASEADDR                     0x40021000U
#define    GPIOH_BASEADDR                     0x40021C00U

#define    RCC_BASEADDR                       0x40023800U

#define    SYSCFG_BASEADDR                    0x40013800U

#define    EXTI_BASEADDR                      0x40013C00U

//Base Addresses of APB1 Peripherals
#define    SPI2_BASEADDR                      0x40003800U
#define    SPI3_BASEADDR                      0x40003C00U

#define    USART2_BASEADDR                    0x40004400U

#define    I2C1_BASEADDR                      0x40005400U
#define    I2C2_BASEADDR                      0x40005800U
#define    I2C3_BASEADDR                      0x40005C00U

//Base Addresses of APB2 Peripherals
#define    SPI5_BASEADDR                      0x40015000U
#define    SPI4_BASEADDR                      0x40013400U
#define    SPI1_BASEADDR                      0x40013000U

#define    USART6_BASEADDR                    0x40011400U
#define    USART1_BASEADDR                    0x40011000U

//Peripheral Register Structure  for RCC
typedef struct
{
	volatile  uint32_t CR;
	volatile  uint32_t PLLCFGR;
	volatile  uint32_t CFGR;
	volatile  uint32_t CIR;
	volatile  uint32_t AHB1RSTR;
	volatile  uint32_t AHB2RSTR;
	  uint32_t RESERVED1[2];
	volatile  uint32_t APB1RSTR;
	volatile  uint32_t APB2RSTR;
	 uint32_t RESERVED2[2];
	volatile  uint32_t AHB1ENR;
	volatile  uint32_t AHB2ENR;
	  uint32_t RESERVED3[2];
    volatile  uint32_t APB1ENR;
    volatile  uint32_t APB2ENR;
      uint32_t RESERVED4[2];
    volatile  uint32_t AHB1LPENR;
    volatile  uint32_t AHB2LPENR;
      uint32_t RESERVED5[2];
    volatile  uint32_t APB1LPENR;
    volatile  uint32_t APB2LPENR;
     uint32_t RESERVED6[2];
    volatile  uint32_t BDCR;
    volatile  uint32_t CSR;
    uint32_t RESERVED7[2];
    volatile  uint32_t SSCGR;
    volatile  uint32_t PLLI2SCFGR;
    volatile  uint32_t DCKCFGR;
}RCC_RegDef_t;

//Peripheral Register Structure  for GPIO
typedef struct
{
	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDR;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint32_t AFR[2];
}GPIO_RegDef_t;

//Peripheral Register Structure for EXTI
typedef struct
{
	volatile uint32_t  IMR;
	volatile uint32_t  EMR;
	volatile uint32_t  RTSR;
	volatile uint32_t  FTSR;
	volatile uint32_t  SWIER;
	volatile uint32_t  PR;
}EXTI_RegDef_t;


//Peripheral register Structure for SYSCFG
typedef struct
{
	volatile uint32_t  MEMRMP;
	volatile uint32_t  PMC;
	volatile uint32_t  EXTICR[4];
	volatile uint32_t  RESERVED8[2];
	volatile uint32_t  CMPCR;
}SYSCFG_RegDef_t;


//Peripheral register Structure for SPI
typedef struct
{
	volatile uint32_t SPI_CR1;
	volatile uint32_t SPI_CR2;
	volatile uint32_t SPI_SR;
	volatile uint32_t SPI_DR;
	volatile uint32_t SPI_CRCPR;
	volatile uint32_t SPI_RXCRCR;
	volatile uint32_t SPI_TXCRCR;
	volatile uint32_t SPI_I2SCFGR;
	volatile uint32_t SPI_I2SPR;
}SPI_RegDef_t;



//Peripheral register Structure for SPI
typedef struct
{
    volatile uint32_t I2C_CR1;
    volatile uint32_t I2C_CR2;
    volatile uint32_t I2C_OAR1;
    volatile uint32_t I2C_OAR2;
    volatile uint32_t I2C_DR;
    volatile uint32_t I2C_SR1;
    volatile uint32_t I2C_SR2;
    volatile uint32_t I2C_CCR;
    volatile uint32_t I2C_TRISE;
    volatile uint32_t I2C_FLTR;
}I2C_RegDef_t;



//Macros with peripherals typecasted to corresponding RegDef_t
#define GPIOA       ((GPIO_RegDef_t*)(GPIOA_BASEADDR))
#define GPIOB       ((GPIO_RegDef_t*)(GPIOB_BASEADDR))
#define GPIOC       ((GPIO_RegDef_t*)(GPIOC_BASEADDR))
#define GPIOD       ((GPIO_RegDef_t*)(GPIOD_BASEADDR))
#define GPIOE       ((GPIO_RegDef_t*)(GPIOE_BASEADDR))
#define GPIOH       ((GPIO_RegDef_t*)(GPIOH_BASEADDR))

#define RCC         ((RCC_RegDef_t*)(RCC_BASEADDR))

#define EXTI        ((EXTI_RegDef_t*)(EXTI_BASEADDR))

#define SYSCFG      ((SYSCFG_RegDef_t*)(SYSCFG_BASEADDR))

#define SPI1        ((SPI_RegDef_t *)(SPI1_BASEADDR))
#define SPI2        ((SPI_RegDef_t *)(SPI2_BASEADDR))
#define SPI3        ((SPI_RegDef_t *)(SPI3_BASEADDR))
#define SPI4        ((SPI_RegDef_t *)(SPI4_BASEADDR))
#define SPI5        ((SPI_RegDef_t *)(SPI5_BASEADDR))


#define I2C1        ((I2C_RegDef_t *)(I2C1_BASEADDR))
#define I2C2        ((I2C_RegDef_t *)(I2C2_BASEADDR))
#define I2C3        ((I2C_RegDef_t *)(I2C3_BASEADDR))

//Clock Enable Macros for GPIO Peripherals
#define GPIOA_PCLK_EN()    (RCC->AHB1ENR |= (1<<0))
#define GPIOB_PCLK_EN()    (RCC->AHB1ENR |= (1<<1))
#define GPIOC_PCLK_EN()    (RCC->AHB1ENR |= (1<<2))
#define GPIOD_PCLK_EN()    (RCC->AHB1ENR |= (1<<3))
#define GPIOE_PCLK_EN()    (RCC->AHB1ENR |= (1<<4))
#define GPIOH_PCLK_EN()    (RCC->AHB1ENR |= (1<<7))

//Clock Enable Macros for I2C Peripherals
#define I2C1_PCLK_EN()    (RCC->APB1ENR |= (1<<21))
#define I2C2_PCLK_EN()    (RCC->APB1ENR |= (1<<22))
#define I2C3_PCLK_EN()    (RCC->APB1ENR |= (1<<23))

//Clock Enable Macros for SPI Peripherals
#define SPI1_PCLK_EN()    (RCC->APB2ENR |= (1<<12))
#define SPI2_PCLK_EN()    (RCC->APB1ENR |= (1<<14))
#define SPI3_PCLK_EN()    (RCC->APB1ENR |= (1<<15))
#define SPI4_PCLK_EN()    (RCC->APB2ENR |= (1<<13))
#define SPI5_PCLK_EN()    (RCC->APB2ENR |= (1<<20))

//Clock Enable Macros for USART Peripherals
#define USART1_PCLK_EN()    (RCC->APB2ENR |= (1<<4))
#define USART2_PCLK_EN()    (RCC->APB1ENR |= (1<<17))
#define USART6_PCLK_EN()    (RCC->APB2ENR |= (1<<5))

//Clock Enable for SYSCFG Peripheral
#define SYSCFG_PCLK_EN()    (RCC->APB2ENR |= (1<<14))

//Clock Disable Macros for GPIO Peripherals
#define GPIOA_PCLK_DI()    (RCC->AHB1ENR &= ~(1<<0))
#define GPIOB_PCLK_DI()    (RCC->AHB1ENR &= ~(1<<1))
#define GPIOC_PCLK_DI()    (RCC->AHB1ENR &= ~(1<<2))
#define GPIOD_PCLK_DI()    (RCC->AHB1ENR &= ~(1<<3))
#define GPIOE_PCLK_DI()    (RCC->AHB1ENR &= ~(1<<4))
#define GPIOH_PCLK_DI()    (RCC->AHB1ENR &= ~(1<<7))

//Clock Disable Macros for I2C Peripherals
#define I2C1_PCLK_DI()    (RCC->APB1ENR &= ~(1<<21))
#define I2C2_PCLK_DI()    (RCC->APB1ENR &= ~(1<<22))
#define I2C3_PCLK_DI()    (RCC->APB1ENR &= ~(1<<23))

//Clock Disable Macros for SPI Peripherals
#define SPI1_PCLK_DI()    (RCC->APB2ENR &= ~(1<<12))
#define SPI2_PCLK_DI()    (RCC->APB1ENR &= ~(1<<14))
#define SPI3_PCLK_DI()    (RCC->APB1ENR &= ~(1<<15))
#define SPI4_PCLK_DI()    (RCC->APB2ENR &= ~(1<<13))
#define SPI5_PCLK_DI()    (RCC->APB2ENR &= ~(1<<20))

//Clock Disable Macros for USART Peripherals
#define USART1_PCLK_DI()  (RCC->APB2ENR &= ~(1<<4))
#define USART2_PCLK_DI()  (RCC->APB1ENR &= ~(1<<17))
#define USART6_PCLK_DI()  (RCC->APB2ENR &= ~(1<<5))

//Clock Disable for SYSCFG Peripheral
#define SYSCFG_PCLK_DI()   (RCC->APB2ENR &= ~(1<<14))

//Macros for reset GPIOx Peripherals
#define GPIOA_REG_RESET()  do{(RCC->AHB1RSTR |= (1<<0));(RCC->AHB1RSTR &= ~(1<<0));}while(0)
#define GPIOB_REG_RESET()  do{(RCC->AHB1RSTR |= (1<<1));(RCC->AHB1RSTR &= ~(1<<1));}while(0)
#define GPIOC_REG_RESET()  do{(RCC->AHB1RSTR |= (1<<2));(RCC->AHB1RSTR &= ~(1<<2));}while(0)
#define GPIOD_REG_RESET()  do{(RCC->AHB1RSTR |= (1<<3));(RCC->AHB1RSTR &= ~(1<<3));}while(0)
#define GPIOE_REG_RESET()  do{(RCC->AHB1RSTR |= (1<<4));(RCC->AHB1RSTR &= ~(1<<4));}while(0)
#define GPIOH_REG_RESET()  do{(RCC->AHB1RSTR |= (1<<7));(RCC->AHB1RSTR &= ~(1<<7));}while(0)

//return port codes for given gpiox base addresses in exti control register
#define GPIO_BASEADDR_TO_CODE(x)    ((x == GPIOA)?0:\
		                             (x == GPIOB)?1:\
				                     (x == GPIOC)?2:\
						             (x == GPIOD)?3:\
						     		 (x == GPIOE)?4:\
									 (x == GPIOH)?7:0)


//IRQ(Interrupt Request ) NUmbers  STM32F411xxMCU
#define IRQ_NO_EXTI0                6
#define IRQ_NO_EXTI1                7
#define IRQ_NO_EXTI2                8
#define IRQ_NO_EXTI3                9
#define IRQ_NO_EXTI4                10
#define IRQ_NO_EXTI9_5              23
#define IRQ_NO_EXTI15_10            40
#define IRQ_NO_SPI1                 35
#define IRQ_NO_SPI2                 36
#define IRQ_NO_SPI3                 51
#define IRQ_NO_SPI4                 84
#define IRQ_NO_SPI5                 85
#define IRQ_NO_I2C1_EV              31
#define IRQ_NO_I2C1_ER              32
#define IRQ_NO_I2C2_EV              33
#define IRQ_NO_I2C2_ER              34
#define IRQ_NO_I2C3_EV              72
#define IRQ_NO_I2C3_ER              73

#define NVIC_IRQ_PRI15              15


#define ENABLE              1
#define DISABLE             0
#define SET                 1
#define RESET               0
#define GPIO_PIN_SET        1
#define GPIO_PIN_RESET      0
#define FLAG_RESET          0
#define FLAG_SET            1





//Bit Position Definitions of SPI Peripheral

//Bit position definitions SPI_CR1
#define  SPI_CR1_CPHA                0
#define  SPI_CR1_CPOL                1
#define  SPI_CR1_MSTR                2
#define  SPI_CR1_BR                  3
#define  SPI_CR1_SPE                 6
#define  SPI_CR1_LSBFIRST            7
#define  SPI_CR1_SSI                 8
#define  SPI_CR1_SSM                 9
#define  SPI_CR1_RX_ONLY             10
#define  SPI_CR1_DFF                 11
#define  SPI_CR1_CRC_NEXT            12
#define  SPI_CR1_CRC_EN              13
#define  SPI_CR1_BIDIOE              14
#define  SPI_CR1_BIDIMODE            15


//Bit position definitions SPI_CR2
#define  SPI_CR2_RXDMAEN             0
#define  SPI_CR2_TXDMAEN             1
#define  SPI_CR2_SSOE                2
#define  SPI_CR2_FRF                 4
#define  SPI_CR2_ERRIE               5
#define  SPI_CR2_RXNEIE              6
#define  SPI_CR2_TXEIE              7


//Bit position definitions SPI_SR
#define  SPI_SR_RXNE                 0
#define  SPI_SR_TXE                  1
#define  SPI_SR_CHSIDE               2
#define  SPI_SR_UDR                  3
#define  SPI_SR_CRCERR               4
#define  SPI_SR_MODF                 5
#define  SPI_SR_OVR                  6
#define  SPI_SR_BSY                  7
#define  SPI_SR_FRE                  8


//Bit Position Definitions of I2C Peripheral

//Bit position definitions I2C_CR1
#define I2C_CR1_PE                   0
#define I2C_CR1_NOSTRETCH            7
#define I2C_CR1_START                8
#define I2C_CR1_STOP                 9
#define I2C_CR1_ACK                  10
#define I2C_CR1_SWRST                15

//Bit position definitions I2C_CR2
#define I2C_CR2_FREQ                 0
#define I2C_CR2_ITERREN              8
#define I2C_CR2_ITEVTEN              9
#define I2C_CR2_ITBUFEN              10

//Bit position definitions I2C_OAR1
#define I2C_OAR1_ADD0                0
#define I2C_OAR1_ADD71               1
#define I2C_OAR1_ADD98               8
#define I2C_OAR1_ADD_MODE            15

//Bit position definitions I2C_SR1
#define I2C_SR1_SB                   0
#define I2C_SR1_ADD                  1
#define I2C_SR1_BTF                  2
#define I2C_SR1_ADD10                3
#define I2C_SR1_STOPF                4
#define I2C_SR1_RXNE                 6
#define I2C_SR1_TXE                  7
#define I2C_SR1_BERR                 8
#define I2C_SR1_ARLO                 9
#define I2C_SR1_AF                   10
#define I2C_SR1_OVR                  11
#define I2C_SR1_TIMEOUT              14

//Bit position definitions I2C_SR2
#define I2C_SR2_MSL                  0
#define I2C_SR2_BUSY                 1
#define I2C_SR2_TRA                  2
#define I2C_SR2_GENCALL              4
#define I2C_SR2_DUALF                7

//Bit position definitions I2C_CCR
#define I2C_CCR_CCR                  0
#define I2C_CCR_DUTY                 14
#define I2C_CCR_FS                   15





















#include "stm32f411xx_spi_driver.h"







#endif /* INC_STM32F411XX_H_ */
