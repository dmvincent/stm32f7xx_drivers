/*
 * stm32f769xx.h
 *
 *  Created on: Jun 21, 2024
 *      Author: daniel
 */

#ifndef INC_STM32F769XX_H_
#define INC_STM32F769XX_H_

#include <stdint.h>

#define __vo  volatile

/*
 * Base Address of Flash and SRAM memories
 */
#define SRAM1_BASEADDR		  0x20020000U				// Base address for SRAM1
#define SRAM2_BASEADDR		  0x2007C000U				// Base address for SRAM2
#define FLASH_BASEADDR		  0x80000000U				// Base address for FLASH MEMORY
#define ROM_BASEADDR			  0x1FF00000U				// Base address for System Memory
#define SRAM							  SRAM1_BASEADDR		// SRAM Peripheral

/*
 * AHBx and APBx Bus Peripheral base addresses
 */
#define PERIPH_BASEADDR     0x40000000U       // Peripheral base address 
#define APB1_BASEADDR       0x40000000U       // APB1 Bus Peripheral base address       
#define APB2_BASEADDR       0x40010000U       // APB2 Bus Peripheral base address
#define AHB1_BASEADDR       0x40020000U       // AHB1 Bus Peripheral base address
#define AHB2_BASEADDR       0x50000000U       // AHB2 Bus Peripheral base address
#define AHB3_BASEADDR       0x60000000U       // AHB3 Bus Peripheral base address

/*
 * Base addresses of peripherals which are hangin on AHB1 bus
 * TODO: Complete for all other peripherals
 */
#define GPIOA_BASEADDR      (AHB1_BASEADDR + 0x0000)       // GPIOA Peripheral Base address
#define GPIOB_BASEADDR      (AHB1_BASEADDR + 0x0400)       // GPIOB Peripheral Base address
#define GPIOC_BASEADDR      (AHB1_BASEADDR + 0x0800)       // GPIOC Peripheral Base address
#define GPIOD_BASEADDR      (AHB1_BASEADDR + 0x0C00)       // GPIOD Peripheral Base address
#define GPIOE_BASEADDR      (AHB1_BASEADDR + 0x1000)       // GPIOE Peripheral Base address
#define GPIOF_BASEADDR      (AHB1_BASEADDR + 0x1400)       // GPIOF Peripheral Base address
#define GPIOG_BASEADDR      (AHB1_BASEADDR + 0x1800)       // GPIOG Peripheral Base address
#define GPIOH_BASEADDR      (AHB1_BASEADDR + 0x1C00)       // GPIOH Peripheral Base address
#define GPIOI_BASEADDR      (AHB1_BASEADDR + 0x2000)       // GPIOI Peripheral Base address
#define GPIOJ_BASEADDR      (AHB1_BASEADDR + 0x2400)       // GPIOJ Peripheral Base address
#define GPIOK_BASEADDR      (AHB1_BASEADDR + 0x2800)       // GPIOK Peripheral Base address
#define CRC_BASEADDR        (AHB1_BASEADDR + 0x3000)       // CRC Peripheral Base address
#define RCC_BASEADDR        (AHB1_BASEADDR + 0x3800)       // RCC Peripheral Base address

/*
 * Base addresses of peripherals which are hangin on APB1 bus
 */
#define TIM2_BASEADDR             (APB1_BASEADDR + 0x0000)       // TIM2 Peripheral Base address
#define TIM3_BASEADDR             (APB1_BASEADDR + 0x0400)       // TIM3 Peripheral Base address
#define TIM4_BASEADDR             (APB1_BASEADDR + 0x0800)       // TIM4 Peripheral Base address
#define TIM5_BASEADDR             (APB1_BASEADDR + 0x0C00)       // TIM5 Peripheral Base address
#define TIM6_BASEADDR             (APB1_BASEADDR + 0x1000)       // TIM6 Peripheral Base address
#define TIM7_BASEADDR             (APB1_BASEADDR + 0x1400)       // TIM7 Peripheral Base address
#define TIM12_BASEADDR            (APB1_BASEADDR + 0x1800)       // TIM12 Peripheral Base address
#define TIM13_BASEADDR            (APB1_BASEADDR + 0x1C00)       // TIM13 Peripheral Base address
#define TIM14_BASEADDR            (APB1_BASEADDR + 0x2000)       // TIM14 Peripheral Base address
#define LPTIM1_BASEADDR           (APB1_BASEADDR + 0x2400)       // LPTIM1 Peripheral Base address
#define RTCBKP_BASEADDR           (APB1_BASEADDR + 0x2800)       // RTCBKP Peripheral Base address
#define WWDG_BASEADDR             (APB1_BASEADDR + 0x2C00)       // WWDG Peripheral Base address
#define IWDG_BASEADDR             (APB1_BASEADDR + 0x3000)       // IWDG Peripheral Base address
#define CAN3_BASEADDR             (APB1_BASEADDR + 0x3400)       // CAN3 Peripheral Base address
#define SPI2_I2S2_BASEADDR        (APB1_BASEADDR + 0x3800)       // SPI2 / I2S2 Peripheral Base address
#define SPI3_I2S3_BASEADDR        (APB1_BASEADDR + 0x3C00)       // SPI3/ I2S3 Peripheral Base address
#define SPDIFRX_BASEADDR          (APB1_BASEADDR + 0x4000)       // SPDIFRX Peripheral Base address
#define USART2_BASEADDR           (APB1_BASEADDR + 0x4400)       // USART2 Peripheral Base address
#define USART3_BASEADDR           (APB1_BASEADDR + 0x4800)       // USART3 Peripheral Base address
#define UART4_BASEADDR            (APB1_BASEADDR + 0x4C00)       // UART4 Peripheral Base address
#define UART5_BASEADDR            (APB1_BASEADDR + 0x5000)       // UART5 Peripheral Base address
#define I2C1_BASEADDR             (APB1_BASEADDR + 0x5400)       // I2C1 Peripheral Base address
#define I2C2_BASEADDR             (APB1_BASEADDR + 0x5800)       // I2C2 Peripheral Base address
#define I2C3_BASEADDR             (APB1_BASEADDR + 0x5C00)       // I2C3 Peripheral Base address
#define I2C4_BASEADDR             (APB1_BASEADDR + 0x6000)       // I2C4 Peripheral Base address
#define CAN1_BASEADDR             (APB1_BASEADDR + 0x6400)       // CAN1 Peripheral Base address
#define CAN2_BASEADDR             (APB1_BASEADDR + 0x6800)       // CAN2 Peripheral Base address
#define HDMICEC_BASEADDR          (APB1_BASEADDR + 0x6C00)       // HDMICEC Peripheral Base address
#define PWR_BASEADDR              (APB1_BASEADDR + 0x7000)       // PWR Peripheral Base address
#define DAC_BASEADDR              (APB1_BASEADDR + 0x7400)       // DAC Peripheral Base address
#define UART7_BASEADDR            (APB1_BASEADDR + 0x7800)       // UART7 Peripheral Base address
#define UART8_BASEADDR            (APB1_BASEADDR + 0x7C00)       // UART8 Peripheral Base address

/*
 * Base addresses of peripherals which are hangin on APB2 bus
 */
#define TIM1_BASEADDR             (APB2_BASEADDR + 0x0000)       // TIM1 Peripheral Base address
#define TIM8_BASEADDR             (APB2_BASEADDR + 0x0400)       // TIM8 Peripheral Base address
#define USART1_BASEADDR           (APB2_BASEADDR + 0x1000)       // USART1 Peripheral Base address
#define USART6_BASEADDR           (APB2_BASEADDR + 0x1400)       // USART6 Peripheral Base address
#define SDMMC2_BASEADDR           (APB2_BASEADDR + 0x1C00)       // SDMMC2 Peripheral Base address
#define ADC1_BASEADDR             (APB2_BASEADDR + 0x2000)       // ADC1 Peripheral Base address
#define ADC2_BASEADDR             (APB2_BASEADDR + 0x2400)       // ADC2 Peripheral Base address
#define ADC3_BASEADDR             (APB2_BASEADDR + 0x2800)       // ADC3 Peripheral Base address
#define SDMMC1_BASEADDR           (APB2_BASEADDR + 0x2C00)       // SDMMC1 Peripheral Base address
#define SPI1_BASEADDR             (APB2_BASEADDR + 0x3000)       // SPI1 Peripheral Base address
#define SPI4_BASEADDR             (APB2_BASEADDR + 0x3400)       // SPI4 Peripheral Base address
#define SYSCFG_BASEADDR           (APB2_BASEADDR + 0x3800)       // SYSCFG Peripheral Base address
#define EXTI_BASEADDR             (APB2_BASEADDR + 0x3C00)       // EXTI Peripheral Base address
#define TIM9_BASEADDR             (APB2_BASEADDR + 0x4000)       // TIM9 Peripheral Base address
#define TIM10_BASEADDR            (APB2_BASEADDR + 0x4400)       // TIM10 Peripheral Base address
#define TIM11_BASEADDR            (APB2_BASEADDR + 0x4800)       // TIM11 Peripheral Base address
#define SPI5_BASEADDR             (APB2_BASEADDR + 0x5000)       // SPI5 Peripheral Base address
#define SPI6_BASEADDR             (APB2_BASEADDR + 0x5400)       // SPI6 Peripheral Base address
#define SAI1_BASEADDR             (APB2_BASEADDR + 0x5800)       // SAI1 Peripheral Base address
#define SAI2_BASEADDR             (APB2_BASEADDR + 0x5C00)       // SAI2 Peripheral Base address
#define LCDTFT_BASEADDR           (APB2_BASEADDR + 0x6800)       // LCDTFT Peripheral Base address
#define DSIHOST_BASEADDR          (APB2_BASEADDR + 0x6C00)       // DSIHOST Peripheral Base address
#define DFSDM1_BASEADDR           (APB2_BASEADDR + 0x7400)       // DFSDM1 Peripheral Base address
#define MDIOS_BASEADDR            (APB2_BASEADDR + 0x7800)       // MDIOS Peripheral Base address
                                                                 


/***************************** Peripheral Register Definition Structures ************************/
// GPIO
typedef struct {
  __vo uint32_t MODER;       // Configured mode for either input, output, alternate function, or analog mode
  __vo uint32_t OTYPER;      // Configures output type for putput push-pull or ouput open-drain
  __vo uint32_t OSPEEDR;     // Configures slew-rate for output pin
  __vo uint32_t PUPDR;       // Configures I/O pull-up or pull-down
  __vo uint32_t IDR;         // Read-only bits containing input value of corresponding I/O port
  __vo uint32_t ODR;         // Read/Write output data bits for corresponing I/O port
  __vo uint32_t BSRR;        // GPIO Port bit set/reset register
  __vo uint32_t LCKR;        // GPIO Port configureation lock register
  __vo uint32_t AFRL;        // GPIO Alternate function low register
  __vo uint32_t AFRH;        // GPIO Alternate function high register
} GPIO_RegDef_t;

// RCC
// TODO: Provide Comments describing each register
typedef struct {
  __vo uint32_t CR;
  __vo uint32_t PLLCFGR;
  __vo uint32_t CFGR;
  __vo uint32_t CIR;
  __vo uint32_t AHB1RSTR;
  __vo uint32_t AHB2RSTR;
  __vo uint32_t AHB3RSTR;
  uint32_t RESERVED1;
  __vo uint32_t APB1RSTR;
  __vo uint32_t APB2RSTR;
  uint32_t RESERVED2;
  uint32_t RESERVED3;
  __vo uint32_t AHB1ENR;
  __vo uint32_t AHB2ENR;
  __vo uint32_t AHB3ENR;
  uint32_t RESERVED4;
  __vo uint32_t APB1ENR;
  __vo uint32_t APB2ENR;
  uint32_t RESERVED5;
  uint32_t RESERVED6;
  __vo uint32_t AHB1LPENR;
  __vo uint32_t AHB2LPENR;
  __vo uint32_t AHB3LPENR;
  uint32_t RESERVED7;
  __vo uint32_t APB1LPENR;
  __vo uint32_t APB2LPENR;
  uint32_t RESERVED8;
  uint32_t RESERVED9;
  __vo uint32_t BDCR;
  __vo uint32_t CSR;
  uint32_t RESERVED10;
  uint32_t RESERVED11;
  __vo uint32_t SSCGR;
  __vo uint32_t PLLI2SCFGR;
  __vo uint32_t PLLSAICFGR;
  __vo uint32_t DCKCFGR1;
  __vo uint32_t DCKCFGR2;
} RCC_RegDef_t;

// TIM
typedef struct {
  __vo uint32_t CR1;
  __vo uint32_t CR2;
  __vo uint32_t SMCR;
  __vo uint32_t DIER;
  __vo uint32_t SR;
  __vo uint32_t EGR;
  __vo uint32_t CCMR1;
  __vo uint32_t CCMR2;
  __vo uint32_t CCER;
  __vo uint32_t CNT;
  __vo uint32_t PSC;
  __vo uint32_t ARR;
  uint32_t RESERVED0;
  __vo uint32_t CCR1;
  __vo uint32_t CCR2;
  __vo uint32_t CCR3;
  __vo uint32_t CCR4;
  uint32_t RESERVED1;
  __vo uint32_t DCR;
  __vo uint32_t TIM2OR;
  __vo uint32_t TIM5OR;

} TIM2thru5_RegDef_t;

/*
 * Peripheral definitions (Peripheral base addresses typecasted to xxx_RegDef_t)
 */
// GPIO Peripherals
#define GPIOA                     ((GPIO_RegDef_t*)GPIOA_BASEADDR)      // TODO: Add comments                                                                  
#define GPIOB                     ((GPIO_RegDef_t*)GPIOB_BASEADDR)      // TODO: Add comments                                                                  
#define GPIOC                     ((GPIO_RegDef_t*)GPIOC_BASEADDR)      // TODO: Add comments                                                                  
#define GPIOD                     ((GPIO_RegDef_t*)GPIOD_BASEADDR)      // TODO: Add comments                                                                  
#define GPIOE                     ((GPIO_RegDef_t*)GPIOE_BASEADDR)      // TODO: Add comments                                                                  
#define GPIOF                     ((GPIO_RegDef_t*)GPIOF_BASEADDR)      // TODO: Add comments                                                                  
#define GPIOG                     ((GPIO_RegDef_t*)GPIOG_BASEADDR)      // TODO: Add comments                                                                  
#define GPIOH                     ((GPIO_RegDef_t*)GPIOH_BASEADDR)      // TODO: Add comments                                                                  
#define GPIOI                     ((GPIO_RegDef_t*)GPIOI_BASEADDR)      // TODO: Add comments                                                                  
#define GPIOJ                     ((GPIO_RegDef_t*)GPIOJ_BASEADDR)      // TODO: Add comments                                                                  
#define GPIOK                     ((GPIO_RegDef_t*)GPIOK_BASEADDR)      // TODO: Add comments                                                                  
                                                                     
// RCC Peripheral
#define RCC                       ((RCC_RegDef_t*)RCC_BASEADDR)         // TODO: Add comments

// TIMx Peripherals
// TODO: TIM1
#define TIM2                      ((TIM2thru5_RegDef_t*)TIM2_BASEADDR)  // TODO: Add comments  
#define TIM3                      ((TIM2thru5_RegDef_t*)TIM3_BASEADDR)  // TODO: Add comments
#define TIM4                      ((TIM2thru5_RegDef_t*)TIM4_BASEADDR)  // TODO: Add comments
#define TIM5                      ((TIM2thru5_RegDef_t*)TIM5_BASEADDR)  // TODO: Add comments
// TODO: TIM6
// TODO: TIM7
// TODO: TIM8
// TODO: TIM9
// TODO: TIM10
// TODO: TIM11
// TODO: TIM12
// TODO: TIM13
// TODO: TIM14

/*
 * Clock Enable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_EN()           ( RCC->AHB1ENR |= ( 1 << 0) )
#define GPIOB_PCLK_EN()           ( RCC->AHB1ENR |= ( 1 << 1) )
#define GPIOC_PCLK_EN()           ( RCC->AHB1ENR |= ( 1 << 2) )
#define GPIOD_PCLK_EN()           ( RCC->AHB1ENR |= ( 1 << 3) )
#define GPIOE_PCLK_EN()           ( RCC->AHB1ENR |= ( 1 << 4) )
#define GPIOF_PCLK_EN()           ( RCC->AHB1ENR |= ( 1 << 5) )
#define GPIOG_PCLK_EN()           ( RCC->AHB1ENR |= ( 1 << 6) )
#define GPIOH_PCLK_EN()           ( RCC->AHB1ENR |= ( 1 << 7) )
#define GPIOI_PCLK_EN()           ( RCC->AHB1ENR |= ( 1 << 8) )
#define GPIOJ_PCLK_EN()           ( RCC->AHB1ENR |= ( 1 << 9) )
#define GPIOK_PCLK_EN()           ( RCC->AHB1ENR |= ( 1 << 10))

/*
 * Clock Enable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_EN()           ( RCC->APB1ENR |= ( 1 << 21) )
#define I2C2_PCLK_EN()           ( RCC->APB1ENR |= ( 1 << 22) )
#define I2C3_PCLK_EN()           ( RCC->APB1ENR |= ( 1 << 23) )
#define I2C4_PCLK_EN()           ( RCC->APB1ENR |= ( 1 << 24) )

/*
 * Clock Enable Macros for SPIx peripherals
 */

/*
 * Clock Enable Macros for USARTx peripherals
 */

/*
 * Clock Enable Macros for SYSCFG peripherals
 */

/*
 * Clock Enable Macros for TIMx peripherals
 */





/*
 * Clock Disable Macros for GPIOx peripherals
 */

/*
 * Clock Disable Macros for I2Cx peripherals
 */

/*
 * Clock Disable Macros for SPIx peripherals
 */

/*
 * Clock Disable Macros for USARTx peripherals
 */

/*
 * Clock Disable Macros for SYSCFG peripherals
 */

/*
 * Clock Disable Macros for TIMx peripherals
 */


#endif /* INC_STM32F769XX_H_ */
