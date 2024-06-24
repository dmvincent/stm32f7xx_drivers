/*
 * stm32f769xx_gpio.c
 *
 *  Created on: Jun 22, 2024
 *      Author: daniel
 */

#include "stm32f769xx_gpio.h"

/**********************************************************************
 *                      APIs supported by this driver
 **********************************************************************/

/*
 * Peripheral Clock Control
 */
/**********************************************************************
 * @fn                - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock fro the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi) {
  if(EnorDi == ENABLE) {
    if(pGPIOx == GPIOA) GPIOA_PCLK_EN();
    else if(pGPIOx == GPIOB) GPIOB_PCLK_EN();
    else if(pGPIOx == GPIOC) GPIOC_PCLK_EN();
    else if(pGPIOx == GPIOD) GPIOD_PCLK_EN();
    else if(pGPIOx == GPIOE) GPIOE_PCLK_EN();
    else if(pGPIOx == GPIOF) GPIOF_PCLK_EN();
    else if(pGPIOx == GPIOG) GPIOG_PCLK_EN();
    else if(pGPIOx == GPIOH) GPIOB_PCLK_EN();
    else if(pGPIOx == GPIOI) GPIOB_PCLK_EN();
    else if(pGPIOx == GPIOJ) GPIOB_PCLK_EN();
    else if(pGPIOx == GPIOK) GPIOB_PCLK_EN();
    else {
      printf("ERROR: Unrecognized Peripheral passed to GPIO_PeriClockControl()");
    }
  }
  else if(EnorDi == DISABLE) {
    if(pGPIOx == GPIOA) GPIOA_PCLK_DI();
    else if(pGPIOx == GPIOB) GPIOB_PCLK_DI();
    else if(pGPIOx == GPIOC) GPIOC_PCLK_DI();
    else if(pGPIOx == GPIOD) GPIOD_PCLK_DI();
    else if(pGPIOx == GPIOE) GPIOE_PCLK_DI();
    else if(pGPIOx == GPIOF) GPIOF_PCLK_DI();
    else if(pGPIOx == GPIOG) GPIOG_PCLK_DI();
    else if(pGPIOx == GPIOH) GPIOH_PCLK_DI();
    else if(pGPIOx == GPIOI) GPIOI_PCLK_DI();
    else if(pGPIOx == GPIOJ) GPIOJ_PCLK_DI();
    else if(pGPIOx == GPIOK) GPIOK_PCLK_DI();
    else {
      printf("ERROR: Unrecognized Peripheral passed to GPIO_PeriClockControl()");
    }
  }
  else {
    printf("ERROR: Invalid EnorDi variable passed to GPIO_PeriClockControl()");
  }
}

/*
 * Peripheral Initialization / De-Initialization
 */

/**********************************************************************
 * @fn                - GPIO_Init
 *
 * @brief             - This function initializes the GPIO peripheral
 *
 * @param[in]         - Structure for handling the GPIO Peripheral
 *
 * @return            - none
 *
 * @Note              - none
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle) {
  uint32_t temp = 0;

  // Set Mode
  if(pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode <= GPIO_MODE_ANALOG) {
    pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->MODER |= temp;
  }
  else {
    // TODO: Handle Interrupt mode
  }
  temp = 0;

  // 2. Configure the speed
  pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
  temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
  pGPIOHandle->pGPIOx->OSPEEDR |= temp;
  temp = 0;

  // 3. Configure the pupd settings
  pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
  temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
  pGPIOHandle->pGPIOx->PUPDR |= temp;
  temp = 0;

  // 4. Configure the optype
  pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
  temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
  pGPIOHandle->pGPIOx->OTYPER |= temp;
  temp = 0;

  // 5. Configure Alt function registers
  if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN) {
    if(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber < GPIO_PIN_8) {
      pGPIOHandle->pGPIOx->AFRL &= ~(0xF << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
      temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
      pGPIOHandle->pGPIOx->AFRL |= temp;
    }
    else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber <= GPIO_PIN_15) {
      pGPIOHandle->pGPIOx->AFRH &= ~(0xF << (4 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%8)));
      temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%8));
      pGPIOHandle->pGPIOx->AFRH |= temp;
    }
    else {
      printf("Unrecognized PinNumber is trying to be used during alternate function register configuration");
    }
  }
  temp = 0;
}


/**********************************************************************
 * @fn                - GPIO_DeInit
 *
 * @brief             - This function de-initializes the GPIO peripheral
 *
 * @param[in]         - Base Address of GPIO Peripheral
 *
 * @return            - none
 *
 * @Note              - none
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) {
  if(pGPIOx == GPIOA) GPIOA_REG_RESET();
  else if(pGPIOx == GPIOB) GPIOB_REG_RESET();
  else if(pGPIOx == GPIOC) GPIOC_REG_RESET();
  else if(pGPIOx == GPIOD) GPIOD_REG_RESET();
  else if(pGPIOx == GPIOE) GPIOE_REG_RESET();
  else if(pGPIOx == GPIOF) GPIOF_REG_RESET();
  else if(pGPIOx == GPIOG) GPIOG_REG_RESET();
  else if(pGPIOx == GPIOH) GPIOB_REG_RESET();
  else if(pGPIOx == GPIOI) GPIOB_REG_RESET();
  else if(pGPIOx == GPIOJ) GPIOB_REG_RESET();
  else if(pGPIOx == GPIOK) GPIOB_REG_RESET();
  else {
    printf("ERROR: Unrecognized Peripheral passed to GPIO_DeInit()");
  }

}

/*
 * Data Read & Write
 */

/**********************************************************************
 * @fn                - GPIO_ReadFromInputPin
 *
 * @brief             - This function read current state of input pin
 *
 * @param[in]         - Base Address of GPIO Peripheral
 * @param[in]         - Pin to read from
 *
 * @return            - Current state of the input pin
 *
 * @Note              - none
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {
  uint8_t readVal;
  readVal = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
  return readVal;
}

/**********************************************************************
 * @fn                - GPIO_ReadFromInputPort
 *
 * @brief             - This function read current state of the GPIO Port
 *
 * @param[in]         - Base Address of the GPIO Port
 *
 * @return            - Current start of the input port
 *
 * @Note              - none
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx) {
  uint16_t readVal;
  readVal = (uint16_t)(pGPIOx->IDR);
  return readVal;
}



/**********************************************************************
 * @fn                - GPIO_WritetoOutputPin
 *
 * @brief             - This function writes to output pin
 *
 * @param[in]         - Base Address of the GPIO Port
 * @param[in]         - Pin to write to
 * @param[in]         - Value to write to pin
 *
 * @return            - None
 *
 * @Note              - none
 */
void GPIO_WritetoOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value) {
  if(value == GPIO_PIN_SET) {
    pGPIOx->ODR |= (1 << PinNumber);
  }
  else {
    pGPIOx->ODR &= ~(1 << PinNumber);
  }
}


/**********************************************************************
 * @fn                - GPIO_WritetoOutputPort
 *
 * @brief             - This function writes to output port
 *
 * @param[in]         - Base Address of the GPIO Port
 * @param[in]         - Value to write to port
 *
 * @return            - None
 *
 * @Note              - none
 */
void GPIO_WritetoOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value) {
  pGPIOx->ODR = value;
}


/**********************************************************************
 * @fn                - GPIO_ToggleOutputPin
 *
 * @brief             - This function toggles a specified pin
 *
 * @param[in]         - Base Address of the GPIO Port
 * @param[in]         - Pin to toggle
 *
 * @return            - None
 *
 * @Note              - none
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {
  pGPIOx->ODR  = pGPIOx->ODR ^ ( 1 << PinNumber );
}



/*
 * IRQ Configuration and ISR handling
 */
/**********************************************************************
 * @fn                - GPIO_IRQConfig
 *
 * @brief             - This function configures the interrupt
 *
 * @param[in]         - IRQ Number of interrupt
 * @param[in]         - Desired Priority of interrupt
 * @param[in]         - ENABLE or DISABLE Macro
 *
 * @return            - None
 *
 * @Note              - none
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi) {
}


/**********************************************************************
 * @fn                - GPIO_IRQHandling
 *
 * @brief             - This function handles the IRQ when interrupt occurs
 *
 * @param[in]         - Pin number 
 *
 * @return            - None
 *
 * @Note              - none
 */
void GPIO_IRQHandling(uint8_t PinNumber) {
}

