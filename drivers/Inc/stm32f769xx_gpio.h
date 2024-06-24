/*
 * stm32f769xx_gpio.h
 *
 *  Created on: Jun 22, 2024
 *      Author: daniel
 */

#ifndef INC_STM32F769XX_GPIO_H_
#define INC_STM32F769XX_GPIO_H_

#include "stm32f769xx.h"

typedef struct {
  uint8_t GPIO_PinNumber;      // From @GPIO_PIN_NUMBERS
  uint8_t GPIO_PinMode;        // From @GPIO_PIN_MODES
  uint8_t GPIO_PinSpeed;       // From @GPIO_PIN_SPEED
  uint8_t GPIO_PinPuPdControl; // From @GPIO_PUPDR_BITS
  uint8_t GPIO_PinOPType;      // From @GPIO_OT_TYPES
  uint8_t GPIO_PinAltFunMode;  // FROM @GPIO_ALTFN_TYPES
}GPIO_PinConfig_t;

typedef struct {
  GPIO_RegDef_t *pGPIOx;              // Holds the base address of the GPIO port to which the pin belongs
  GPIO_PinConfig_t GPIO_PinConfig;   // Holds the GPIO Pin configuration settings
}GPIO_Handle_t;

/*
 * @GPIO_PIN_NUMBERS
 * GPIO Pins
 */
#define GPIO_PIN_0      0        
#define GPIO_PIN_1      1        
#define GPIO_PIN_2      2        
#define GPIO_PIN_3      3        
#define GPIO_PIN_4      4        
#define GPIO_PIN_5      5        
#define GPIO_PIN_6      6        
#define GPIO_PIN_7      7        
#define GPIO_PIN_8      8        
#define GPIO_PIN_9      9        
#define GPIO_PIN_10     10       
#define GPIO_PIN_11     11       
#define GPIO_PIN_12     12       
#define GPIO_PIN_13     13       
#define GPIO_PIN_14     14       
#define GPIO_PIN_15     15       

/*
 * @GPIO_PIN_MODES
 * GPIO possible pin modes
 */
#define GPIO_MODE_IN           0
#define GPIO_MODE_OUT          1
#define GPIO_MODE_ALTFN        2
#define GPIO_MODE_ANALOG       3
#define GPIO_MODE_IT_FT        4
#define GPIO_MODE_IT_RT        5
#define GPIO_MODE_IT_RFT       6

/*
 * @GPIO_OT_TYPES
 * GPIO possible output types
 */
#define GPIO_OP_TYPE_PP        0
#define GPIO_OP_TYPE_OD        1

/*
 * @GPIO_PIN_SPEED
 * GPIO Possible output speed register values
 */
#define GPIO_OSPEED_LOW        0
#define GPIO_OSPEED_MED        1
#define GPIO_OSPEED_HIGH       2
#define GPIO_OSPEED_VERY_HIGH  3

/*
 * @GPIO_PUPDR_BITS
 * GPIO Possible pull-up/pull-down values
 */
#define GPIO_NO_PUPD        0
#define GPIO_PIN_PU          1
#define GPIO_PIN_PD          2

/*
 * @GPIO_ALTFN_TYPES
 * GPIO Possible Alternate Function Types
 */
#define GPIO_AF0       0        
#define GPIO_AF1       1        
#define GPIO_AF2       2        
#define GPIO_AF3       3        
#define GPIO_AF4       4        
#define GPIO_AF5       5        
#define GPIO_AF6       6        
#define GPIO_AF7       7        
#define GPIO_AF8       8        
#define GPIO_AF9       9        
#define GPIO_AF10      10       
#define GPIO_AF11      11       
#define GPIO_AF12      12       
#define GPIO_AF13      13       
#define GPIO_AF14      14       
#define GPIO_AF15      15       
#define GPIO_AF16      16       

/*
 * General Macros
 */
#define GPIO_PIN_SET    1
#define GPIO_PIN_RESET  0

/**********************************************************************
 *                      APIs supported by this driver
 **********************************************************************/

/*
 * Peripheral Clock COntrol
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 * Peripheral Initialization / De-Initialization
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Data Read & Write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WritetoOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value);
void GPIO_WritetoOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ Configuration and ISR handling
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_STM32F769 
XX_GPIO_H_ */
