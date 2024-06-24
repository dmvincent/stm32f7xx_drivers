#include "stm32f769xx.h"

void delay(void) {
  for(uint32_t i = 0; i < 500000; i++);
}

int main(void) {

  // Define the LED GPIO Pin Config
  GPIO_PinConfig_t LED_PinConfig;
  LED_PinConfig.GPIO_PinNumber = GPIO_PIN_4;
  LED_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
  LED_PinConfig.GPIO_PinSpeed = GPIO_OSPEED_LOW;
  LED_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
  LED_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;

  // Define the BTN GPIO Pin Config
  GPIO_PinConfig_t BTN_PinConfig;
  BTN_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
  BTN_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
  BTN_PinConfig.GPIO_PinSpeed = GPIO_OSPEED_LOW;
  BTN_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

  // Define LED GPIO Handle
  GPIO_Handle_t LED_pGPIOHandle;
  LED_pGPIOHandle.pGPIOx = GPIOD;
  LED_pGPIOHandle.GPIO_PinConfig = LED_PinConfig;
  
  // Define BTN GPIO Handle
  GPIO_Handle_t BTN_pGPIOHandle;
  BTN_pGPIOHandle.pGPIOx = GPIOA;
  BTN_pGPIOHandle.GPIO_PinConfig = BTN_PinConfig;

  // Enable the clock for GPIOD
  GPIO_PeriClockControl(LED_pGPIOHandle.pGPIOx, ENABLE);

  // Enable the clock for GPIOA
  GPIO_PeriClockControl(BTN_pGPIOHandle.pGPIOx, ENABLE);  // Initialize GPIOD

  // Initialize both handles
  GPIO_Init(&LED_pGPIOHandle);
  GPIO_Init(&BTN_pGPIOHandle);

  while(1) {
    uint8_t readVal;
    readVal = GPIO_ReadFromInputPin(BTN_pGPIOHandle.pGPIOx, BTN_pGPIOHandle.GPIO_PinConfig.GPIO_PinNumber);
    if(readVal) {
      GPIO_ToggleOutputPin(LED_pGPIOHandle.pGPIOx, LED_pGPIOHandle.GPIO_PinConfig.GPIO_PinNumber);
    }
    delay();
    
  }

  // De-Initialize Port
  GPIO_DeInit(LED_pGPIOHandle.pGPIOx);
  GPIO_DeInit(BTN_pGPIOHandle.pGPIOx);

  // DISABLE PERIPHERAL CLOCKS
  GPIO_PeriClockControl(LED_pGPIOHandle.pGPIOx, DISABLE);
  GPIO_PeriClockControl(BTN_pGPIOHandle.pGPIOx, DISABLE);
}
