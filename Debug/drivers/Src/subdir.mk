################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/Src/stm32f769xx_gpio.c 

OBJS += \
./drivers/Src/stm32f769xx_gpio.o 

C_DEPS += \
./drivers/Src/stm32f769xx_gpio.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/Src/%.o drivers/Src/%.su drivers/Src/%.cyclo: ../drivers/Src/%.c drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F769I_DISCO -DSTM32F7 -DSTM32F769NIHx -c -I../Inc -I../drivers/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-drivers-2f-Src

clean-drivers-2f-Src:
	-$(RM) ./drivers/Src/stm32f769xx_gpio.cyclo ./drivers/Src/stm32f769xx_gpio.d ./drivers/Src/stm32f769xx_gpio.o ./drivers/Src/stm32f769xx_gpio.su

.PHONY: clean-drivers-2f-Src

