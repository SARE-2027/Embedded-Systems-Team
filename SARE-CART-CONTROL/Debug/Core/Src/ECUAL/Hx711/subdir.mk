################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/ECUAL/Hx711/hx711.c 

OBJS += \
./Core/Src/ECUAL/Hx711/hx711.o 

C_DEPS += \
./Core/Src/ECUAL/Hx711/hx711.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/ECUAL/Hx711/%.o Core/Src/ECUAL/Hx711/%.su Core/Src/ECUAL/Hx711/%.cyclo: ../Core/Src/ECUAL/Hx711/%.c Core/Src/ECUAL/Hx711/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-ECUAL-2f-Hx711

clean-Core-2f-Src-2f-ECUAL-2f-Hx711:
	-$(RM) ./Core/Src/ECUAL/Hx711/hx711.cyclo ./Core/Src/ECUAL/Hx711/hx711.d ./Core/Src/ECUAL/Hx711/hx711.o ./Core/Src/ECUAL/Hx711/hx711.su

.PHONY: clean-Core-2f-Src-2f-ECUAL-2f-Hx711

