################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/ECUAL/ecual_init.c 

OBJS += \
./Core/Src/ECUAL/ecual_init.o 

C_DEPS += \
./Core/Src/ECUAL/ecual_init.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/ECUAL/%.o Core/Src/ECUAL/%.su Core/Src/ECUAL/%.cyclo: ../Core/Src/ECUAL/%.c Core/Src/ECUAL/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-ECUAL

clean-Core-2f-Src-2f-ECUAL:
	-$(RM) ./Core/Src/ECUAL/ecual_init.cyclo ./Core/Src/ECUAL/ecual_init.d ./Core/Src/ECUAL/ecual_init.o ./Core/Src/ECUAL/ecual_init.su

.PHONY: clean-Core-2f-Src-2f-ECUAL

