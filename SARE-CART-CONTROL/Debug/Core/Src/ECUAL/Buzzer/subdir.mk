################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/ECUAL/Buzzer/buzzer.c 

OBJS += \
./Core/Src/ECUAL/Buzzer/buzzer.o 

C_DEPS += \
./Core/Src/ECUAL/Buzzer/buzzer.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/ECUAL/Buzzer/%.o Core/Src/ECUAL/Buzzer/%.su Core/Src/ECUAL/Buzzer/%.cyclo: ../Core/Src/ECUAL/Buzzer/%.c Core/Src/ECUAL/Buzzer/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-ECUAL-2f-Buzzer

clean-Core-2f-Src-2f-ECUAL-2f-Buzzer:
	-$(RM) ./Core/Src/ECUAL/Buzzer/buzzer.cyclo ./Core/Src/ECUAL/Buzzer/buzzer.d ./Core/Src/ECUAL/Buzzer/buzzer.o ./Core/Src/ECUAL/Buzzer/buzzer.su

.PHONY: clean-Core-2f-Src-2f-ECUAL-2f-Buzzer

