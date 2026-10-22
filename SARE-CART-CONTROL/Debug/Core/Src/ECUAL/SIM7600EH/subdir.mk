################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/ECUAL/SIM7600EH/sim7600_gps.c \
../Core/Src/ECUAL/SIM7600EH/sim7600_gps_debug.c \
../Core/Src/ECUAL/SIM7600EH/sim7600_port.c 

OBJS += \
./Core/Src/ECUAL/SIM7600EH/sim7600_gps.o \
./Core/Src/ECUAL/SIM7600EH/sim7600_gps_debug.o \
./Core/Src/ECUAL/SIM7600EH/sim7600_port.o 

C_DEPS += \
./Core/Src/ECUAL/SIM7600EH/sim7600_gps.d \
./Core/Src/ECUAL/SIM7600EH/sim7600_gps_debug.d \
./Core/Src/ECUAL/SIM7600EH/sim7600_port.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/ECUAL/SIM7600EH/%.o Core/Src/ECUAL/SIM7600EH/%.su Core/Src/ECUAL/SIM7600EH/%.cyclo: ../Core/Src/ECUAL/SIM7600EH/%.c Core/Src/ECUAL/SIM7600EH/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-ECUAL-2f-SIM7600EH

clean-Core-2f-Src-2f-ECUAL-2f-SIM7600EH:
	-$(RM) ./Core/Src/ECUAL/SIM7600EH/sim7600_gps.cyclo ./Core/Src/ECUAL/SIM7600EH/sim7600_gps.d ./Core/Src/ECUAL/SIM7600EH/sim7600_gps.o ./Core/Src/ECUAL/SIM7600EH/sim7600_gps.su ./Core/Src/ECUAL/SIM7600EH/sim7600_gps_debug.cyclo ./Core/Src/ECUAL/SIM7600EH/sim7600_gps_debug.d ./Core/Src/ECUAL/SIM7600EH/sim7600_gps_debug.o ./Core/Src/ECUAL/SIM7600EH/sim7600_gps_debug.su ./Core/Src/ECUAL/SIM7600EH/sim7600_port.cyclo ./Core/Src/ECUAL/SIM7600EH/sim7600_port.d ./Core/Src/ECUAL/SIM7600EH/sim7600_port.o ./Core/Src/ECUAL/SIM7600EH/sim7600_port.su

.PHONY: clean-Core-2f-Src-2f-ECUAL-2f-SIM7600EH

