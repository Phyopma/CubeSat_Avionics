################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Application/Drivers/Src/adt7420.c 

OBJS += \
./Application/Drivers/Src/adt7420.o 

C_DEPS += \
./Application/Drivers/Src/adt7420.d 


# Each subdirectory must supply rules for building sources it contributes
Application/Drivers/Src/%.o Application/Drivers/Src/%.su Application/Drivers/Src/%.cyclo: ../Application/Drivers/Src/%.c Application/Drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Inc -I"/Users/phyopyae/eecs159/cube_sat_nucleo/Application/Drivers/Inc" -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Application-2f-Drivers-2f-Src

clean-Application-2f-Drivers-2f-Src:
	-$(RM) ./Application/Drivers/Src/adt7420.cyclo ./Application/Drivers/Src/adt7420.d ./Application/Drivers/Src/adt7420.o ./Application/Drivers/Src/adt7420.su

.PHONY: clean-Application-2f-Drivers-2f-Src

