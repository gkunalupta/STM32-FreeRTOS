################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../EXTE_DRIV/w25q128jv.c 

OBJS += \
./EXTE_DRIV/w25q128jv.o 

C_DEPS += \
./EXTE_DRIV/w25q128jv.d 


# Each subdirectory must supply rules for building sources it contributes
EXTE_DRIV/%.o: ../EXTE_DRIV/%.c EXTE_DRIV/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../EXTE_DRIV -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-EXTE_DRIV

clean-EXTE_DRIV:
	-$(RM) ./EXTE_DRIV/w25q128jv.d ./EXTE_DRIV/w25q128jv.o

.PHONY: clean-EXTE_DRIV

