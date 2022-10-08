################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ext_driv/driv_src/w25q128jv.c 

OBJS += \
./ext_driv/driv_src/w25q128jv.o 

C_DEPS += \
./ext_driv/driv_src/w25q128jv.d 


# Each subdirectory must supply rules for building sources it contributes
ext_driv/driv_src/%.o: ../ext_driv/driv_src/%.c ext_driv/driv_src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../ext_driv/driv_inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-ext_driv-2f-driv_src

clean-ext_driv-2f-driv_src:
	-$(RM) ./ext_driv/driv_src/w25q128jv.d ./ext_driv/driv_src/w25q128jv.o

.PHONY: clean-ext_driv-2f-driv_src

