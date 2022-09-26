################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Encoder/Encoder.c 

OBJS += \
./Core/Encoder/Encoder.o 

C_DEPS += \
./Core/Encoder/Encoder.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Encoder/%.o: ../Core/Encoder/%.c Core/Encoder/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F303xC -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -I"C:/_Cloud/GitHub/BalancingRobot/BalancingRobot/Drivers/Components" -I"C:/_Cloud/GitHub/BalancingRobot/BalancingRobot/Drivers/STM32F3-Discovery" -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Encoder

clean-Core-2f-Encoder:
	-$(RM) ./Core/Encoder/Encoder.d ./Core/Encoder/Encoder.o

.PHONY: clean-Core-2f-Encoder

