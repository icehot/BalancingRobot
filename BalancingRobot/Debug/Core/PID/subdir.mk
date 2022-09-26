################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/PID/PID.c \
../Core/PID/PID_Cfg.c 

OBJS += \
./Core/PID/PID.o \
./Core/PID/PID_Cfg.o 

C_DEPS += \
./Core/PID/PID.d \
./Core/PID/PID_Cfg.d 


# Each subdirectory must supply rules for building sources it contributes
Core/PID/%.o: ../Core/PID/%.c Core/PID/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F303xC -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -I"C:/_Cloud/GitHub/BalancingRobot/BalancingRobot/Drivers/Components" -I"C:/_Cloud/GitHub/BalancingRobot/BalancingRobot/Drivers/STM32F3-Discovery" -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-PID

clean-Core-2f-PID:
	-$(RM) ./Core/PID/PID.d ./Core/PID/PID.o ./Core/PID/PID_Cfg.d ./Core/PID/PID_Cfg.o

.PHONY: clean-Core-2f-PID

