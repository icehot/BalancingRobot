################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Main/freertos.c \
../Core/Main/main.c \
../Core/Main/selftest.c \
../Core/Main/stm32f3xx_hal_msp.c \
../Core/Main/stm32f3xx_hal_timebase_tim.c \
../Core/Main/stm32f3xx_it.c \
../Core/Main/syscalls.c \
../Core/Main/sysmem.c \
../Core/Main/system_stm32f3xx.c 

OBJS += \
./Core/Main/freertos.o \
./Core/Main/main.o \
./Core/Main/selftest.o \
./Core/Main/stm32f3xx_hal_msp.o \
./Core/Main/stm32f3xx_hal_timebase_tim.o \
./Core/Main/stm32f3xx_it.o \
./Core/Main/syscalls.o \
./Core/Main/sysmem.o \
./Core/Main/system_stm32f3xx.o 

C_DEPS += \
./Core/Main/freertos.d \
./Core/Main/main.d \
./Core/Main/selftest.d \
./Core/Main/stm32f3xx_hal_msp.d \
./Core/Main/stm32f3xx_hal_timebase_tim.d \
./Core/Main/stm32f3xx_it.d \
./Core/Main/syscalls.d \
./Core/Main/sysmem.d \
./Core/Main/system_stm32f3xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Main/%.o: ../Core/Main/%.c Core/Main/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F303xC -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -I"C:/_Cloud/GitHub/BalancingRobot/BalancingRobot/Drivers/Components" -I"C:/_Cloud/GitHub/BalancingRobot/BalancingRobot/Drivers/STM32F3-Discovery" -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Main

clean-Core-2f-Main:
	-$(RM) ./Core/Main/freertos.d ./Core/Main/freertos.o ./Core/Main/main.d ./Core/Main/main.o ./Core/Main/selftest.d ./Core/Main/selftest.o ./Core/Main/stm32f3xx_hal_msp.d ./Core/Main/stm32f3xx_hal_msp.o ./Core/Main/stm32f3xx_hal_timebase_tim.d ./Core/Main/stm32f3xx_hal_timebase_tim.o ./Core/Main/stm32f3xx_it.d ./Core/Main/stm32f3xx_it.o ./Core/Main/syscalls.d ./Core/Main/syscalls.o ./Core/Main/sysmem.d ./Core/Main/sysmem.o ./Core/Main/system_stm32f3xx.d ./Core/Main/system_stm32f3xx.o

.PHONY: clean-Core-2f-Main

