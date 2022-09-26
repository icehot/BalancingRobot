################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/STM32F3-Discovery/stm32f3_discovery.c \
../Drivers/STM32F3-Discovery/stm32f3_discovery_accelerometer.c \
../Drivers/STM32F3-Discovery/stm32f3_discovery_gyroscope.c 

OBJS += \
./Drivers/STM32F3-Discovery/stm32f3_discovery.o \
./Drivers/STM32F3-Discovery/stm32f3_discovery_accelerometer.o \
./Drivers/STM32F3-Discovery/stm32f3_discovery_gyroscope.o 

C_DEPS += \
./Drivers/STM32F3-Discovery/stm32f3_discovery.d \
./Drivers/STM32F3-Discovery/stm32f3_discovery_accelerometer.d \
./Drivers/STM32F3-Discovery/stm32f3_discovery_gyroscope.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/STM32F3-Discovery/%.o: ../Drivers/STM32F3-Discovery/%.c Drivers/STM32F3-Discovery/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F303xC -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/balaz/STM32Cube/Repository/STM32Cube_FW_F3_V1.11.3/Drivers/BSP" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-STM32F3-2d-Discovery

clean-Drivers-2f-STM32F3-2d-Discovery:
	-$(RM) ./Drivers/STM32F3-Discovery/stm32f3_discovery.d ./Drivers/STM32F3-Discovery/stm32f3_discovery.o ./Drivers/STM32F3-Discovery/stm32f3_discovery_accelerometer.d ./Drivers/STM32F3-Discovery/stm32f3_discovery_accelerometer.o ./Drivers/STM32F3-Discovery/stm32f3_discovery_gyroscope.d ./Drivers/STM32F3-Discovery/stm32f3_discovery_gyroscope.o

.PHONY: clean-Drivers-2f-STM32F3-2d-Discovery

