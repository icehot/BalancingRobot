################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Components/i3g4250d/i3g4250d.c 

OBJS += \
./Drivers/Components/i3g4250d/i3g4250d.o 

C_DEPS += \
./Drivers/Components/i3g4250d/i3g4250d.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Components/i3g4250d/%.o: ../Drivers/Components/i3g4250d/%.c Drivers/Components/i3g4250d/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F303xC -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/balaz/STM32Cube/Repository/STM32Cube_FW_F3_V1.11.3/Drivers/BSP" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Components-2f-i3g4250d

clean-Drivers-2f-Components-2f-i3g4250d:
	-$(RM) ./Drivers/Components/i3g4250d/i3g4250d.d ./Drivers/Components/i3g4250d/i3g4250d.o

.PHONY: clean-Drivers-2f-Components-2f-i3g4250d

