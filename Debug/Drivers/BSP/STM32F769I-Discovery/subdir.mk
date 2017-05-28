################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery.c \
../Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery_audio.c 

OBJS += \
./Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery.o \
./Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery_audio.o 

C_DEPS += \
./Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery.d \
./Drivers/BSP/STM32F769I-Discovery/stm32f769i_discovery_audio.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/STM32F769I-Discovery/%.o: ../Drivers/BSP/STM32F769I-Discovery/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-d16 '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -DUSE_HAL_DRIVER -DSTM32F769xx -I"/Users/john/src/stm32/stm32f7xx/audio_pass-through/audio_pass-through/Inc" -I"/Users/john/src/stm32/stm32f7xx/audio_pass-through/audio_pass-through/Drivers/STM32F7xx_HAL_Driver/Inc" -I"/Users/john/src/stm32/stm32f7xx/audio_pass-through/audio_pass-through/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy" -I"/Users/john/src/stm32/stm32f7xx/audio_pass-through/audio_pass-through/Drivers/CMSIS/Device/ST/STM32F7xx/Include" -I"/Users/john/src/stm32/stm32f7xx/audio_pass-through/audio_pass-through/Drivers/CMSIS/Include" -I"/Users/john/src/stm32/stm32f7xx/audio_pass-through/audio_pass-through/Drivers/BSP/STM32F769I-Discovery"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


