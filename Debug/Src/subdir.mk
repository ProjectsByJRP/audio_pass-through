################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/dfsdm.c \
../Src/gpio.c \
../Src/main.c \
../Src/sai.c \
../Src/stm32f7xx_hal_msp.c \
../Src/stm32f7xx_it.c \
../Src/syscalls.c \
../Src/system_stm32f7xx.c \
../Src/usart.c 

OBJS += \
./Src/dfsdm.o \
./Src/gpio.o \
./Src/main.o \
./Src/sai.o \
./Src/stm32f7xx_hal_msp.o \
./Src/stm32f7xx_it.o \
./Src/syscalls.o \
./Src/system_stm32f7xx.o \
./Src/usart.o 

C_DEPS += \
./Src/dfsdm.d \
./Src/gpio.d \
./Src/main.d \
./Src/sai.d \
./Src/stm32f7xx_hal_msp.d \
./Src/stm32f7xx_it.d \
./Src/syscalls.d \
./Src/system_stm32f7xx.d \
./Src/usart.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-d16 '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -DUSE_HAL_DRIVER -DSTM32F769xx -I"/Users/john/src/stm32/stm32f7xx/audio_pass-through/audio_pass-through/Inc" -I"/Users/john/src/stm32/stm32f7xx/audio_pass-through/audio_pass-through/Drivers/STM32F7xx_HAL_Driver/Inc" -I"/Users/john/src/stm32/stm32f7xx/audio_pass-through/audio_pass-through/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy" -I"/Users/john/src/stm32/stm32f7xx/audio_pass-through/audio_pass-through/Drivers/CMSIS/Device/ST/STM32F7xx/Include" -I"/Users/john/src/stm32/stm32f7xx/audio_pass-through/audio_pass-through/Drivers/CMSIS/Include" -I"/Users/john/src/stm32/stm32f7xx/audio_pass-through/audio_pass-through/Drivers/BSP/STM32F769I-Discovery"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


