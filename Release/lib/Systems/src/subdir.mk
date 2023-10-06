################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../lib/Systems/src/stm32l4xx_hal_msp.c \
../lib/Systems/src/stm32l4xx_hal_timebase_tim.c \
../lib/Systems/src/stm32l4xx_it.c \
../lib/Systems/src/syscalls.c \
../lib/Systems/src/sysmem.c \
../lib/Systems/src/system_stm32l4xx.c 

OBJS += \
./lib/Systems/src/stm32l4xx_hal_msp.o \
./lib/Systems/src/stm32l4xx_hal_timebase_tim.o \
./lib/Systems/src/stm32l4xx_it.o \
./lib/Systems/src/syscalls.o \
./lib/Systems/src/sysmem.o \
./lib/Systems/src/system_stm32l4xx.o 

C_DEPS += \
./lib/Systems/src/stm32l4xx_hal_msp.d \
./lib/Systems/src/stm32l4xx_hal_timebase_tim.d \
./lib/Systems/src/stm32l4xx_it.d \
./lib/Systems/src/syscalls.d \
./lib/Systems/src/sysmem.d \
./lib/Systems/src/system_stm32l4xx.d 


# Each subdirectory must supply rules for building sources it contributes
lib/Systems/src/%.o lib/Systems/src/%.su lib/Systems/src/%.cyclo: ../lib/Systems/src/%.c lib/Systems/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32L475xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-lib-2f-Systems-2f-src

clean-lib-2f-Systems-2f-src:
	-$(RM) ./lib/Systems/src/stm32l4xx_hal_msp.cyclo ./lib/Systems/src/stm32l4xx_hal_msp.d ./lib/Systems/src/stm32l4xx_hal_msp.o ./lib/Systems/src/stm32l4xx_hal_msp.su ./lib/Systems/src/stm32l4xx_hal_timebase_tim.cyclo ./lib/Systems/src/stm32l4xx_hal_timebase_tim.d ./lib/Systems/src/stm32l4xx_hal_timebase_tim.o ./lib/Systems/src/stm32l4xx_hal_timebase_tim.su ./lib/Systems/src/stm32l4xx_it.cyclo ./lib/Systems/src/stm32l4xx_it.d ./lib/Systems/src/stm32l4xx_it.o ./lib/Systems/src/stm32l4xx_it.su ./lib/Systems/src/syscalls.cyclo ./lib/Systems/src/syscalls.d ./lib/Systems/src/syscalls.o ./lib/Systems/src/syscalls.su ./lib/Systems/src/sysmem.cyclo ./lib/Systems/src/sysmem.d ./lib/Systems/src/sysmem.o ./lib/Systems/src/sysmem.su ./lib/Systems/src/system_stm32l4xx.cyclo ./lib/Systems/src/system_stm32l4xx.d ./lib/Systems/src/system_stm32l4xx.o ./lib/Systems/src/system_stm32l4xx.su

.PHONY: clean-lib-2f-Systems-2f-src

