################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../src/startup_stm32l475vgtx.s 

C_SRCS += \
../src/ble_check_state.c \
../src/ble_interface.c \
../src/ble_manager.c \
../src/idle_state.c \
../src/io_manager.c \
../src/main.c \
../src/startup_state.c \
../src/state_all.c \
../src/state_machine.c \
../src/time_manager.c \
../src/uart_manager.c 

OBJS += \
./src/ble_check_state.o \
./src/ble_interface.o \
./src/ble_manager.o \
./src/idle_state.o \
./src/io_manager.o \
./src/main.o \
./src/startup_state.o \
./src/startup_stm32l475vgtx.o \
./src/state_all.o \
./src/state_machine.o \
./src/time_manager.o \
./src/uart_manager.o 

S_DEPS += \
./src/startup_stm32l475vgtx.d 

C_DEPS += \
./src/ble_check_state.d \
./src/ble_interface.d \
./src/ble_manager.d \
./src/idle_state.d \
./src/io_manager.d \
./src/main.d \
./src/startup_state.d \
./src/state_all.d \
./src/state_machine.d \
./src/time_manager.d \
./src/uart_manager.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o src/%.su src/%.cyclo: ../src/%.c src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32L475xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
src/%.o: ../src/%.s src/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -c -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-src

clean-src:
	-$(RM) ./src/ble_check_state.cyclo ./src/ble_check_state.d ./src/ble_check_state.o ./src/ble_check_state.su ./src/ble_interface.cyclo ./src/ble_interface.d ./src/ble_interface.o ./src/ble_interface.su ./src/ble_manager.cyclo ./src/ble_manager.d ./src/ble_manager.o ./src/ble_manager.su ./src/idle_state.cyclo ./src/idle_state.d ./src/idle_state.o ./src/idle_state.su ./src/io_manager.cyclo ./src/io_manager.d ./src/io_manager.o ./src/io_manager.su ./src/main.cyclo ./src/main.d ./src/main.o ./src/main.su ./src/startup_state.cyclo ./src/startup_state.d ./src/startup_state.o ./src/startup_state.su ./src/startup_stm32l475vgtx.d ./src/startup_stm32l475vgtx.o ./src/state_all.cyclo ./src/state_all.d ./src/state_all.o ./src/state_all.su ./src/state_machine.cyclo ./src/state_machine.d ./src/state_machine.o ./src/state_machine.su ./src/time_manager.cyclo ./src/time_manager.d ./src/time_manager.o ./src/time_manager.su ./src/uart_manager.cyclo ./src/uart_manager.d ./src/uart_manager.o ./src/uart_manager.su

.PHONY: clean-src

