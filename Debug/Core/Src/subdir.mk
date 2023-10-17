################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/ble_check_state.c \
../Core/Src/enable.c \
../Core/Src/general_functions.c \
../Core/Src/idle_state.c \
../Core/Src/io_manager.c \
../Core/Src/main.c \
../Core/Src/sensors.c \
../Core/Src/startup_state.c \
../Core/Src/state_all.c \
../Core/Src/state_machine.c \
../Core/Src/stm32l4xx_hal_msp.c \
../Core/Src/stm32l4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/system_stm32l4xx.c \
../Core/Src/time_manager.c \
../Core/Src/uart_manager.c 

OBJS += \
./Core/Src/ble_check_state.o \
./Core/Src/enable.o \
./Core/Src/general_functions.o \
./Core/Src/idle_state.o \
./Core/Src/io_manager.o \
./Core/Src/main.o \
./Core/Src/sensors.o \
./Core/Src/startup_state.o \
./Core/Src/state_all.o \
./Core/Src/state_machine.o \
./Core/Src/stm32l4xx_hal_msp.o \
./Core/Src/stm32l4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/system_stm32l4xx.o \
./Core/Src/time_manager.o \
./Core/Src/uart_manager.o 

C_DEPS += \
./Core/Src/ble_check_state.d \
./Core/Src/enable.d \
./Core/Src/general_functions.d \
./Core/Src/idle_state.d \
./Core/Src/io_manager.d \
./Core/Src/main.d \
./Core/Src/sensors.d \
./Core/Src/startup_state.d \
./Core/Src/state_all.d \
./Core/Src/state_machine.d \
./Core/Src/stm32l4xx_hal_msp.d \
./Core/Src/stm32l4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/system_stm32l4xx.d \
./Core/Src/time_manager.d \
./Core/Src/uart_manager.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32L475xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/mattia.sacchi/Desktop/WorkspaceVSC/BLE-IOT/STM32_B-L475E-IOT01A/BLE-sender/Drivers/BSP/include" -I"C:/Users/Mattia Sacchi/Workspace/C Workspace/Progetto/Drivers/BSP/include" -Og -ffunction-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/ble_check_state.cyclo ./Core/Src/ble_check_state.d ./Core/Src/ble_check_state.o ./Core/Src/ble_check_state.su ./Core/Src/enable.cyclo ./Core/Src/enable.d ./Core/Src/enable.o ./Core/Src/enable.su ./Core/Src/general_functions.cyclo ./Core/Src/general_functions.d ./Core/Src/general_functions.o ./Core/Src/general_functions.su ./Core/Src/idle_state.cyclo ./Core/Src/idle_state.d ./Core/Src/idle_state.o ./Core/Src/idle_state.su ./Core/Src/io_manager.cyclo ./Core/Src/io_manager.d ./Core/Src/io_manager.o ./Core/Src/io_manager.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/sensors.cyclo ./Core/Src/sensors.d ./Core/Src/sensors.o ./Core/Src/sensors.su ./Core/Src/startup_state.cyclo ./Core/Src/startup_state.d ./Core/Src/startup_state.o ./Core/Src/startup_state.su ./Core/Src/state_all.cyclo ./Core/Src/state_all.d ./Core/Src/state_all.o ./Core/Src/state_all.su ./Core/Src/state_machine.cyclo ./Core/Src/state_machine.d ./Core/Src/state_machine.o ./Core/Src/state_machine.su ./Core/Src/stm32l4xx_hal_msp.cyclo ./Core/Src/stm32l4xx_hal_msp.d ./Core/Src/stm32l4xx_hal_msp.o ./Core/Src/stm32l4xx_hal_msp.su ./Core/Src/stm32l4xx_it.cyclo ./Core/Src/stm32l4xx_it.d ./Core/Src/stm32l4xx_it.o ./Core/Src/stm32l4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/system_stm32l4xx.cyclo ./Core/Src/system_stm32l4xx.d ./Core/Src/system_stm32l4xx.o ./Core/Src/system_stm32l4xx.su ./Core/Src/time_manager.cyclo ./Core/Src/time_manager.d ./Core/Src/time_manager.o ./Core/Src/time_manager.su ./Core/Src/uart_manager.cyclo ./Core/Src/uart_manager.d ./Core/Src/uart_manager.o ./Core/Src/uart_manager.su

.PHONY: clean-Core-2f-Src

