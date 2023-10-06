################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal.c \
../lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_cortex.c \
../lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dfsdm.c \
../lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma.c \
../lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma_ex.c \
../lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_exti.c \
../lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash.c \
../lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ex.c \
../lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ramfunc.c \
../lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_gpio.c \
../lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c.c \
../lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c_ex.c \
../lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_lcd.c \
../lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_lptim.c \
../lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr.c \
../lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr_ex.c \
../lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_qspi.c \
../lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc.c \
../lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc_ex.c \
../lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rtc.c \
../lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rtc_ex.c \
../lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_sai.c \
../lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_spi.c \
../lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_spi_ex.c \
../lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_sram.c \
../lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_swpmi.c \
../lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim.c \
../lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim_ex.c \
../lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart.c \
../lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart_ex.c \
../lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_fmc.c \
../lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_sdmmc.c \
../lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_usb.c 

OBJS += \
./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal.o \
./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_cortex.o \
./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dfsdm.o \
./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma.o \
./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma_ex.o \
./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_exti.o \
./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash.o \
./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ex.o \
./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ramfunc.o \
./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_gpio.o \
./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c.o \
./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c_ex.o \
./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_lcd.o \
./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_lptim.o \
./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr.o \
./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr_ex.o \
./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_qspi.o \
./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc.o \
./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc_ex.o \
./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rtc.o \
./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rtc_ex.o \
./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_sai.o \
./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_spi.o \
./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_spi_ex.o \
./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_sram.o \
./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_swpmi.o \
./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim.o \
./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim_ex.o \
./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart.o \
./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart_ex.o \
./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_fmc.o \
./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_sdmmc.o \
./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_usb.o 

C_DEPS += \
./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal.d \
./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_cortex.d \
./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dfsdm.d \
./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma.d \
./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma_ex.d \
./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_exti.d \
./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash.d \
./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ex.d \
./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ramfunc.d \
./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_gpio.d \
./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c.d \
./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c_ex.d \
./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_lcd.d \
./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_lptim.d \
./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr.d \
./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr_ex.d \
./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_qspi.d \
./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc.d \
./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc_ex.d \
./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rtc.d \
./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rtc_ex.d \
./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_sai.d \
./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_spi.d \
./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_spi_ex.d \
./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_sram.d \
./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_swpmi.d \
./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim.d \
./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim_ex.d \
./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart.d \
./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart_ex.d \
./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_fmc.d \
./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_sdmmc.d \
./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_usb.d 


# Each subdirectory must supply rules for building sources it contributes
lib/STM32L4xx_HAL_Driver/Src/%.o lib/STM32L4xx_HAL_Driver/Src/%.su lib/STM32L4xx_HAL_Driver/Src/%.cyclo: ../lib/STM32L4xx_HAL_Driver/Src/%.c lib/STM32L4xx_HAL_Driver/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32L475xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-lib-2f-STM32L4xx_HAL_Driver-2f-Src

clean-lib-2f-STM32L4xx_HAL_Driver-2f-Src:
	-$(RM) ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal.cyclo ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal.d ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal.o ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal.su ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_cortex.cyclo ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_cortex.d ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_cortex.o ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_cortex.su ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dfsdm.cyclo ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dfsdm.d ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dfsdm.o ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dfsdm.su ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma.cyclo ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma.d ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma.o ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma.su ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma_ex.cyclo ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma_ex.d ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma_ex.o ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma_ex.su ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_exti.cyclo ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_exti.d ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_exti.o ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_exti.su ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash.cyclo ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash.d ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash.o ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash.su ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ex.cyclo ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ex.d ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ex.o ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ex.su ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ramfunc.cyclo ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ramfunc.d ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ramfunc.o ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ramfunc.su ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_gpio.cyclo ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_gpio.d ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_gpio.o ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_gpio.su ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c.cyclo ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c.d ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c.o ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c.su ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c_ex.cyclo ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c_ex.d ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c_ex.o ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c_ex.su ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_lcd.cyclo ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_lcd.d ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_lcd.o ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_lcd.su ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_lptim.cyclo ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_lptim.d ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_lptim.o ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_lptim.su ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr.cyclo ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr.d ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr.o ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr.su ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr_ex.cyclo ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr_ex.d ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr_ex.o ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr_ex.su ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_qspi.cyclo ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_qspi.d ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_qspi.o ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_qspi.su ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc.cyclo ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc.d ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc.o ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc.su ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc_ex.cyclo ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc_ex.d ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc_ex.o ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc_ex.su ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rtc.cyclo ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rtc.d ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rtc.o ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rtc.su ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rtc_ex.cyclo ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rtc_ex.d ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rtc_ex.o ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rtc_ex.su ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_sai.cyclo ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_sai.d ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_sai.o ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_sai.su ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_spi.cyclo ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_spi.d ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_spi.o ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_spi.su ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_spi_ex.cyclo ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_spi_ex.d ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_spi_ex.o ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_spi_ex.su ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_sram.cyclo ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_sram.d ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_sram.o ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_sram.su ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_swpmi.cyclo ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_swpmi.d ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_swpmi.o ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_swpmi.su ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim.cyclo ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim.d ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim.o ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim.su ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim_ex.cyclo ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim_ex.d ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim_ex.o
	-$(RM) ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim_ex.su ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart.cyclo ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart.d ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart.o ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart.su ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart_ex.cyclo ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart_ex.d ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart_ex.o ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart_ex.su ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_fmc.cyclo ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_fmc.d ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_fmc.o ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_fmc.su ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_sdmmc.cyclo ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_sdmmc.d ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_sdmmc.o ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_sdmmc.su ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_usb.cyclo ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_usb.d ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_usb.o ./lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_usb.su

.PHONY: clean-lib-2f-STM32L4xx_HAL_Driver-2f-Src

