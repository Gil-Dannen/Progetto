################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../lib/BSP/source/cs42l51.c \
../lib/BSP/source/cs43l22.c \
../lib/BSP/source/cy8c4014lqi.c \
../lib/BSP/source/ft3x67.c \
../lib/BSP/source/ft5336.c \
../lib/BSP/source/ft6x06.c \
../lib/BSP/source/hts221.c \
../lib/BSP/source/hx8347g.c \
../lib/BSP/source/hx8347i.c \
../lib/BSP/source/l3gd20.c \
../lib/BSP/source/lis3mdl.c \
../lib/BSP/source/lps22hb.c \
../lib/BSP/source/ls016b8uy.c \
../lib/BSP/source/lsm303c.c \
../lib/BSP/source/lsm303dlhc.c \
../lib/BSP/source/lsm6dsl.c \
../lib/BSP/source/m24sr.c \
../lib/BSP/source/mfxstm32l152.c \
../lib/BSP/source/ov9655.c \
../lib/BSP/source/st25dv.c \
../lib/BSP/source/st25dv_reg.c \
../lib/BSP/source/st7735.c \
../lib/BSP/source/st7789h2.c \
../lib/BSP/source/stm32l475e_iot01.c \
../lib/BSP/source/stm32l475e_iot01_accelero.c \
../lib/BSP/source/stm32l475e_iot01_gyro.c \
../lib/BSP/source/stm32l475e_iot01_hsensor.c \
../lib/BSP/source/stm32l475e_iot01_magneto.c \
../lib/BSP/source/stm32l475e_iot01_psensor.c \
../lib/BSP/source/stm32l475e_iot01_tsensor.c \
../lib/BSP/source/stmpe1600.c \
../lib/BSP/source/stmpe811.c \
../lib/BSP/source/wm8994.c 

OBJS += \
./lib/BSP/source/cs42l51.o \
./lib/BSP/source/cs43l22.o \
./lib/BSP/source/cy8c4014lqi.o \
./lib/BSP/source/ft3x67.o \
./lib/BSP/source/ft5336.o \
./lib/BSP/source/ft6x06.o \
./lib/BSP/source/hts221.o \
./lib/BSP/source/hx8347g.o \
./lib/BSP/source/hx8347i.o \
./lib/BSP/source/l3gd20.o \
./lib/BSP/source/lis3mdl.o \
./lib/BSP/source/lps22hb.o \
./lib/BSP/source/ls016b8uy.o \
./lib/BSP/source/lsm303c.o \
./lib/BSP/source/lsm303dlhc.o \
./lib/BSP/source/lsm6dsl.o \
./lib/BSP/source/m24sr.o \
./lib/BSP/source/mfxstm32l152.o \
./lib/BSP/source/ov9655.o \
./lib/BSP/source/st25dv.o \
./lib/BSP/source/st25dv_reg.o \
./lib/BSP/source/st7735.o \
./lib/BSP/source/st7789h2.o \
./lib/BSP/source/stm32l475e_iot01.o \
./lib/BSP/source/stm32l475e_iot01_accelero.o \
./lib/BSP/source/stm32l475e_iot01_gyro.o \
./lib/BSP/source/stm32l475e_iot01_hsensor.o \
./lib/BSP/source/stm32l475e_iot01_magneto.o \
./lib/BSP/source/stm32l475e_iot01_psensor.o \
./lib/BSP/source/stm32l475e_iot01_tsensor.o \
./lib/BSP/source/stmpe1600.o \
./lib/BSP/source/stmpe811.o \
./lib/BSP/source/wm8994.o 

C_DEPS += \
./lib/BSP/source/cs42l51.d \
./lib/BSP/source/cs43l22.d \
./lib/BSP/source/cy8c4014lqi.d \
./lib/BSP/source/ft3x67.d \
./lib/BSP/source/ft5336.d \
./lib/BSP/source/ft6x06.d \
./lib/BSP/source/hts221.d \
./lib/BSP/source/hx8347g.d \
./lib/BSP/source/hx8347i.d \
./lib/BSP/source/l3gd20.d \
./lib/BSP/source/lis3mdl.d \
./lib/BSP/source/lps22hb.d \
./lib/BSP/source/ls016b8uy.d \
./lib/BSP/source/lsm303c.d \
./lib/BSP/source/lsm303dlhc.d \
./lib/BSP/source/lsm6dsl.d \
./lib/BSP/source/m24sr.d \
./lib/BSP/source/mfxstm32l152.d \
./lib/BSP/source/ov9655.d \
./lib/BSP/source/st25dv.d \
./lib/BSP/source/st25dv_reg.d \
./lib/BSP/source/st7735.d \
./lib/BSP/source/st7789h2.d \
./lib/BSP/source/stm32l475e_iot01.d \
./lib/BSP/source/stm32l475e_iot01_accelero.d \
./lib/BSP/source/stm32l475e_iot01_gyro.d \
./lib/BSP/source/stm32l475e_iot01_hsensor.d \
./lib/BSP/source/stm32l475e_iot01_magneto.d \
./lib/BSP/source/stm32l475e_iot01_psensor.d \
./lib/BSP/source/stm32l475e_iot01_tsensor.d \
./lib/BSP/source/stmpe1600.d \
./lib/BSP/source/stmpe811.d \
./lib/BSP/source/wm8994.d 


# Each subdirectory must supply rules for building sources it contributes
lib/BSP/source/%.o lib/BSP/source/%.su lib/BSP/source/%.cyclo: ../lib/BSP/source/%.c lib/BSP/source/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32L475xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-lib-2f-BSP-2f-source

clean-lib-2f-BSP-2f-source:
	-$(RM) ./lib/BSP/source/cs42l51.cyclo ./lib/BSP/source/cs42l51.d ./lib/BSP/source/cs42l51.o ./lib/BSP/source/cs42l51.su ./lib/BSP/source/cs43l22.cyclo ./lib/BSP/source/cs43l22.d ./lib/BSP/source/cs43l22.o ./lib/BSP/source/cs43l22.su ./lib/BSP/source/cy8c4014lqi.cyclo ./lib/BSP/source/cy8c4014lqi.d ./lib/BSP/source/cy8c4014lqi.o ./lib/BSP/source/cy8c4014lqi.su ./lib/BSP/source/ft3x67.cyclo ./lib/BSP/source/ft3x67.d ./lib/BSP/source/ft3x67.o ./lib/BSP/source/ft3x67.su ./lib/BSP/source/ft5336.cyclo ./lib/BSP/source/ft5336.d ./lib/BSP/source/ft5336.o ./lib/BSP/source/ft5336.su ./lib/BSP/source/ft6x06.cyclo ./lib/BSP/source/ft6x06.d ./lib/BSP/source/ft6x06.o ./lib/BSP/source/ft6x06.su ./lib/BSP/source/hts221.cyclo ./lib/BSP/source/hts221.d ./lib/BSP/source/hts221.o ./lib/BSP/source/hts221.su ./lib/BSP/source/hx8347g.cyclo ./lib/BSP/source/hx8347g.d ./lib/BSP/source/hx8347g.o ./lib/BSP/source/hx8347g.su ./lib/BSP/source/hx8347i.cyclo ./lib/BSP/source/hx8347i.d ./lib/BSP/source/hx8347i.o ./lib/BSP/source/hx8347i.su ./lib/BSP/source/l3gd20.cyclo ./lib/BSP/source/l3gd20.d ./lib/BSP/source/l3gd20.o ./lib/BSP/source/l3gd20.su ./lib/BSP/source/lis3mdl.cyclo ./lib/BSP/source/lis3mdl.d ./lib/BSP/source/lis3mdl.o ./lib/BSP/source/lis3mdl.su ./lib/BSP/source/lps22hb.cyclo ./lib/BSP/source/lps22hb.d ./lib/BSP/source/lps22hb.o ./lib/BSP/source/lps22hb.su ./lib/BSP/source/ls016b8uy.cyclo ./lib/BSP/source/ls016b8uy.d ./lib/BSP/source/ls016b8uy.o ./lib/BSP/source/ls016b8uy.su ./lib/BSP/source/lsm303c.cyclo ./lib/BSP/source/lsm303c.d ./lib/BSP/source/lsm303c.o ./lib/BSP/source/lsm303c.su ./lib/BSP/source/lsm303dlhc.cyclo ./lib/BSP/source/lsm303dlhc.d ./lib/BSP/source/lsm303dlhc.o ./lib/BSP/source/lsm303dlhc.su ./lib/BSP/source/lsm6dsl.cyclo ./lib/BSP/source/lsm6dsl.d ./lib/BSP/source/lsm6dsl.o ./lib/BSP/source/lsm6dsl.su ./lib/BSP/source/m24sr.cyclo ./lib/BSP/source/m24sr.d ./lib/BSP/source/m24sr.o ./lib/BSP/source/m24sr.su ./lib/BSP/source/mfxstm32l152.cyclo ./lib/BSP/source/mfxstm32l152.d ./lib/BSP/source/mfxstm32l152.o ./lib/BSP/source/mfxstm32l152.su ./lib/BSP/source/ov9655.cyclo ./lib/BSP/source/ov9655.d ./lib/BSP/source/ov9655.o ./lib/BSP/source/ov9655.su ./lib/BSP/source/st25dv.cyclo ./lib/BSP/source/st25dv.d ./lib/BSP/source/st25dv.o ./lib/BSP/source/st25dv.su ./lib/BSP/source/st25dv_reg.cyclo ./lib/BSP/source/st25dv_reg.d ./lib/BSP/source/st25dv_reg.o ./lib/BSP/source/st25dv_reg.su ./lib/BSP/source/st7735.cyclo ./lib/BSP/source/st7735.d ./lib/BSP/source/st7735.o ./lib/BSP/source/st7735.su ./lib/BSP/source/st7789h2.cyclo ./lib/BSP/source/st7789h2.d ./lib/BSP/source/st7789h2.o ./lib/BSP/source/st7789h2.su ./lib/BSP/source/stm32l475e_iot01.cyclo ./lib/BSP/source/stm32l475e_iot01.d ./lib/BSP/source/stm32l475e_iot01.o ./lib/BSP/source/stm32l475e_iot01.su ./lib/BSP/source/stm32l475e_iot01_accelero.cyclo ./lib/BSP/source/stm32l475e_iot01_accelero.d ./lib/BSP/source/stm32l475e_iot01_accelero.o ./lib/BSP/source/stm32l475e_iot01_accelero.su ./lib/BSP/source/stm32l475e_iot01_gyro.cyclo ./lib/BSP/source/stm32l475e_iot01_gyro.d ./lib/BSP/source/stm32l475e_iot01_gyro.o ./lib/BSP/source/stm32l475e_iot01_gyro.su ./lib/BSP/source/stm32l475e_iot01_hsensor.cyclo ./lib/BSP/source/stm32l475e_iot01_hsensor.d ./lib/BSP/source/stm32l475e_iot01_hsensor.o ./lib/BSP/source/stm32l475e_iot01_hsensor.su ./lib/BSP/source/stm32l475e_iot01_magneto.cyclo ./lib/BSP/source/stm32l475e_iot01_magneto.d ./lib/BSP/source/stm32l475e_iot01_magneto.o ./lib/BSP/source/stm32l475e_iot01_magneto.su ./lib/BSP/source/stm32l475e_iot01_psensor.cyclo ./lib/BSP/source/stm32l475e_iot01_psensor.d ./lib/BSP/source/stm32l475e_iot01_psensor.o ./lib/BSP/source/stm32l475e_iot01_psensor.su ./lib/BSP/source/stm32l475e_iot01_tsensor.cyclo ./lib/BSP/source/stm32l475e_iot01_tsensor.d ./lib/BSP/source/stm32l475e_iot01_tsensor.o ./lib/BSP/source/stm32l475e_iot01_tsensor.su ./lib/BSP/source/stmpe1600.cyclo ./lib/BSP/source/stmpe1600.d ./lib/BSP/source/stmpe1600.o ./lib/BSP/source/stmpe1600.su ./lib/BSP/source/stmpe811.cyclo ./lib/BSP/source/stmpe811.d ./lib/BSP/source/stmpe811.o ./lib/BSP/source/stmpe811.su ./lib/BSP/source/wm8994.cyclo ./lib/BSP/source/wm8994.d ./lib/BSP/source/wm8994.o ./lib/BSP/source/wm8994.su

.PHONY: clean-lib-2f-BSP-2f-source

