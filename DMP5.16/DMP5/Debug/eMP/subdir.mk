################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../eMP/inv_mpu.c \
../eMP/inv_mpu_dmp_motion_driver.c \
../eMP/mpu6050.c 

C_DEPS += \
./eMP/inv_mpu.d \
./eMP/inv_mpu_dmp_motion_driver.d \
./eMP/mpu6050.d 

OBJS += \
./eMP/inv_mpu.o \
./eMP/inv_mpu_dmp_motion_driver.o \
./eMP/mpu6050.o 


# Each subdirectory must supply rules for building sources it contributes
eMP/%.o eMP/%.su eMP/%.cyclo: ../eMP/%.c eMP/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-eMP

clean-eMP:
	-$(RM) ./eMP/inv_mpu.cyclo ./eMP/inv_mpu.d ./eMP/inv_mpu.o ./eMP/inv_mpu.su ./eMP/inv_mpu_dmp_motion_driver.cyclo ./eMP/inv_mpu_dmp_motion_driver.d ./eMP/inv_mpu_dmp_motion_driver.o ./eMP/inv_mpu_dmp_motion_driver.su ./eMP/mpu6050.cyclo ./eMP/mpu6050.d ./eMP/mpu6050.o ./eMP/mpu6050.su

.PHONY: clean-eMP

