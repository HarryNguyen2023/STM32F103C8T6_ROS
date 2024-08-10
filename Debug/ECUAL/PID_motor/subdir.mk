################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ECUAL/PID_motor/PID_motor.c \
../ECUAL/PID_motor/PID_motor_cfg.c 

C_DEPS += \
./ECUAL/PID_motor/PID_motor.d \
./ECUAL/PID_motor/PID_motor_cfg.d 

OBJS += \
./ECUAL/PID_motor/PID_motor.o \
./ECUAL/PID_motor/PID_motor_cfg.o 


# Each subdirectory must supply rules for building sources it contributes
ECUAL/PID_motor/%.o ECUAL/PID_motor/%.su ECUAL/PID_motor/%.cyclo: ../ECUAL/PID_motor/%.c ECUAL/PID_motor/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I/ECUAL -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-ECUAL-2f-PID_motor

clean-ECUAL-2f-PID_motor:
	-$(RM) ./ECUAL/PID_motor/PID_motor.cyclo ./ECUAL/PID_motor/PID_motor.d ./ECUAL/PID_motor/PID_motor.o ./ECUAL/PID_motor/PID_motor.su ./ECUAL/PID_motor/PID_motor_cfg.cyclo ./ECUAL/PID_motor/PID_motor_cfg.d ./ECUAL/PID_motor/PID_motor_cfg.o ./ECUAL/PID_motor/PID_motor_cfg.su

.PHONY: clean-ECUAL-2f-PID_motor

