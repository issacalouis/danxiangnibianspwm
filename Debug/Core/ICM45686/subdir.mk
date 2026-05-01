################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/ICM45686/inv_imu_driver.c \
../Core/ICM45686/inv_imu_transport.c 

OBJS += \
./Core/ICM45686/inv_imu_driver.o \
./Core/ICM45686/inv_imu_transport.o 

C_DEPS += \
./Core/ICM45686/inv_imu_driver.d \
./Core/ICM45686/inv_imu_transport.d 


# Each subdirectory must supply rules for building sources it contributes
Core/ICM45686/%.o Core/ICM45686/%.su Core/ICM45686/%.cyclo: ../Core/ICM45686/%.c Core/ICM45686/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-ICM45686

clean-Core-2f-ICM45686:
	-$(RM) ./Core/ICM45686/inv_imu_driver.cyclo ./Core/ICM45686/inv_imu_driver.d ./Core/ICM45686/inv_imu_driver.o ./Core/ICM45686/inv_imu_driver.su ./Core/ICM45686/inv_imu_transport.cyclo ./Core/ICM45686/inv_imu_transport.d ./Core/ICM45686/inv_imu_transport.o ./Core/ICM45686/inv_imu_transport.su

.PHONY: clean-Core-2f-ICM45686

