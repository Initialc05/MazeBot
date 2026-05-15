################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/bt_cmd.c \
../Core/Src/button.c \
../Core/Src/encoder.c \
../Core/Src/freertos.c \
../Core/Src/im948.c \
../Core/Src/lidar.c \
../Core/Src/main.c \
../Core/Src/motor.c \
../Core/Src/pid.c \
../Core/Src/potentiometer.c \
../Core/Src/robot_state.c \
../Core/Src/ssd1306.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/uart_device.c \
../Core/Src/ui_task.c 

OBJS += \
./Core/Src/bt_cmd.o \
./Core/Src/button.o \
./Core/Src/encoder.o \
./Core/Src/freertos.o \
./Core/Src/im948.o \
./Core/Src/lidar.o \
./Core/Src/main.o \
./Core/Src/motor.o \
./Core/Src/pid.o \
./Core/Src/potentiometer.o \
./Core/Src/robot_state.o \
./Core/Src/ssd1306.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/uart_device.o \
./Core/Src/ui_task.o 

C_DEPS += \
./Core/Src/bt_cmd.d \
./Core/Src/button.d \
./Core/Src/encoder.d \
./Core/Src/freertos.d \
./Core/Src/im948.d \
./Core/Src/lidar.d \
./Core/Src/main.d \
./Core/Src/motor.d \
./Core/Src/pid.d \
./Core/Src/potentiometer.d \
./Core/Src/robot_state.d \
./Core/Src/ssd1306.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/uart_device.d \
./Core/Src/ui_task.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/bt_cmd.cyclo ./Core/Src/bt_cmd.d ./Core/Src/bt_cmd.o ./Core/Src/bt_cmd.su ./Core/Src/button.cyclo ./Core/Src/button.d ./Core/Src/button.o ./Core/Src/button.su ./Core/Src/encoder.cyclo ./Core/Src/encoder.d ./Core/Src/encoder.o ./Core/Src/encoder.su ./Core/Src/freertos.cyclo ./Core/Src/freertos.d ./Core/Src/freertos.o ./Core/Src/freertos.su ./Core/Src/im948.cyclo ./Core/Src/im948.d ./Core/Src/im948.o ./Core/Src/im948.su ./Core/Src/lidar.cyclo ./Core/Src/lidar.d ./Core/Src/lidar.o ./Core/Src/lidar.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/motor.cyclo ./Core/Src/motor.d ./Core/Src/motor.o ./Core/Src/motor.su ./Core/Src/pid.cyclo ./Core/Src/pid.d ./Core/Src/pid.o ./Core/Src/pid.su ./Core/Src/potentiometer.cyclo ./Core/Src/potentiometer.d ./Core/Src/potentiometer.o ./Core/Src/potentiometer.su ./Core/Src/robot_state.cyclo ./Core/Src/robot_state.d ./Core/Src/robot_state.o ./Core/Src/robot_state.su ./Core/Src/ssd1306.cyclo ./Core/Src/ssd1306.d ./Core/Src/ssd1306.o ./Core/Src/ssd1306.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/uart_device.cyclo ./Core/Src/uart_device.d ./Core/Src/uart_device.o ./Core/Src/uart_device.su ./Core/Src/ui_task.cyclo ./Core/Src/ui_task.d ./Core/Src/ui_task.o ./Core/Src/ui_task.su

.PHONY: clean-Core-2f-Src

