################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/louib/Desktop/KTH\ Aerospace/Thesis\ Project/3)\ MATLAB\ Simulink\ Electronics/3_5)\ Simulink/Torque\ Controller/Torque_Controller/Torque_Controller.c 

OBJS += \
./Drivers/Simulink/Torque_Controller.o 

C_DEPS += \
./Drivers/Simulink/Torque_Controller.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Simulink/Torque_Controller.o: C:/Users/louib/Desktop/KTH\ Aerospace/Thesis\ Project/3)\ MATLAB\ Simulink\ Electronics/3_5)\ Simulink/Torque\ Controller/Torque_Controller/Torque_Controller.c Drivers/Simulink/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F302x8 -c -I../../Inc -I"C:/Users/louib/Desktop/KTH Aerospace/Thesis Project/3) MATLAB Simulink Electronics/3_5) Simulink/Torque Controller/Torque_Controller" -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.1.2-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.1.2-Full/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Simulink

clean-Drivers-2f-Simulink:
	-$(RM) ./Drivers/Simulink/Torque_Controller.cyclo ./Drivers/Simulink/Torque_Controller.d ./Drivers/Simulink/Torque_Controller.o ./Drivers/Simulink/Torque_Controller.su

.PHONY: clean-Drivers-2f-Simulink

