################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/louib/.st_workbench/projects/Torque_Control/Src/main.c \
C:/Users/louib/.st_workbench/projects/Torque_Control/Src/mc_api.c \
C:/Users/louib/.st_workbench/projects/Torque_Control/Src/mc_app_hooks.c \
C:/Users/louib/.st_workbench/projects/Torque_Control/Src/mc_config.c \
C:/Users/louib/.st_workbench/projects/Torque_Control/Src/mc_configuration_registers.c \
C:/Users/louib/.st_workbench/projects/Torque_Control/Src/mc_interface.c \
C:/Users/louib/.st_workbench/projects/Torque_Control/Src/mc_math.c \
C:/Users/louib/.st_workbench/projects/Torque_Control/Src/mc_parameters.c \
C:/Users/louib/.st_workbench/projects/Torque_Control/Src/mc_perf.c \
C:/Users/louib/.st_workbench/projects/Torque_Control/Src/mc_tasks.c \
C:/Users/louib/.st_workbench/projects/Torque_Control/Src/motorcontrol.c \
C:/Users/louib/.st_workbench/projects/Torque_Control/Src/pwm_curr_fdbk.c \
C:/Users/louib/.st_workbench/projects/Torque_Control/Src/register_interface.c \
C:/Users/louib/.st_workbench/projects/Torque_Control/Src/regular_conversion_manager.c \
C:/Users/louib/.st_workbench/projects/Torque_Control/Src/stm32f30x_mc_it.c \
C:/Users/louib/.st_workbench/projects/Torque_Control/Src/stm32f3xx_hal_msp.c \
C:/Users/louib/.st_workbench/projects/Torque_Control/Src/stm32f3xx_it.c \
../Application/User/syscalls.c \
../Application/User/sysmem.c 

OBJS += \
./Application/User/main.o \
./Application/User/mc_api.o \
./Application/User/mc_app_hooks.o \
./Application/User/mc_config.o \
./Application/User/mc_configuration_registers.o \
./Application/User/mc_interface.o \
./Application/User/mc_math.o \
./Application/User/mc_parameters.o \
./Application/User/mc_perf.o \
./Application/User/mc_tasks.o \
./Application/User/motorcontrol.o \
./Application/User/pwm_curr_fdbk.o \
./Application/User/register_interface.o \
./Application/User/regular_conversion_manager.o \
./Application/User/stm32f30x_mc_it.o \
./Application/User/stm32f3xx_hal_msp.o \
./Application/User/stm32f3xx_it.o \
./Application/User/syscalls.o \
./Application/User/sysmem.o 

C_DEPS += \
./Application/User/main.d \
./Application/User/mc_api.d \
./Application/User/mc_app_hooks.d \
./Application/User/mc_config.d \
./Application/User/mc_configuration_registers.d \
./Application/User/mc_interface.d \
./Application/User/mc_math.d \
./Application/User/mc_parameters.d \
./Application/User/mc_perf.d \
./Application/User/mc_tasks.d \
./Application/User/motorcontrol.d \
./Application/User/pwm_curr_fdbk.d \
./Application/User/register_interface.d \
./Application/User/regular_conversion_manager.d \
./Application/User/stm32f30x_mc_it.d \
./Application/User/stm32f3xx_hal_msp.d \
./Application/User/stm32f3xx_it.d \
./Application/User/syscalls.d \
./Application/User/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Application/User/main.o: C:/Users/louib/.st_workbench/projects/Torque_Control/Src/main.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F302x8 -c -I../../Inc -I"C:/Users/louib/Desktop/KTH Aerospace/Thesis Project/3) MATLAB Simulink Electronics/3_5) Simulink/Torque Controller/Torque_Controller" -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.1.2-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.1.2-Full/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/mc_api.o: C:/Users/louib/.st_workbench/projects/Torque_Control/Src/mc_api.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F302x8 -c -I../../Inc -I"C:/Users/louib/Desktop/KTH Aerospace/Thesis Project/3) MATLAB Simulink Electronics/3_5) Simulink/Torque Controller/Torque_Controller" -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.1.2-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.1.2-Full/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/mc_app_hooks.o: C:/Users/louib/.st_workbench/projects/Torque_Control/Src/mc_app_hooks.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F302x8 -c -I../../Inc -I"C:/Users/louib/Desktop/KTH Aerospace/Thesis Project/3) MATLAB Simulink Electronics/3_5) Simulink/Torque Controller/Torque_Controller" -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.1.2-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.1.2-Full/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/mc_config.o: C:/Users/louib/.st_workbench/projects/Torque_Control/Src/mc_config.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F302x8 -c -I../../Inc -I"C:/Users/louib/Desktop/KTH Aerospace/Thesis Project/3) MATLAB Simulink Electronics/3_5) Simulink/Torque Controller/Torque_Controller" -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.1.2-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.1.2-Full/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/mc_configuration_registers.o: C:/Users/louib/.st_workbench/projects/Torque_Control/Src/mc_configuration_registers.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F302x8 -c -I../../Inc -I"C:/Users/louib/Desktop/KTH Aerospace/Thesis Project/3) MATLAB Simulink Electronics/3_5) Simulink/Torque Controller/Torque_Controller" -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.1.2-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.1.2-Full/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/mc_interface.o: C:/Users/louib/.st_workbench/projects/Torque_Control/Src/mc_interface.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F302x8 -c -I../../Inc -I"C:/Users/louib/Desktop/KTH Aerospace/Thesis Project/3) MATLAB Simulink Electronics/3_5) Simulink/Torque Controller/Torque_Controller" -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.1.2-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.1.2-Full/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/mc_math.o: C:/Users/louib/.st_workbench/projects/Torque_Control/Src/mc_math.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F302x8 -c -I../../Inc -I"C:/Users/louib/Desktop/KTH Aerospace/Thesis Project/3) MATLAB Simulink Electronics/3_5) Simulink/Torque Controller/Torque_Controller" -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.1.2-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.1.2-Full/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/mc_parameters.o: C:/Users/louib/.st_workbench/projects/Torque_Control/Src/mc_parameters.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F302x8 -c -I../../Inc -I"C:/Users/louib/Desktop/KTH Aerospace/Thesis Project/3) MATLAB Simulink Electronics/3_5) Simulink/Torque Controller/Torque_Controller" -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.1.2-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.1.2-Full/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/mc_perf.o: C:/Users/louib/.st_workbench/projects/Torque_Control/Src/mc_perf.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F302x8 -c -I../../Inc -I"C:/Users/louib/Desktop/KTH Aerospace/Thesis Project/3) MATLAB Simulink Electronics/3_5) Simulink/Torque Controller/Torque_Controller" -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.1.2-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.1.2-Full/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/mc_tasks.o: C:/Users/louib/.st_workbench/projects/Torque_Control/Src/mc_tasks.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F302x8 -c -I../../Inc -I"C:/Users/louib/Desktop/KTH Aerospace/Thesis Project/3) MATLAB Simulink Electronics/3_5) Simulink/Torque Controller/Torque_Controller" -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.1.2-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.1.2-Full/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/motorcontrol.o: C:/Users/louib/.st_workbench/projects/Torque_Control/Src/motorcontrol.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F302x8 -c -I../../Inc -I"C:/Users/louib/Desktop/KTH Aerospace/Thesis Project/3) MATLAB Simulink Electronics/3_5) Simulink/Torque Controller/Torque_Controller" -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.1.2-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.1.2-Full/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/pwm_curr_fdbk.o: C:/Users/louib/.st_workbench/projects/Torque_Control/Src/pwm_curr_fdbk.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F302x8 -c -I../../Inc -I"C:/Users/louib/Desktop/KTH Aerospace/Thesis Project/3) MATLAB Simulink Electronics/3_5) Simulink/Torque Controller/Torque_Controller" -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.1.2-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.1.2-Full/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/register_interface.o: C:/Users/louib/.st_workbench/projects/Torque_Control/Src/register_interface.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F302x8 -c -I../../Inc -I"C:/Users/louib/Desktop/KTH Aerospace/Thesis Project/3) MATLAB Simulink Electronics/3_5) Simulink/Torque Controller/Torque_Controller" -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.1.2-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.1.2-Full/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/regular_conversion_manager.o: C:/Users/louib/.st_workbench/projects/Torque_Control/Src/regular_conversion_manager.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F302x8 -c -I../../Inc -I"C:/Users/louib/Desktop/KTH Aerospace/Thesis Project/3) MATLAB Simulink Electronics/3_5) Simulink/Torque Controller/Torque_Controller" -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.1.2-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.1.2-Full/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/stm32f30x_mc_it.o: C:/Users/louib/.st_workbench/projects/Torque_Control/Src/stm32f30x_mc_it.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F302x8 -c -I../../Inc -I"C:/Users/louib/Desktop/KTH Aerospace/Thesis Project/3) MATLAB Simulink Electronics/3_5) Simulink/Torque Controller/Torque_Controller" -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.1.2-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.1.2-Full/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/stm32f3xx_hal_msp.o: C:/Users/louib/.st_workbench/projects/Torque_Control/Src/stm32f3xx_hal_msp.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F302x8 -c -I../../Inc -I"C:/Users/louib/Desktop/KTH Aerospace/Thesis Project/3) MATLAB Simulink Electronics/3_5) Simulink/Torque Controller/Torque_Controller" -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.1.2-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.1.2-Full/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/stm32f3xx_it.o: C:/Users/louib/.st_workbench/projects/Torque_Control/Src/stm32f3xx_it.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F302x8 -c -I../../Inc -I"C:/Users/louib/Desktop/KTH Aerospace/Thesis Project/3) MATLAB Simulink Electronics/3_5) Simulink/Torque Controller/Torque_Controller" -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.1.2-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.1.2-Full/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/%.o Application/User/%.su Application/User/%.cyclo: ../Application/User/%.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F302x8 -c -I../../Inc -I"C:/Users/louib/Desktop/KTH Aerospace/Thesis Project/3) MATLAB Simulink Electronics/3_5) Simulink/Torque Controller/Torque_Controller" -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.1.2-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.1.2-Full/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Application-2f-User

clean-Application-2f-User:
	-$(RM) ./Application/User/main.cyclo ./Application/User/main.d ./Application/User/main.o ./Application/User/main.su ./Application/User/mc_api.cyclo ./Application/User/mc_api.d ./Application/User/mc_api.o ./Application/User/mc_api.su ./Application/User/mc_app_hooks.cyclo ./Application/User/mc_app_hooks.d ./Application/User/mc_app_hooks.o ./Application/User/mc_app_hooks.su ./Application/User/mc_config.cyclo ./Application/User/mc_config.d ./Application/User/mc_config.o ./Application/User/mc_config.su ./Application/User/mc_configuration_registers.cyclo ./Application/User/mc_configuration_registers.d ./Application/User/mc_configuration_registers.o ./Application/User/mc_configuration_registers.su ./Application/User/mc_interface.cyclo ./Application/User/mc_interface.d ./Application/User/mc_interface.o ./Application/User/mc_interface.su ./Application/User/mc_math.cyclo ./Application/User/mc_math.d ./Application/User/mc_math.o ./Application/User/mc_math.su ./Application/User/mc_parameters.cyclo ./Application/User/mc_parameters.d ./Application/User/mc_parameters.o ./Application/User/mc_parameters.su ./Application/User/mc_perf.cyclo ./Application/User/mc_perf.d ./Application/User/mc_perf.o ./Application/User/mc_perf.su ./Application/User/mc_tasks.cyclo ./Application/User/mc_tasks.d ./Application/User/mc_tasks.o ./Application/User/mc_tasks.su ./Application/User/motorcontrol.cyclo ./Application/User/motorcontrol.d ./Application/User/motorcontrol.o ./Application/User/motorcontrol.su ./Application/User/pwm_curr_fdbk.cyclo ./Application/User/pwm_curr_fdbk.d ./Application/User/pwm_curr_fdbk.o ./Application/User/pwm_curr_fdbk.su ./Application/User/register_interface.cyclo ./Application/User/register_interface.d ./Application/User/register_interface.o ./Application/User/register_interface.su ./Application/User/regular_conversion_manager.cyclo ./Application/User/regular_conversion_manager.d ./Application/User/regular_conversion_manager.o ./Application/User/regular_conversion_manager.su ./Application/User/stm32f30x_mc_it.cyclo ./Application/User/stm32f30x_mc_it.d ./Application/User/stm32f30x_mc_it.o ./Application/User/stm32f30x_mc_it.su ./Application/User/stm32f3xx_hal_msp.cyclo ./Application/User/stm32f3xx_hal_msp.d ./Application/User/stm32f3xx_hal_msp.o ./Application/User/stm32f3xx_hal_msp.su ./Application/User/stm32f3xx_it.cyclo ./Application/User/stm32f3xx_it.d ./Application/User/stm32f3xx_it.o ./Application/User/stm32f3xx_it.su ./Application/User/syscalls.cyclo ./Application/User/syscalls.d ./Application/User/syscalls.o ./Application/User/syscalls.su ./Application/User/sysmem.cyclo ./Application/User/sysmem.d ./Application/User/sysmem.o ./Application/User/sysmem.su

.PHONY: clean-Application-2f-User

