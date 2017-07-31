################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../CMSIS/system_MK63F12.c 

OBJS += \
./CMSIS/system_MK63F12.o 

C_DEPS += \
./CMSIS/system_MK63F12.d 


# Each subdirectory must supply rules for building sources it contributes
CMSIS/%.o: ../CMSIS/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -DSDK_DEBUGCONSOLE=0 -D__MCUXPRESSO -D__USE_CMSIS -DDEBUG -DSDK_OS_FREE_RTOS -DFSL_RTOS_FREE_RTOS -DCPU_MK63FN1M0VLQ12 -DCPU_MK63FN1M0VLQ12_cm4 -D__NEWLIB__ -I"C:\Users\nhoch\Documents\MCUXpressoIDE_10.0.0_344\workspace\DC2017\source" -I"C:\Users\nhoch\Documents\MCUXpressoIDE_10.0.0_344\workspace\DC2017" -I"C:\Users\nhoch\Documents\MCUXpressoIDE_10.0.0_344\workspace\DC2017\drivers" -I"C:\Users\nhoch\Documents\MCUXpressoIDE_10.0.0_344\workspace\DC2017\freertos" -I"C:\Users\nhoch\Documents\MCUXpressoIDE_10.0.0_344\workspace\DC2017\startup" -I"C:\Users\nhoch\Documents\MCUXpressoIDE_10.0.0_344\workspace\DC2017\CMSIS" -I"C:\Users\nhoch\Documents\MCUXpressoIDE_10.0.0_344\workspace\DC2017\utilities" -O0 -fno-common -g3 -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -D__NEWLIB__ -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


