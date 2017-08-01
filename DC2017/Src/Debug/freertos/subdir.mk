################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../freertos/croutine.c \
../freertos/event_groups.c \
../freertos/fsl_tickless_lptmr.c \
../freertos/fsl_tickless_systick.c \
../freertos/heap_4.c \
../freertos/list.c \
../freertos/port.c \
../freertos/queue.c \
../freertos/tasks.c \
../freertos/timers.c 

OBJS += \
./freertos/croutine.o \
./freertos/event_groups.o \
./freertos/fsl_tickless_lptmr.o \
./freertos/fsl_tickless_systick.o \
./freertos/heap_4.o \
./freertos/list.o \
./freertos/port.o \
./freertos/queue.o \
./freertos/tasks.o \
./freertos/timers.o 

C_DEPS += \
./freertos/croutine.d \
./freertos/event_groups.d \
./freertos/fsl_tickless_lptmr.d \
./freertos/fsl_tickless_systick.d \
./freertos/heap_4.d \
./freertos/list.d \
./freertos/port.d \
./freertos/queue.d \
./freertos/tasks.d \
./freertos/timers.d 


# Each subdirectory must supply rules for building sources it contributes
freertos/%.o: ../freertos/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -DSDK_DEBUGCONSOLE=0 -D__MCUXPRESSO -D__USE_CMSIS -DDEBUG -DSDK_OS_FREE_RTOS -DFSL_RTOS_FREE_RTOS -DCPU_MK63FN1M0VLQ12 -DCPU_MK63FN1M0VLQ12_cm4 -D__NEWLIB__ -I"C:\Users\nhoch\Documents\MCUXpressoIDE_10.0.0_344\workspace\DC2017\source" -I"C:\Users\nhoch\Documents\MCUXpressoIDE_10.0.0_344\workspace\DC2017" -I"C:\Users\nhoch\Documents\MCUXpressoIDE_10.0.0_344\workspace\DC2017\drivers" -I"C:\Users\nhoch\Documents\MCUXpressoIDE_10.0.0_344\workspace\DC2017\freertos" -I"C:\Users\nhoch\Documents\MCUXpressoIDE_10.0.0_344\workspace\DC2017\startup" -I"C:\Users\nhoch\Documents\MCUXpressoIDE_10.0.0_344\workspace\DC2017\CMSIS" -I"C:\Users\nhoch\Documents\MCUXpressoIDE_10.0.0_344\workspace\DC2017\utilities" -O0 -fno-common -g3 -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -D__NEWLIB__ -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


