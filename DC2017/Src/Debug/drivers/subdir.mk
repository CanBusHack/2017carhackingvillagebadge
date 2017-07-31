################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/fsl_clock.c \
../drivers/fsl_common.c \
../drivers/fsl_crc.c \
../drivers/fsl_enet.c \
../drivers/fsl_flash.c \
../drivers/fsl_flexcan.c \
../drivers/fsl_ftm.c \
../drivers/fsl_gpio.c \
../drivers/fsl_i2c.c \
../drivers/fsl_i2c_freertos.c \
../drivers/fsl_pit.c \
../drivers/fsl_sdhc.c \
../drivers/fsl_uart.c \
../drivers/fsl_uart_freertos.c \
../drivers/fsl_wdog.c 

OBJS += \
./drivers/fsl_clock.o \
./drivers/fsl_common.o \
./drivers/fsl_crc.o \
./drivers/fsl_enet.o \
./drivers/fsl_flash.o \
./drivers/fsl_flexcan.o \
./drivers/fsl_ftm.o \
./drivers/fsl_gpio.o \
./drivers/fsl_i2c.o \
./drivers/fsl_i2c_freertos.o \
./drivers/fsl_pit.o \
./drivers/fsl_sdhc.o \
./drivers/fsl_uart.o \
./drivers/fsl_uart_freertos.o \
./drivers/fsl_wdog.o 

C_DEPS += \
./drivers/fsl_clock.d \
./drivers/fsl_common.d \
./drivers/fsl_crc.d \
./drivers/fsl_enet.d \
./drivers/fsl_flash.d \
./drivers/fsl_flexcan.d \
./drivers/fsl_ftm.d \
./drivers/fsl_gpio.d \
./drivers/fsl_i2c.d \
./drivers/fsl_i2c_freertos.d \
./drivers/fsl_pit.d \
./drivers/fsl_sdhc.d \
./drivers/fsl_uart.d \
./drivers/fsl_uart_freertos.d \
./drivers/fsl_wdog.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/%.o: ../drivers/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -DSDK_DEBUGCONSOLE=0 -D__MCUXPRESSO -D__USE_CMSIS -DDEBUG -DSDK_OS_FREE_RTOS -DFSL_RTOS_FREE_RTOS -DCPU_MK63FN1M0VLQ12 -DCPU_MK63FN1M0VLQ12_cm4 -D__NEWLIB__ -I"C:\Users\nhoch\Documents\MCUXpressoIDE_10.0.0_344\workspace\DC2017\source" -I"C:\Users\nhoch\Documents\MCUXpressoIDE_10.0.0_344\workspace\DC2017" -I"C:\Users\nhoch\Documents\MCUXpressoIDE_10.0.0_344\workspace\DC2017\drivers" -I"C:\Users\nhoch\Documents\MCUXpressoIDE_10.0.0_344\workspace\DC2017\freertos" -I"C:\Users\nhoch\Documents\MCUXpressoIDE_10.0.0_344\workspace\DC2017\startup" -I"C:\Users\nhoch\Documents\MCUXpressoIDE_10.0.0_344\workspace\DC2017\CMSIS" -I"C:\Users\nhoch\Documents\MCUXpressoIDE_10.0.0_344\workspace\DC2017\utilities" -O0 -fno-common -g3 -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -D__NEWLIB__ -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


