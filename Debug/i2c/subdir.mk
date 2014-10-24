################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../i2c/i2c.c \
../i2c/i2c_driver.c 

OBJS += \
./i2c/i2c.o \
./i2c/i2c_driver.o 

C_DEPS += \
./i2c/i2c.d \
./i2c/i2c_driver.d 


# Each subdirectory must supply rules for building sources it contributes
i2c/%.o: ../i2c/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	avr-gcc -Wall -g2 -gstabs -O0 -fpack-struct -fshort-enums -ffunction-sections -fdata-sections -std=gnu99 -funsigned-char -funsigned-bitfields -mmcu=atmega328p -DF_CPU=8000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


