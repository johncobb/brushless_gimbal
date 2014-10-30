################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../imu/gyro.c \
../imu/imu.c \
../imu/mpu6050.c 

OBJS += \
./imu/gyro.o \
./imu/imu.o \
./imu/mpu6050.o 

C_DEPS += \
./imu/gyro.d \
./imu/imu.d \
./imu/mpu6050.d 


# Each subdirectory must supply rules for building sources it contributes
imu/%.o: ../imu/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	avr-gcc -Wall -Os -fpack-struct -fshort-enums -ffunction-sections -fdata-sections -std=gnu99 -funsigned-char -funsigned-bitfields -mmcu=atmega328p -DF_CPU=8000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


