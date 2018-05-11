################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Sources/Ruby.cpp \
../Sources/delay.cpp \
../Sources/ftfe.cpp \
../Sources/hardware.cpp \
../Sources/i2c.cpp \
../Sources/spi.cpp \
../Sources/usbdmError.cpp 

OBJS += \
./Sources/Ruby.o \
./Sources/delay.o \
./Sources/ftfe.o \
./Sources/hardware.o \
./Sources/i2c.o \
./Sources/spi.o \
./Sources/usbdmError.o 

CPP_DEPS += \
./Sources/Ruby.d \
./Sources/delay.d \
./Sources/ftfe.d \
./Sources/hardware.d \
./Sources/i2c.d \
./Sources/spi.d \
./Sources/usbdmError.d 


# Each subdirectory must supply rules for building sources it contributes
Sources/%.o: ../Sources/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: ARM C++ Compiler'
	arm-none-eabi-g++ -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -g3 -O0 -ffunction-sections -fdata-sections -fno-rtti -Wall -Wextra -DDEBUG_BUILD -I"C:/Users/jaeha/workspace.kds/Ruby/Sources" -I"C:/Users/jaeha/workspace.kds/Ruby/Project_Headers" -fno-exceptions -std=gnu++11 -c -fmessage-length=0 -MT"$@" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)"  -o "$@" $<
	@echo 'Finished building: $<'
	@echo ' '


