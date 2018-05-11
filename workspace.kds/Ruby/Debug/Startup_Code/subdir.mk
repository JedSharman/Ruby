################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Startup_Code/console.cpp \
../Startup_Code/mcg.cpp \
../Startup_Code/rtc.cpp \
../Startup_Code/system-gcc.cpp \
../Startup_Code/system.cpp \
../Startup_Code/vectors.cpp 

C_SRCS += \
../Startup_Code/newlib_stubs.c \
../Startup_Code/security.c 

S_UPPER_SRCS += \
../Startup_Code/startup_ARMLtdGCC.S 

OBJS += \
./Startup_Code/console.o \
./Startup_Code/mcg.o \
./Startup_Code/newlib_stubs.o \
./Startup_Code/rtc.o \
./Startup_Code/security.o \
./Startup_Code/startup_ARMLtdGCC.o \
./Startup_Code/system-gcc.o \
./Startup_Code/system.o \
./Startup_Code/vectors.o 

C_DEPS += \
./Startup_Code/newlib_stubs.d \
./Startup_Code/security.d 

CPP_DEPS += \
./Startup_Code/console.d \
./Startup_Code/mcg.d \
./Startup_Code/rtc.d \
./Startup_Code/system-gcc.d \
./Startup_Code/system.d \
./Startup_Code/vectors.d 


# Each subdirectory must supply rules for building sources it contributes
Startup_Code/%.o: ../Startup_Code/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: ARM C++ Compiler'
	arm-none-eabi-g++ -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -g3 -O0 -ffunction-sections -fdata-sections -fno-rtti -Wall -Wextra -DDEBUG_BUILD -I"C:/Users/jaeha/workspace.kds/Ruby/Sources" -I"C:/Users/jaeha/workspace.kds/Ruby/Project_Headers" -fno-exceptions -std=gnu++11 -c -fmessage-length=0 -MT"$@" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)"  -o "$@" $<
	@echo 'Finished building: $<'
	@echo ' '

Startup_Code/%.o: ../Startup_Code/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -g3 -O0 -ffunction-sections -fdata-sections -Wall -Wextra -DDEBUG_BUILD -I"C:/Users/jaeha/workspace.kds/Ruby/Sources" -I"C:/Users/jaeha/workspace.kds/Ruby/Project_Headers" -std=c11 -c -fmessage-length=0 -MT"$@" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)"  -o "$@" $<
	@echo 'Finished building: $<'
	@echo ' '

Startup_Code/%.o: ../Startup_Code/%.S
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Assembler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -g3 -O0 -ffunction-sections -fdata-sections -Wall -Wextra -x assembler-with-cpp -DDEBUG_BUILD -I"C:/Users/jaeha/workspace.kds/Ruby/Project_Headers" -Wall -Wextra -c -fmessage-length=0  -o "$@" $<
	@echo 'Finished building: $<'
	@echo ' '


