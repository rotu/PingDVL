################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/dvl_filter.cpp \
../src/geometry.cpp \
../src/main.cpp 

OBJS += \
./src/dvl_filter.o \
./src/geometry.o \
./src/main.o 

CPP_DEPS += \
./src/dvl_filter.d \
./src/geometry.d \
./src/main.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	g++ -I"C:\Users\dan\Documents\PingDVL\src\TinyEKF" -I"C:\Users\dan\Documents\PingDVL\src\TinyEKF\src" -O0 -g3 -Wall -c -fmessage-length=0 -std=gnu++14 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"

