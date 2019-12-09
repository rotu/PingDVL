################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/TinyEKF/src/tiny_ekf.c 

OBJS += \
./src/TinyEKF/src/tiny_ekf.o 

C_DEPS += \
./src/TinyEKF/src/tiny_ekf.d 


# Each subdirectory must supply rules for building sources it contributes
src/TinyEKF/src/%.o: ../src/TinyEKF/src/%.c
	gcc -I"C:\Users\dan\Documents\PingDVL\src\TinyEKF" -I"C:\Users\dan\Documents\PingDVL\src\TinyEKF\src" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"

