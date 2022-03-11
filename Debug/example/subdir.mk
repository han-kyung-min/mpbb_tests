################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../example/mpbb_run.cpp 

OBJS += \
./example/mpbb_run.o 

CPP_DEPS += \
./example/mpbb_run.d 


# Each subdirectory must supply rules for building sources it contributes
example/%.o: ../example/%.cpp example/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/home/hankm/binary_ws/mpbb_tests/include -include/home/hankm/binary_ws/mpbb_tests/include -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


