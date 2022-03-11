################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CXX_SRCS += \
../build/CMakeFiles/feature_tests.cxx 

C_SRCS += \
../build/CMakeFiles/feature_tests.c 

CXX_DEPS += \
./build/CMakeFiles/feature_tests.d 

OBJS += \
./build/CMakeFiles/feature_tests.o 

C_DEPS += \
./build/CMakeFiles/feature_tests.d 


# Each subdirectory must supply rules for building sources it contributes
build/CMakeFiles/%.o: ../build/CMakeFiles/%.c build/CMakeFiles/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -I/home/hankm/binary_ws/mpbb_tests/include -include/home/hankm/binary_ws/mpbb_tests/include -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

build/CMakeFiles/%.o: ../build/CMakeFiles/%.cxx build/CMakeFiles/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/home/hankm/binary_ws/mpbb_tests/include -include/home/hankm/binary_ws/mpbb_tests/include -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


