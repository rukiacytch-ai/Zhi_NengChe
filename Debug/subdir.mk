################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
"../test.c" 

COMPILED_SRCS += \
"test.src" 

C_DEPS += \
"./test.d" 

OBJS += \
"test.o" 


# Each subdirectory must supply rules for building sources it contributes
"test.src":"../test.c" "subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2012 -D__CPU__=tc26xb "-fD:/Infineon/AURIX-Studio-1.10.32/AURIX-Studio-1.10.32-workspace/Seekfree_TC264_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"test.o":"test.src" "subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean--2e-

clean--2e-:
	-$(RM) ./test.d ./test.o ./test.src

.PHONY: clean--2e-

