################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
"../code/Car.c" \
"../code/Encoder.c" \
"../code/Kalman_Gyro.c" \
"../code/LED.c" \
"../code/PID.c" \
"../code/Switch.c" \
"../code/figure.c" \
"../code/scan_line.c" 

COMPILED_SRCS += \
"code/Car.src" \
"code/Encoder.src" \
"code/Kalman_Gyro.src" \
"code/LED.src" \
"code/PID.src" \
"code/Switch.src" \
"code/figure.src" \
"code/scan_line.src" 

C_DEPS += \
"./code/Car.d" \
"./code/Encoder.d" \
"./code/Kalman_Gyro.d" \
"./code/LED.d" \
"./code/PID.d" \
"./code/Switch.d" \
"./code/figure.d" \
"./code/scan_line.d" 

OBJS += \
"code/Car.o" \
"code/Encoder.o" \
"code/Kalman_Gyro.o" \
"code/LED.o" \
"code/PID.o" \
"code/Switch.o" \
"code/figure.o" \
"code/scan_line.o" 


# Each subdirectory must supply rules for building sources it contributes
"code/Car.src":"../code/Car.c" "code/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2012 -D__CPU__=tc26xb "-fD:/Infineon/AURIX-Studio-1.10.32/AURIX-Studio-1.10.32-workspace/Seekfree_TC264_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"code/Car.o":"code/Car.src" "code/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/Encoder.src":"../code/Encoder.c" "code/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2012 -D__CPU__=tc26xb "-fD:/Infineon/AURIX-Studio-1.10.32/AURIX-Studio-1.10.32-workspace/Seekfree_TC264_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"code/Encoder.o":"code/Encoder.src" "code/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/Kalman_Gyro.src":"../code/Kalman_Gyro.c" "code/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2012 -D__CPU__=tc26xb "-fD:/Infineon/AURIX-Studio-1.10.32/AURIX-Studio-1.10.32-workspace/Seekfree_TC264_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"code/Kalman_Gyro.o":"code/Kalman_Gyro.src" "code/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/LED.src":"../code/LED.c" "code/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2012 -D__CPU__=tc26xb "-fD:/Infineon/AURIX-Studio-1.10.32/AURIX-Studio-1.10.32-workspace/Seekfree_TC264_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"code/LED.o":"code/LED.src" "code/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/PID.src":"../code/PID.c" "code/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2012 -D__CPU__=tc26xb "-fD:/Infineon/AURIX-Studio-1.10.32/AURIX-Studio-1.10.32-workspace/Seekfree_TC264_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"code/PID.o":"code/PID.src" "code/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/Switch.src":"../code/Switch.c" "code/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2012 -D__CPU__=tc26xb "-fD:/Infineon/AURIX-Studio-1.10.32/AURIX-Studio-1.10.32-workspace/Seekfree_TC264_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"code/Switch.o":"code/Switch.src" "code/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/figure.src":"../code/figure.c" "code/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2012 -D__CPU__=tc26xb "-fD:/Infineon/AURIX-Studio-1.10.32/AURIX-Studio-1.10.32-workspace/Seekfree_TC264_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"code/figure.o":"code/figure.src" "code/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/scan_line.src":"../code/scan_line.c" "code/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2012 -D__CPU__=tc26xb "-fD:/Infineon/AURIX-Studio-1.10.32/AURIX-Studio-1.10.32-workspace/Seekfree_TC264_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"code/scan_line.o":"code/scan_line.src" "code/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-code

clean-code:
	-$(RM) ./code/Car.d ./code/Car.o ./code/Car.src ./code/Encoder.d ./code/Encoder.o ./code/Encoder.src ./code/Kalman_Gyro.d ./code/Kalman_Gyro.o ./code/Kalman_Gyro.src ./code/LED.d ./code/LED.o ./code/LED.src ./code/PID.d ./code/PID.o ./code/PID.src ./code/Switch.d ./code/Switch.o ./code/Switch.src ./code/figure.d ./code/figure.o ./code/figure.src ./code/scan_line.d ./code/scan_line.o ./code/scan_line.src

.PHONY: clean-code

