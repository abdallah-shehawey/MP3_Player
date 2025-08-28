################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/EXTI_program.c \
../Src/GPIO_prog.c \
../Src/IR_Program.c \
../Src/NVIC_program.c \
../Src/RCC_program.c \
../Src/SYSCFG_program.c \
../Src/SYSTIC_program.c \
../Src/SYS_init.c \
../Src/USART_program.c \
../Src/main.c \
../Src/syscalls.c \
../Src/sysmem.c 

OBJS += \
./Src/EXTI_program.o \
./Src/GPIO_prog.o \
./Src/IR_Program.o \
./Src/NVIC_program.o \
./Src/RCC_program.o \
./Src/SYSCFG_program.o \
./Src/SYSTIC_program.o \
./Src/SYS_init.o \
./Src/USART_program.o \
./Src/main.o \
./Src/syscalls.o \
./Src/sysmem.o 

C_DEPS += \
./Src/EXTI_program.d \
./Src/GPIO_prog.d \
./Src/IR_Program.d \
./Src/NVIC_program.d \
./Src/RCC_program.d \
./Src/SYSCFG_program.d \
./Src/SYSTIC_program.d \
./Src/SYS_init.d \
./Src/USART_program.d \
./Src/main.d \
./Src/syscalls.d \
./Src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F4 -DSTM32F446RETx -DNUCLEO_F446RE -c -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/EXTI_program.cyclo ./Src/EXTI_program.d ./Src/EXTI_program.o ./Src/EXTI_program.su ./Src/GPIO_prog.cyclo ./Src/GPIO_prog.d ./Src/GPIO_prog.o ./Src/GPIO_prog.su ./Src/IR_Program.cyclo ./Src/IR_Program.d ./Src/IR_Program.o ./Src/IR_Program.su ./Src/NVIC_program.cyclo ./Src/NVIC_program.d ./Src/NVIC_program.o ./Src/NVIC_program.su ./Src/RCC_program.cyclo ./Src/RCC_program.d ./Src/RCC_program.o ./Src/RCC_program.su ./Src/SYSCFG_program.cyclo ./Src/SYSCFG_program.d ./Src/SYSCFG_program.o ./Src/SYSCFG_program.su ./Src/SYSTIC_program.cyclo ./Src/SYSTIC_program.d ./Src/SYSTIC_program.o ./Src/SYSTIC_program.su ./Src/SYS_init.cyclo ./Src/SYS_init.d ./Src/SYS_init.o ./Src/SYS_init.su ./Src/USART_program.cyclo ./Src/USART_program.d ./Src/USART_program.o ./Src/USART_program.su ./Src/main.cyclo ./Src/main.d ./Src/main.o ./Src/main.su ./Src/syscalls.cyclo ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.cyclo ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su

.PHONY: clean-Src

