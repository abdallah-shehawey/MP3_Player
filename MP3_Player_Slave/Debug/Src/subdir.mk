################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/GPIO_prog.c \
../Src/NVIC_program.c \
../Src/RCC_program.c \
../Src/SPI_program.c \
../Src/SYSCFG_program.c \
../Src/SYSTIC_program.c \
../Src/SYS_init.c \
../Src/TFT_program.c \
../Src/USART_program.c \
../Src/main.c \
../Src/syscalls.c \
../Src/sysmem.c 

OBJS += \
./Src/GPIO_prog.o \
./Src/NVIC_program.o \
./Src/RCC_program.o \
./Src/SPI_program.o \
./Src/SYSCFG_program.o \
./Src/SYSTIC_program.o \
./Src/SYS_init.o \
./Src/TFT_program.o \
./Src/USART_program.o \
./Src/main.o \
./Src/syscalls.o \
./Src/sysmem.o 

C_DEPS += \
./Src/GPIO_prog.d \
./Src/NVIC_program.d \
./Src/RCC_program.d \
./Src/SPI_program.d \
./Src/SYSCFG_program.d \
./Src/SYSTIC_program.d \
./Src/SYS_init.d \
./Src/TFT_program.d \
./Src/USART_program.d \
./Src/main.d \
./Src/syscalls.d \
./Src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32F401CCUx -DSTM32 -DSTM32F4 -c -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/GPIO_prog.cyclo ./Src/GPIO_prog.d ./Src/GPIO_prog.o ./Src/GPIO_prog.su ./Src/NVIC_program.cyclo ./Src/NVIC_program.d ./Src/NVIC_program.o ./Src/NVIC_program.su ./Src/RCC_program.cyclo ./Src/RCC_program.d ./Src/RCC_program.o ./Src/RCC_program.su ./Src/SPI_program.cyclo ./Src/SPI_program.d ./Src/SPI_program.o ./Src/SPI_program.su ./Src/SYSCFG_program.cyclo ./Src/SYSCFG_program.d ./Src/SYSCFG_program.o ./Src/SYSCFG_program.su ./Src/SYSTIC_program.cyclo ./Src/SYSTIC_program.d ./Src/SYSTIC_program.o ./Src/SYSTIC_program.su ./Src/SYS_init.cyclo ./Src/SYS_init.d ./Src/SYS_init.o ./Src/SYS_init.su ./Src/TFT_program.cyclo ./Src/TFT_program.d ./Src/TFT_program.o ./Src/TFT_program.su ./Src/USART_program.cyclo ./Src/USART_program.d ./Src/USART_program.o ./Src/USART_program.su ./Src/main.cyclo ./Src/main.d ./Src/main.o ./Src/main.su ./Src/syscalls.cyclo ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.cyclo ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su

.PHONY: clean-Src

