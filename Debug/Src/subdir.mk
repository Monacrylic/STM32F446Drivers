################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/004_Spi_tx_testing.c 

OBJS += \
./Src/004_Spi_tx_testing.o 

C_DEPS += \
./Src/004_Spi_tx_testing.d 


# Each subdirectory must supply rules for building sources it contributes
Src/004_Spi_tx_testing.o: ../Src/004_Spi_tx_testing.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DDEBUG -DNUCLEO_F446RE -c -I"C:/Users/siddk/STM32CubeIDE/workspace_1.2.0/stm32f446xx_drivers/drivers/inc" -I"C:/Users/siddk/STM32CubeIDE/workspace_1.2.0/stm32f446xx_drivers/drivers/src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/004_Spi_tx_testing.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

