################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/src/stm32f446xx_gpio_driver.c \
../drivers/src/stm32f446xx_rcc_driver.c \
../drivers/src/stm32f446xx_spi_driver.c \
../drivers/src/stm32f446xx_usart_driver.c 

OBJS += \
./drivers/src/stm32f446xx_gpio_driver.o \
./drivers/src/stm32f446xx_rcc_driver.o \
./drivers/src/stm32f446xx_spi_driver.o \
./drivers/src/stm32f446xx_usart_driver.o 

C_DEPS += \
./drivers/src/stm32f446xx_gpio_driver.d \
./drivers/src/stm32f446xx_rcc_driver.d \
./drivers/src/stm32f446xx_spi_driver.d \
./drivers/src/stm32f446xx_usart_driver.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/src/stm32f446xx_gpio_driver.o: ../drivers/src/stm32f446xx_gpio_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DDEBUG -DNUCLEO_F446RE -c -I"C:/Users/siddk/STM32CubeIDE/workspace_1.2.0/stm32f446xx_drivers/drivers/inc" -I"C:/Users/siddk/STM32CubeIDE/workspace_1.2.0/stm32f446xx_drivers/drivers/src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/src/stm32f446xx_gpio_driver.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
drivers/src/stm32f446xx_rcc_driver.o: ../drivers/src/stm32f446xx_rcc_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DDEBUG -DNUCLEO_F446RE -c -I"C:/Users/siddk/STM32CubeIDE/workspace_1.2.0/stm32f446xx_drivers/drivers/inc" -I"C:/Users/siddk/STM32CubeIDE/workspace_1.2.0/stm32f446xx_drivers/drivers/src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/src/stm32f446xx_rcc_driver.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
drivers/src/stm32f446xx_spi_driver.o: ../drivers/src/stm32f446xx_spi_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DDEBUG -DNUCLEO_F446RE -c -I"C:/Users/siddk/STM32CubeIDE/workspace_1.2.0/stm32f446xx_drivers/drivers/inc" -I"C:/Users/siddk/STM32CubeIDE/workspace_1.2.0/stm32f446xx_drivers/drivers/src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/src/stm32f446xx_spi_driver.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
drivers/src/stm32f446xx_usart_driver.o: ../drivers/src/stm32f446xx_usart_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DDEBUG -DNUCLEO_F446RE -c -I"C:/Users/siddk/STM32CubeIDE/workspace_1.2.0/stm32f446xx_drivers/drivers/inc" -I"C:/Users/siddk/STM32CubeIDE/workspace_1.2.0/stm32f446xx_drivers/drivers/src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/src/stm32f446xx_usart_driver.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

