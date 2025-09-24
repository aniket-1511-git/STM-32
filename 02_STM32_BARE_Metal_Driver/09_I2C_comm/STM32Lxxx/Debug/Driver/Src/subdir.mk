################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Driver/Src/STM32Lxx_I2C_Driver.c \
../Driver/Src/STM32Lxx_SPI_Driver.c \
../Driver/Src/STM32Lxx_USART_Driver.c \
../Driver/Src/STM32Lxx_gpio_driver.c 

OBJS += \
./Driver/Src/STM32Lxx_I2C_Driver.o \
./Driver/Src/STM32Lxx_SPI_Driver.o \
./Driver/Src/STM32Lxx_USART_Driver.o \
./Driver/Src/STM32Lxx_gpio_driver.o 

C_DEPS += \
./Driver/Src/STM32Lxx_I2C_Driver.d \
./Driver/Src/STM32Lxx_SPI_Driver.d \
./Driver/Src/STM32Lxx_USART_Driver.d \
./Driver/Src/STM32Lxx_gpio_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Driver/Src/%.o Driver/Src/%.su Driver/Src/%.cyclo: ../Driver/Src/%.c Driver/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g -DDEBUG -DSTM32L4 -DSTM32 -DNUCLEO_L476RG -DSTM32L476RGTx -c -I../Inc -I"D:/BitSilica/STM_PRO/STM32Lxxx/Driver/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Driver-2f-Src

clean-Driver-2f-Src:
	-$(RM) ./Driver/Src/STM32Lxx_I2C_Driver.cyclo ./Driver/Src/STM32Lxx_I2C_Driver.d ./Driver/Src/STM32Lxx_I2C_Driver.o ./Driver/Src/STM32Lxx_I2C_Driver.su ./Driver/Src/STM32Lxx_SPI_Driver.cyclo ./Driver/Src/STM32Lxx_SPI_Driver.d ./Driver/Src/STM32Lxx_SPI_Driver.o ./Driver/Src/STM32Lxx_SPI_Driver.su ./Driver/Src/STM32Lxx_USART_Driver.cyclo ./Driver/Src/STM32Lxx_USART_Driver.d ./Driver/Src/STM32Lxx_USART_Driver.o ./Driver/Src/STM32Lxx_USART_Driver.su ./Driver/Src/STM32Lxx_gpio_driver.cyclo ./Driver/Src/STM32Lxx_gpio_driver.d ./Driver/Src/STM32Lxx_gpio_driver.o ./Driver/Src/STM32Lxx_gpio_driver.su

.PHONY: clean-Driver-2f-Src

