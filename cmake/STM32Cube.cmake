cmake_minimum_required(VERSION 3.22)

add_library(stm32cubemx INTERFACE)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

target_compile_definitions(stm32cubemx INTERFACE 
	USE_HAL_DRIVER 
	STM32F103xB
)

target_include_directories(stm32cubemx INTERFACE
    Core/Inc
    Modules/STM32F1xx_HAL_Driver/Inc
    Modules/STM32F1xx_HAL_Driver/Inc/Legacy
    Modules/CMSIS_device_F1/Include
    Drivers/CMSIS/Include
)

target_sources(stm32cubemx INTERFACE
    Core/Src/main.c
    Core/Src/gpio.c
    Core/Src/dma.c
    Core/Src/i2c.c
    Core/Src/tim.c
    Core/Src/usart.c
    Core/Src/stm32f1xx_it.c
    Core/Src/stm32f1xx_hal_msp.c
    Core/Src/system_stm32f1xx.c
    Core/Src/sysmem.c
    Core/Src/syscalls.c
    Modules/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio_ex.c
    Modules/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_i2c.c
    Modules/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c
    Modules/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c
    Modules/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc_ex.c
    Modules/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.c
    Modules/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_dma.c
    Modules/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c
    Modules/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pwr.c
    Modules/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash.c
    Modules/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash_ex.c
    Modules/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_exti.c
    Modules/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.c
    Modules/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim_ex.c
    Modules/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_uart.c
    Modules/CMSIS_device_F1/Source/Templates/gcc/startup_stm32f103xb.s
)

target_link_directories(stm32cubemx INTERFACE
)

target_link_libraries(stm32cubemx INTERFACE
)

# Validate that STM32CubeMX code is compatible with C standard
if(CMAKE_C_STANDARD LESS 11)
    message(ERROR "Generated code requires C11 or higher")
endif()


