cmake_minimum_required(VERSION 3.25)

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# Include target creation file
include("cmake/createTarget.cmake")

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Core project settings
project("ParkingSensor")
message("Build type: " ${CMAKE_BUILD_TYPE})

# Core MCU flags, CPU type, instruction set and FPU setup
set(cpu_PARAMS     
    -mcpu=cortex-m3
    -mfloat-abi=soft
    -mthumb
)

# Set user definitions
set(user_DEFS
    ADVUTILS_USE_DYNAMIC_ALLOCATION
    $<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:ASM>>:DEBUG>
    $<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:C>>:DEBUG>
)

# Compiler options
set(compiler_OPTS
    -fdiagnostics-color=always
    -Wall
    -Wextra
    -Wpedantic
    -Wno-unused-parameter
    $<$<CONFIG:Debug>:-Og -g3>
    $<$<AND:$<CONFIG:Release>,$<COMPILE_LANGUAGE:ASM>>:-g0>
    $<$<AND:$<CONFIG:Release>,$<COMPILE_LANGUAGE:C>>:-O2 -g0>
    $<$<AND:$<CONFIG:Release>,$<COMPILE_LANGUAGE:CXX>>:-Os -g0>
)

# Linker options
set(linker_OPTS
    #-u _printf_float # STDIO float formatting support (remove if not used)
    -static
    -flto
    -Wl,--start-group
    -lc
    -lm
    -lstdc++
    -lsupc++
    -Wl,--end-group
    #-Wl,-z,max-page-size=8 # Allow good software remapping across address space (with proper GCC section making)
    -Wl,--print-memory-usage
)

# Add STM32CubeMX library
add_subdirectory(cmake/stm32cubemx)

# Add ADVutils library
set (ADVUtils_COMPILE_OPTS
    ${cpu_PARAMS}
    ${compiler_OPTS}
)
set (ADVUtils_COMPILE_DEFS
    ${user_DEFS}
)
add_subdirectory(Modules/Utils)

# Add miniPrintf library
set(miniPrintf_COMPILE_DEFS
    MINIPRINTF_USE_NANO_VERSION
    MINIPRINTF_USE_UART_DMA
    MINIPRINTF_UART_HANDLE=&huart1
    PRINTF_DEFAULT_FLOAT_PRECISION=1
    PRINTF_MAX_INTEGRAL_DIGITS_FOR_DECIMAL=1
    PRINTF_SUPPORT_DECIMAL_SPECIFIERS=0
    PRINTF_SUPPORT_EXPONENTIAL_SPECIFIERS=0
    SUPPORT_MSVC_STYLE_INTEGER_SPECIFIERS=0
    PRINTF_SUPPORT_WRITEBACK_SPECIFIER=0
    PRINTF_SUPPORT_LONG_LONG=0
    NANOPRINTF_USE_FIELD_WIDTH_FORMAT_SPECIFIERS=0
    NANOPRINTF_USE_PRECISION_FORMAT_SPECIFIERS=1
    NANOPRINTF_USE_FLOAT_FORMAT_SPECIFIERS=1
    NANOPRINTF_USE_LARGE_FORMAT_SPECIFIERS=0
    NANOPRINTF_USE_BINARY_FORMAT_SPECIFIERS=0
    NANOPRINTF_USE_WRITEBACK_FORMAT_SPECIFIERS=0
)
add_subdirectory(Modules/miniPrintf)

# Add smartLED library
add_subdirectory(Modules/smartLED)


create_target(${CMAKE_BUILD_TYPE})
