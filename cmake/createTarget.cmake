
function(create_target CONFIG_TYPE)

# Set name of target
set(TARGET_NAME ${PROJECT_NAME})

# Add executable
add_executable(${TARGET_NAME})

# Sources
set(sources_SRCS
    Drivers/BSP/src/eeprom.c
    Drivers/BSP/VL53L1X/src/vl53l1_platform.c
    Drivers/BSP/VL53L1X/src/VL53L1X_api.c
    Drivers/BSP/VL53L1X/src/VL53L1X_calibration.c
)

# Include directories for all compilers
set(include_DIRS
    Drivers/BSP/inc
    Drivers/BSP/VL53L1X/inc
)

# Linker script
set(linker_script_SRC ${PROJECT_SOURCE_DIR}/STM32F103C8Tx_FLASH.ld)

target_compile_definitions(
    ${TARGET_NAME} PRIVATE
    ${user_DEFS}
)

target_include_directories(
    ${TARGET_NAME} PRIVATE
    ${include_DIRS}
)

# Setup target
target_compile_options(
    ${TARGET_NAME} PRIVATE
    ${cpu_PARAMS}
    ${compiler_OPTS}
)

target_link_options(
    ${TARGET_NAME} PRIVATE
    -T${linker_script_SRC}
    -Wl,-Map=${TARGET_NAME}.map
    ${cpu_PARAMS}
    ${linker_OPTS}
)

target_sources(
    ${TARGET_NAME} PRIVATE
    ${sources_SRCS}
)

# Link libraries
target_link_libraries(${TARGET_NAME} stm32cubemx ADVUtils miniPrintf smartLED)

# Execute post-build to print size, generate hex and bin
add_custom_command(
    TARGET ${TARGET_NAME} POST_BUILD
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${TARGET_NAME}>
    #COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${TARGET_NAME}> ${TARGET_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${TARGET_NAME}> ${TARGET_NAME}.bin
)

endfunction()