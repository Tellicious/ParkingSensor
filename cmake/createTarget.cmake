
function(create_target CONFIG_TYPE)

# Set name of target
set(TARGET_NAME ${PROJECT_NAME})
string(FIND ${CONFIG_TYPE} Debug pos)
if(NOT ${pos} EQUAL -1)
    string(REPLACE Debug "" C_NAME ${CONFIG_TYPE})
    string(APPEND TARGET_NAME _D_${C_NAME})
else()
    string(REPLACE Release "" C_NAME ${CONFIG_TYPE})
    string(APPEND TARGET_NAME _${C_NAME})
endif()

# Add executable
add_executable(${TARGET_NAME})

# Sources
set(sources_SRCS
    Drivers/BSP/SX1278/SX1278.c
)

# Include directories for all compilers
set(include_DIRS
    ${PROJECT_SOURCE_DIR}/Drivers/BSP/SX1278
)

# Linker script
set(linker_script_SRC ${PROJECT_SOURCE_DIR}/STM32F103C8TX_FLASH.ld)

target_compile_definitions(
    ${TARGET_NAME} PRIVATE
    ${user_DEFS}
    $<$<OR:$<STREQUAL:${CONFIG_TYPE},DebugMain>,$<STREQUAL:${CONFIG_TYPE},ReleaseMain>>:modeMAIN>
    $<$<OR:$<STREQUAL:${CONFIG_TYPE},DebugRemote>,$<STREQUAL:${CONFIG_TYPE},ReleaseRemote>>:modeREMOTE>
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
target_link_libraries(${TARGET_NAME} stm32cubemx ADVUtils debugPrintf)

# Execute post-build to print size, generate hex and bin
add_custom_command(
    TARGET ${TARGET_NAME} POST_BUILD
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${TARGET_NAME}>
    #COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${TARGET_NAME}> ${TARGET_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${TARGET_NAME}> ${TARGET_NAME}.bin
)

endfunction()