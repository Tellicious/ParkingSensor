# 
# This file provides set of variables for end user
# and also generates one (or more) libraries, that can be added to the project using target_link_libraries(...)
#
# Before this file is included to the root CMakeLists file (using add_subdirectory() function), user can set some variables:
#
# libName_COMPILE_OPTS: If defined, it provide compiler options for generated library.
# libName_COMPILE_DEFS: If defined, it provides "-D" definitions to the library build
#
cmake_minimum_required(VERSION 3.22)

set (libName smartLED)

set(${libName}_src
    ${CMAKE_CURRENT_LIST_DIR}/src/smartLED.c
)

# Setup include directories
set(${libName}_inc
    ${CMAKE_CURRENT_LIST_DIR}/inc
)

# Register library to the system
add_library(${libName} INTERFACE)
target_sources(${libName} INTERFACE ${${libName}_src})
target_include_directories(${libName} INTERFACE ${${libName}_inc})