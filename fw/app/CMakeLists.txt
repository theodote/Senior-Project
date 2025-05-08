cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME app3)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# we can fix this

# add_compile_options(-O3)
# target_compile_options(${PROJECT_NAME} PRIVATE -ffast-math)

set(CMSIS_DSP_SOURCES
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS_DSP/Source/BasicMathFunctions/BasicMathFunctions.c
    # ${CMAKE_SOURCE_DIR}/Drivers/CMSIS_DSP/Source/BayesFunctions/BayesFunctions.c
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS_DSP/Source/CommonTables/CommonTables.c
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS_DSP/Source/ComplexMathFunctions/ComplexMathFunctions.c
    # ${CMAKE_SOURCE_DIR}/Drivers/CMSIS_DSP/Source/ControllerFunctions/ControllerFunctions.c
    # ${CMAKE_SOURCE_DIR}/Drivers/CMSIS_DSP/Source/DistanceFunctions/DistanceFunctions.c
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS_DSP/Source/FastMathFunctions/FastMathFunctions.c
    # ${CMAKE_SOURCE_DIR}/Drivers/CMSIS_DSP/Source/FilteringFunctions/FilteringFunctions.c
    # ${CMAKE_SOURCE_DIR}/Drivers/CMSIS_DSP/Source/InterpolationFunctions/InterpolationFunctions.c
    # ${CMAKE_SOURCE_DIR}/Drivers/CMSIS_DSP/Source/MatrixFunctions/MatrixFunctions.c
    # ${CMAKE_SOURCE_DIR}/Drivers/CMSIS_DSP/Source/QuaternionMathFunctions/QuaternionMathFunctions.c
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS_DSP/Source/StatisticsFunctions/StatisticsFunctions.c
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS_DSP/Source/SupportFunctions/SupportFunctions.c
    # ${CMAKE_SOURCE_DIR}/Drivers/CMSIS_DSP/Source/SVMFunctions/SVMFunctions.c
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS_DSP/Source/TransformFunctions/TransformFunctions.c
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    ${CMSIS_DSP_SOURCES}
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ./Drivers/CMSIS_DSP/Include
    ./Drivers/CMSIS_DSP/PrivateInclude
    # ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Include
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
