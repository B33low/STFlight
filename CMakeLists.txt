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
set(CMAKE_PROJECT_NAME STFlight)


# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE

    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE

    # Add user sources here
    # firmware/main_flight.c
    HXbots/app/app.c
    HXbots/app/bus_registery_app.c

    HXbots/core/bus/bus_registery_core.c
    HXbots/core/bus/bus_rx.c
    HXbots/core/bus/bus_serialize.c

    HXbots/core/param/param_base.c
    HXbots/core/state/state_base.c
    HXbots/core/stream/stream_base.c

    HXbots/drivers/sensors/lis2mdl.c
    HXbots/drivers/sensors/lps22hb.c
    HXbots/drivers/sensors/lsm6dso32.c

    HXbots/modules/altitude/altitude_estimator.c
    HXbots/modules/altitude/altitude_state.c

    HXbots/modules/attitude/attitude_state.c
    HXbots/modules/attitude/attitude_estimator.c
    HXbots/modules/attitude/att_filter_params.c

    HXbots/modules/fc/fc.c

    HXbots/modules/imu/imu_params.c
    HXbots/modules/imu/imu_stream.c

    HXbots/modules/imu_lsm6dso32/imu_lsm6dso32.c


    HXbots/platform/stm32/comm/uart_port.c

)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE

    # Add user defined include paths
    ${CMAKE_CURRENT_SOURCE_DIR}/HXbots
    ${CMAKE_CURRENT_SOURCE_DIR}/HXbots/app
    ${CMAKE_CURRENT_SOURCE_DIR}/HXbots/core/bus
    ${CMAKE_CURRENT_SOURCE_DIR}/HXbots/core/param
    ${CMAKE_CURRENT_SOURCE_DIR}/HXbots/core/state
    ${CMAKE_CURRENT_SOURCE_DIR}/HXbots/core/stream
    ${CMAKE_CURRENT_SOURCE_DIR}/HXbots/drivers/sensors
    ${CMAKE_CURRENT_SOURCE_DIR}/HXbots/modules/altitude
    ${CMAKE_CURRENT_SOURCE_DIR}/HXbots/modules/attitude
    ${CMAKE_CURRENT_SOURCE_DIR}/HXbots/modules/fc
    ${CMAKE_CURRENT_SOURCE_DIR}/HXbots/modules/imu
    ${CMAKE_CURRENT_SOURCE_DIR}/HXbots/modules/imu_lsm6dso32
    ${CMAKE_CURRENT_SOURCE_DIR}/HXbots/platform/stm32/comm/

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

target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -u _printf_float
)
