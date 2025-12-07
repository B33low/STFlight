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
    firmware/flight_control.c
    firmware/sensor_drivers/sensor_imu.c
    firmware/sensor_drivers/lsm6dso32.c
    firmware/sensor_drivers/lis2mdl.c
    firmware/sensor_drivers/lps22hb.c
    firmware/flight_control_core/fc.c
    firmware/flight_control_core/altitude_estimator.c
    firmware/flight_control_core/attitude_estimator.c
    firmware/hw_app/app.c
    # Data store core
    firmware/data_store/stream_base.c
    firmware/data_store/state_base.c
    firmware/data_store/param_base.c

    firmware/params/imu_params.c
    firmware/params/att_filter_params.c
    firmware/streams/imu_stream.c
    firmware/states/altitude_state.c
    firmware/states/attitude_state.c


    # Serializer registry (if your main uses it)
    firmware/data_store/serializer/bus_registery.c
    firmware/data_store/serializer/bus_serialize.c
    firmware/data_store/serializer/bus_rx.c

)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE

    # Add user defined include paths
    ${CMAKE_CURRENT_SOURCE_DIR}/firmware
    ${CMAKE_CURRENT_SOURCE_DIR}/firmware/sensor_drivers
    ${CMAKE_CURRENT_SOURCE_DIR}/firmware/flight_control_core
    ${CMAKE_CURRENT_SOURCE_DIR}/firmware/hw_app
    ${CMAKE_CURRENT_SOURCE_DIR}/firmware/data_store
    ${CMAKE_CURRENT_SOURCE_DIR}/firmware/streams
    ${CMAKE_CURRENT_SOURCE_DIR}/firmware/states
    ${CMAKE_CURRENT_SOURCE_DIR}/firmware/params
    ${CMAKE_CURRENT_SOURCE_DIR}/firmware/data_store/serializer

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
