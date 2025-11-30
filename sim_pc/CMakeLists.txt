cmake_minimum_required(VERSION 3.22)

project(fc_sim_pc C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# --- Executable PC de simulation ---
add_executable(fc_sim_pc
    ./main_pc.c
    ../firmware/flight_control_core/fc.c

)
target_include_directories(fc_sim_pc PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../firmware/flight_control_core
)
