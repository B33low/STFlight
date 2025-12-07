cmake_minimum_required(VERSION 3.22)

project(fc_sim_pc C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)


add_executable(fc_sim_pc
    ./main_pc.c
    ./simulator.c

    # Flight control core
    ../firmware/flight_control_core/fc.c

    # Data store core
    ../firmware/data_store/stream_base.c
    ../firmware/data_store/state_base.c
    ../firmware/data_store/param_base.c

    # Serializer registry (if your main uses it)
    ../firmware/data_store/serializer/bus_registery.c
    ../firmware/data_store/serializer/bus_serialize.c
    ../firmware/data_store/serializer/bus_rx.c
)

target_include_directories(fc_sim_pc PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../firmware/flight_control_core
    ${CMAKE_CURRENT_SOURCE_DIR}/../firmware/data_store
    ${CMAKE_CURRENT_SOURCE_DIR}/../firmware/data_store/serializer

    ${CMAKE_CURRENT_SOURCE_DIR}/../firmware/streams
    ${CMAKE_CURRENT_SOURCE_DIR}/../firmware/states
    ${CMAKE_CURRENT_SOURCE_DIR}/../firmware/params
)
target_compile_definitions(fc_sim_pc PRIVATE _CRT_SECURE_NO_WARNINGS)