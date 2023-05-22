cmake_minimum_required(VERSION 3.21)

project(EXAMPLE_CONNECT_SAFE)

message("-- [INFO] Building Example")

add_executable(example_connect_safe
    connect_safe.cpp
    ../common/utils.cpp
)

add_subdirectory(/home/matter/workspace/SimpleBLE/simpleble ${CMAKE_BINARY_DIR}/simpleble)
target_link_libraries(example_connect_safe simpleble::simpleble)
