cmake_minimum_required(VERSION 3.21)

project(EXAMPLE_BP)

message("-- [INFO] Building Example")

add_executable(example_bp
    BP.cpp
    ../common/utils.cpp
)

set_target_properties(example_bp PROPERTIES
    CXX_STANDARD 17
)


add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../../../../simpleble ${CMAKE_BINARY_DIR}/simpleble)
target_link_libraries(example_bp simpleble::simpleble)
