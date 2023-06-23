cmake_minimum_required(VERSION 3.0.2)
project(keti_sdk)

add_compile_options(-std=c++11)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/robot
    ${CMAKE_CURRENT_SOURCE_DIR}/include/gripper
    # ${CMAKE_CURRENT_SOURCE_DIR}/ketirobotsdk
)

link_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include/robot
    # ${CMAKE_CURRENT_SOURCE_DIR}/ketirobotsdk
)

set(TARGET_EXAMPLE example)
add_executable(${TARGET_EXAMPLE} src/example.cpp include/gripper/zimmergripper.cpp)
target_link_libraries(${TARGET_EXAMPLE} robotsdkv2 pthread modbus)
