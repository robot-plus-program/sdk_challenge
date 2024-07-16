cmake_minimum_required(VERSION 3.0.2)
project(keti_sdk)

add_compile_options(-std=c++11)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/gripper
    ${CMAKE_CURRENT_SOURCE_DIR}/ketirobotsdk
)

link_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/ketirobotsdk
)

set(TARGET_EXAMPLE example)
add_executable(${TARGET_EXAMPLE} src/example.cpp gripper/zimmergripper.cpp)
target_link_libraries(${TARGET_EXAMPLE} robotsdkv4 pthread modbus)

set(TARGET_TEST test)
add_executable(${TARGET_TEST} src/test.cpp)
target_link_libraries(${TARGET_TEST} robotsdkv4)
