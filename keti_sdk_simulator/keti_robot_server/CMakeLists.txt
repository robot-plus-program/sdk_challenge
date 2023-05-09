cmake_minimum_required(VERSION 3.0.2)
project(keti_robot_server)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    actionlib
    keti_robot_control
)
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
    LIBRARIES
    INCLUDE_DIRS
)

include_directories(
    SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR} 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

set(SERVER_TARGET robot_server)

add_executable(${SERVER_TARGET} src/main.cpp src/robot_server.cpp)
add_dependencies(${SERVER_TARGET} ${${SERVER_TARGET}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}) 
target_link_libraries(${SERVER_TARGET} ${catkin_LIBRARIES} ${Boost_LIBRARIES} modbus)
install(TARGETS ${SERVER_TARGET} RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

