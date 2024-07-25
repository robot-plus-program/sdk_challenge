cmake_minimum_required(VERSION 3.13)
project(keti_robot_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    message_generation
    moveit_ros_planning_interface
    moveit_visual_tools
    actionlib
    actionlib_msgs
)
find_package(Boost REQUIRED COMPONENTS system)

add_message_files(
    FILES
    RobotState.msg
    GripperState.msg
)

add_action_files(
    DIRECTORY action
    FILES RobotMove.action GripperMove.action
)

generate_messages(
    DEPENDENCIES std_msgs actionlib_msgs
)

catkin_package(
    LIBRARIES
    INCLUDE_DIRS
    CATKIN_DEPENDS
        moveit_ros_planning_interface
        moveit_visual_tools
        std_msgs
        message_runtime
        actionlib_msgs
)

include_directories(
    SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR} 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

set(ROBOT_TARGET robot_control)

add_executable(${ROBOT_TARGET} src/main.cpp src/robot_control.cpp)
add_dependencies(${ROBOT_TARGET} ${${ROBOT_TARGET}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}) 
target_link_libraries(${ROBOT_TARGET} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS ${ROBOT_TARGET} RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


