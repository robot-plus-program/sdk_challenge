cmake_minimum_required(VERSION 3.0.2)
project(keti_robot)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    keti_msgs
    actionlib 
    actionlib_msgs
    sensor_msgs
    moveit_ros_planning_interface
    moveit_visual_tools
)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)

add_action_files(
  DIRECTORY action
  FILES RobotMove.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs sensor_msgs
)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS 
    actionlib_msgs
    moveit_ros_planning_interface
    moveit_visual_tools
  DEPENDS
    EIGEN3
)

include_directories(
  SYSTEM 
  ${catkin_INCLUDE_DIRS} 
  ${Boost_INCLUDE_DIR} 
  ${EIGEN3_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include/robot
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# set(TARGET_NAME robot_control)

# add_executable(${TARGET_NAME}  src/robot_control.cpp src/robot_move_action.cpp) 
# add_dependencies(${TARGET_NAME} ${${TARGET_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}) 
# target_link_libraries(${TARGET_NAME} ${catkin_LIBRARIES} robotsdk)
# install(TARGETS ${TARGET_NAME} RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

set(TARGET_SIM_NAME robot_control_sim)

add_executable(${TARGET_SIM_NAME}  src/robot_control_sim.cpp src/robot_move_sim_action.cpp) 
add_dependencies(${TARGET_SIM_NAME} ${${TARGET_SIM_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}) 
target_link_libraries(${TARGET_SIM_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS ${TARGET_SIM_NAME} RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})



