cmake_minimum_required(VERSION 3.0.2)
project(keti_gripper)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    actionlib 
    actionlib_msgs
)
find_package(Boost REQUIRED COMPONENTS system)

add_action_files(
  DIRECTORY action
  FILES GripperMove.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# set(TARGET_NAME onrobot_2fg7)

# add_executable(${TARGET_NAME}  src/keti_gripper.cpp src/gripper_move_action.cpp) 
# add_dependencies(${TARGET_NAME} ${${TARGET_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}) 
# target_link_libraries(${TARGET_NAME} ${catkin_LIBRARIES})
# install(TARGETS ${TARGET_NAME} RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(PROGRAMS scripts/keti_gripper_server.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
