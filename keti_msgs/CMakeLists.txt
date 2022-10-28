cmake_minimum_required(VERSION 3.0.2)
project(keti_msgs)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

add_message_files(
  FILES
  RobotState.msg
  VisionState.msg
  GripperState.msg
)

add_service_files(
  FILES
  VisionComm.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs message_runtime
)

include_directories(
)