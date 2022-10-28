cmake_minimum_required(VERSION 3.0.2)
project(keti_tcp)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    keti_msgs
)

generate_messages(
  DEPENDENCIES std_msgs
)

catkin_package(
)

include_directories(
  SYSTEM 
  ${catkin_INCLUDE_DIRS} 
  ${Boost_INCLUDE_DIR} 
  ${EIGEN3_INCLUDE_DIRS}
)

set(TARGET_NAME tcp_vision)

add_executable(${TARGET_NAME}  src/tcp_vision.cpp) 
add_dependencies(${TARGET_NAME} ${${TARGET_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}) 
target_link_libraries(${TARGET_NAME} ${catkin_LIBRARIES})
install(TARGETS ${TARGET_NAME} RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})



