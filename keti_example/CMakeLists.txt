cmake_minimum_required(VERSION 3.0.2)
project(keti_example)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    keti_msgs
    actionlib
)

catkin_package(
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

set(TARGET_NAME example)

add_executable(${TARGET_NAME} src/example.cpp) 
add_dependencies(${TARGET_NAME} ${${TARGET_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}) 
target_link_libraries(${TARGET_NAME} ${catkin_LIBRARIES} ${QT_LIBRARIES})
install(TARGETS ${TARGET_NAME} RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


