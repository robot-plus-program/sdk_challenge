cmake_minimum_required(VERSION 3.0.2)
project(keti_example)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    keti_msgs
    actionlib
    sensor_msgs
    image_transport
    cv_bridge
)

find_package(
    OpenCV REQUIRED
)

find_package(
    realsense2 REQUIRED
)

catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

set(TARGET_NAME example)

add_executable(${TARGET_NAME} src/example.cpp) 
add_dependencies(${TARGET_NAME} ${${TARGET_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}) 
target_link_libraries(${TARGET_NAME} ${catkin_LIBRARIES})
install(TARGETS ${TARGET_NAME} RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

set(TARGET_SAMPLE pick_and_place)

add_executable(${TARGET_SAMPLE} src/pick_and_place.cpp) 
add_dependencies(${TARGET_SAMPLE} ${${TARGET_SAMPLE}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}) 
target_link_libraries(${TARGET_SAMPLE} ${catkin_LIBRARIES})
install(TARGETS ${TARGET_SAMPLE} RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

set(TARGET_SAMPLE_CAL calibration)

add_executable(${TARGET_SAMPLE_CAL} src/calibration.cpp) 
add_dependencies(${TARGET_SAMPLE_CAL} ${${TARGET_SAMPLE_CAL}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}) 
target_link_libraries(${TARGET_SAMPLE_CAL} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${realsense2_LIBRARY})
install(TARGETS ${TARGET_SAMPLE_CAL} RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(PROGRAMS scripts/example.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(PROGRAMS scripts/pick_and_place.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

