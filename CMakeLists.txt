cmake_minimum_required(VERSION 3.5)
project(people_msgs_utils)

add_definitions(-std=c++17)

find_package(catkin REQUIRED
    COMPONENTS
        people_msgs
        geometry_msgs
        tf2
        tf2_ros
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        people_msgs_utils
    CATKIN_DEPENDS
        people_msgs
        geometry_msgs
        tf2
        tf2_ros
)

## Library
add_library(people_msgs_utils
    include/${PROJECT_NAME}/person.h
    src/person.cpp
    include/${PROJECT_NAME}/group.h
    src/group.cpp
    include/${PROJECT_NAME}/utils.h
    src/utils.cpp
)
target_link_libraries(people_msgs_utils
    ${catkin_LIBRARIES}
)

## Install
install(TARGETS people_msgs_utils
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
