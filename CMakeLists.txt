cmake_minimum_required(VERSION 3.5)
project(people_msgs_utils)

add_definitions(-std=c++17)

find_package(catkin REQUIRED
    COMPONENTS
        people_msgs
        geometry_msgs
        tf2
        tf2_ros
        social_nav_utils
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${social_nav_utils_INCLUDE_DIRS}
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
        social_nav_utils
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
    ${social_nav_utils_LIBRARIES}
)

## Install
install(TARGETS people_msgs_utils
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Testing
# catkin run_tests --no-deps people_msgs_utils --verbose
if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_creation test/test_creation.cpp)
  if(TARGET test_creation)
    target_link_libraries(test_creation people_msgs_utils)
  endif()
  catkin_add_gtest(test_group test/test_group.cpp)
  if(TARGET test_group)
    target_link_libraries(test_group people_msgs_utils)
  endif()
endif()
