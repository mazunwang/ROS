cmake_minimum_required(VERSION 3.8.0)
project(nav)

set(CMAKE_CXX_FLAGS "-std=c++11 -g -w -s")
set(CMAKE_BUILD_TYPE Release)
set(THIRD_PARTY ./third_party)
## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    geometry_msgs
    nav_msgs
    sensor_msgs
    std_msgs
    tf
    message_generation
    actionlib
    dynamic_reconfigure
)

add_action_files(
    DIRECTORY
        action
    FILES
        Navigation.action
)

generate_messages(
    DEPENDENCIES
        actionlib_msgs
        geometry_msgs
)

# generate_dynamic_reconfigure_options(cfg/navigation.cfg)

catkin_package(
    INCLUDE_DIRS
    include
    LIBRARIES
    CATKIN_DEPENDS
    dynamic_reconfigure
)



include_directories(
    ${catkin_INCLUDE_DIRS}
    include
    ${THIRD_PARTY}/eigen3
    ${THIRD_PARTY}/toml
)

add_library(nav STATIC  src/basic_function.cpp src/nav_sl.cpp src/nav_mpc.cpp)

add_executable(nav_test src/test.cpp )
target_link_libraries(nav_test nav ${catkin_LIBRARIES})

add_executable(navigation src/nav_node.cpp src/nav_interface.cpp)
target_link_libraries(navigation nav ${catkin_LIBRARIES})
# add_dependencies(navigation &{PROJECT_NAME}_gencfg)