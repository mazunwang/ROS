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
    #deep_msgs
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

catkin_package(
    INCLUDE_DIRS
    include
    # ${THIRD_PARTY}/eigen3
    # ${EIGEN3_INCLUDE_DIRS}
    # ${PCL_INCLUDE_DIRS}
    # ${OpenVDB_INCLUDE_DIR}
    # ${TBB_INCLUDE_DIRS}
    # ${EXTERNAL_INCLUDE_DIRS}
    LIBRARIES
    # local_planner
    # ${EXTERNAL_LIBS}
    CATKIN_DEPENDS
        # move_base_msgs
    # dynamic_reconfigure
    # geometry_msgs
    # interactive_markers
    # message_runtime
    # nav_msgs
    # pluginlib
    # roscpp
    # std_msgs
    # tf_conversions
    # tf2_eigen
    # tf2_geometry_msgs
    # tf2_sensor_msgs
    # tf2
    # tf2_ros
    # visualization_msgs
    # DEPENDS
    # SUITESPARSE
    # G2O
    # TBB
    # OpenVDB
    # PCL
    # EIGEN3
    # Boost
)



include_directories(
    ${catkin_INCLUDE_DIRS}
    include
    ${THIRD_PARTY}/eigen3
)



add_library(nav_interface STATIC src/nav_interface.cpp src/nav_sl.cpp)

add_executable(nav_test src/test.cpp)
target_link_libraries(nav_test nav_interface)