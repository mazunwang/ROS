cmake_minimum_required(VERSION 3.0.3)
project(quadog_control_cpp)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  geometry_msgs
)

#find_library()

add_message_files(FILES my_msg.msg)

generate_messages(DEPENDENCIES std_msgs)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES quadog_control_cpp
#  CATKIN_DEPENDS roscpp std_msg
#  DEPENDS system_lib
  CATKIN_DEPENDS
  message_runtime
)
include_directories("./third-party/qpOASES/include")
include_directories("./third-party/qpOASES/include/qpOASES")
include_directories("./third-party/qpOASES/include/qpOASES/extra")
#include_directories("./third-party/")
#include_directories("./third-party/qpOASES/include/qpOASES")
#include_directories("./third-party/qpOASES/src/")
#include_directories("./third-party/qpOASES/")
#include_directories("./")
#include_directories("./include")
#include_directories("./include/")
#include_
include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

aux_source_directory(./src DIR_LIB_SRCS)
aux_source_directory(./third-party/qpOASES/src QPOASES_SRCS)

#aux_source_directory(src "./src")
#aux_source_directory(src "./")

#set(src_files src/LegController.cpp)


add_library(CONTROLLER STATIC
#${src_files}
${DIR_LIB_SRCS}
${QPOASES_SRCS}
)

add_executable(main main.cpp)


add_dependencies(main quadog_control_cpp_generate_messages_cpp)
target_link_libraries(main 
${catkin_LIBRARIES}
CONTROLLER
)


add_executable(pub_test pub_test.cpp)
add_dependencies(pub_test quadog_control_cpp_generate_messages_cpp)
target_link_libraries(pub_test ${catkin_LIBRARIES})
target_link_libraries(pub_test 
${catkin_LIBRARIES}
CONTROLLER
)



