cmake_minimum_required(VERSION 2.8.3)
project(nise_interface)

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_MODULE_PATH /usr/local/lib/cmake)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  std_msgs
  rospy
  message_generation
  sensor_msgs
)
find_package(aruco REQUIRED)
add_service_files(
    DIRECTORY srv
    FILES
    Interface.srv
)

generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS message_runtime)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS ./script/udp_interface.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
