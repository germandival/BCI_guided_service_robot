cmake_minimum_required(VERSION 2.8.3)
project(nise_walk)


find_package(catkin REQUIRED cmake_modules COMPONENTS
  roscpp
  std_msgs
  naoqi_bridge_msgs
  std_srvs
  tf
  actionlib_msgs
  geometry_msgs
  sensor_msgs
  message_generation
)

add_service_files(
    DIRECTORY srv
    FILES
    Walk.srv
)

generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS message_runtime)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(nise_walk src/main.cpp)

target_link_libraries(nise_walk ${catkin_LIBRARIES})

find_package(Boost REQUIRED COMPONENTS system thread)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(nise_walk ${Boost_LIBRARIES})
