cmake_minimum_required(VERSION 2.8.3)
project(nise_pose)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  rospy
  message_generation
  sensor_msgs
  cv_bridge
  image_transport
)
find_package(OpenCV REQUIRED)

add_service_files(
    DIRECTORY srv
    FILES
    ChangePosture.srv
)

generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS ./script/change_posture.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
