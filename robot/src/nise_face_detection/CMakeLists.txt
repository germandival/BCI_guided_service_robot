cmake_minimum_required(VERSION 2.8.3)
project(nise_face_detection)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  rospy
  message_generation
  sensor_msgs
  cv_bridge
  image_transport
)

add_service_files(
    DIRECTORY srv
    FILES
    RoiFace.srv
    NaoFace.srv
)

generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS message_runtime)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS ./script/face_recognition.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
add_executable(nise_face_detection src/main.cpp)
target_link_libraries(nise_face_detection ${catkin_LIBRARIES})

find_package(OpenCV REQUIRED)
target_link_libraries(nise_face_detection ${OpenCV_LIBS})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /usr/local/lib/cmake)
find_package(aruco REQUIRED)
target_link_libraries(nise_face_detection  ${aruco_LIBS})
