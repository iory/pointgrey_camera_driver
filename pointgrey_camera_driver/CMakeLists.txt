cmake_minimum_required(VERSION 2.8.3)
project(pointgrey_camera_driver)

find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  diagnostic_updater
  dynamic_reconfigure
  image_exposure_msgs
  image_transport
  nodelet
  roscpp
  sensor_msgs
  wfov_camera_msgs
  cv_bridge)

find_package(OpenCV REQUIRED)
message("OpenCV VERSION: ${OpenCV_VERSION}")
message("OpenCV Components: ${OpenCV_LIB_COMPONENTS}")

catkin_package(CATKIN_DEPENDS
  image_exposure_msgs nodelet roscpp sensor_msgs wfov_camera_msgs cv_bridge
)

include_directories(include /usr/include/spinnaker ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_library(PointGreySyncStereoImagesNodelet src/message_filter_stereo.cpp)
# target_link_libraries(PointGreySyncStereoImagesNodelet ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(PointGreySyncStereoImagesNodelet ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
# target_link_libraries(PointGreySyncStereoImagesNodelet ${catkin_LIBRARIES})

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
