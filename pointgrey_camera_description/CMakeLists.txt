cmake_minimum_required(VERSION 2.8.3)
project(pointgrey_camera_description)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch meshes rviz urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

message("desc ${OpenCV_INCLUDE_DIRS}")
message("desc ${OpenCV_LIBRARIES}")
