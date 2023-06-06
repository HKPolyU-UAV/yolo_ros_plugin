cmake_minimum_required(VERSION 3.0.2)
project(yolo_node)


find_package(catkin COMPONENTS
  nodelet
  roscpp
  std_msgs
  roslaunch
  geometry_msgs
  mavros_msgs
  cv_bridge
  image_transport
  sensor_msgs
  message_generation
  genmsg
  visualization_msgs
  pcl_conversions
  pcl_ros 

)

find_package(OpenCV 4.4 REQUIRED)
find_package(Eigen3 REQUIRED)

roslaunch_add_file_check(launch)

add_message_files(
  FILES
)


generate_messages(DEPENDENCIES std_msgs sensor_msgs)

catkin_package(
  #  INCLUDE_DIRS include
  #  LIBRARIES offb
  CATKIN_DEPENDS 
  geometry_msgs 
  mavros_msgs 
  roscpp
  std_msgs
  message_generation
  #  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(mission_control_center
  src/yolo.cpp
  src/RosTopicConfigs.cpp
  )

target_link_libraries(
  mission_control_center
  ${catkin_LIBRARIES}
  ${OpenCV_INCLUDE_DIRS}
)