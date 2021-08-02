cmake_minimum_required(VERSION 2.8.3)
project(flappy_automation_code)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  laser_assembler
  tf2_ros
  pcl_ros
)

catkin_package(
  #INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs laser_assembler tf2_ros pcl_ros
)

include_directories(
  #include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/flappy_automation_code.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

add_executable(pointcloud_processor
  src/pointcloud_processor.cpp
)

target_include_directories(pointcloud_processor PUBLIC
  #include
  ${catkin_INCLUDE_DIRS}
)

target_link_libraries(pointcloud_processor
  ${catkin_LIBRARIES}
)
