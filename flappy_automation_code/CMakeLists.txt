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
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs laser_assembler tf2_ros pcl_ros
)

add_executable(${PROJECT_NAME}_node
  src/flappy_automation_code.cpp
  src/flappy_controller.cpp
)

target_include_directories(${PROJECT_NAME}_node PUBLIC
  include
  ${catkin_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

add_executable(pointcloud_processor_node
  src/pointcloud_processor_node.cpp
  src/pointcloud_processor.cpp
)

target_include_directories(pointcloud_processor_node PUBLIC
  include
  ${catkin_INCLUDE_DIRS}
)

target_link_libraries(pointcloud_processor_node
  ${catkin_LIBRARIES}
)
