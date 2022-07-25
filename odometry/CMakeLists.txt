cmake_minimum_required(VERSION 3.0.2)
project(odometry)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
)

 generate_messages(
   DEPENDENCIES
   geometry_msgs nav_msgs sensor_msgs std_msgs
)

catkin_package(
   LIBRARIES odometry
   CATKIN_DEPENDS geometry_msgs nav_msgs roscpp sensor_msgs std_msgs tf
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
 
add_executable(odometry_publisher src/odom_publisher.cpp)

add_dependencies(odometry_publisher ${odometry_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(odometry_publisher
   ${catkin_LIBRARIES}
 )


