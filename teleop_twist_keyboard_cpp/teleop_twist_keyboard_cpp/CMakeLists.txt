cmake_minimum_required(VERSION 2.8.3)
project(teleop_twist_keyboard_cpp)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS src
  CATKIN_DEPENDS roscpp geometry_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(teleop_twist_keyboard src/teleop_twist_keyboard.cpp)
add_executable(key_sub src/key_sub.cpp)

add_dependencies(teleop_twist_keyboard ${teleop_twist_keyboard_cpp_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(key_sub ${teleop_twist_keyboard_cpp_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(teleop_twist_keyboard ${catkin_LIBRARIES})
target_link_libraries(key_sub ${catkin_LIBRARIES})

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
