cmake_minimum_required(VERSION 3.5)
project(car_gazebo)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(FastRTPS_INCLUDE_DIR /opt/ros/foxy/include)
set(FastRTPS_LIBRARY_RELEASE /opt/ros/foxy/lib/libfastrtps.so)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(car_msgs REQUIRED)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(${PROJECT_NAME} SHARED src/car_plugin.cpp)
target_link_libraries(${PROJECT_NAME} ${GAZEBO_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME} rclcpp car_msgs)
