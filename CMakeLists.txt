cmake_minimum_required(VERSION 3.8)
project(go_under_trolley)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(go_under_trolley src/go_under_trolley.cpp)
ament_target_dependencies(go_under_trolley rclcpp std_msgs geometry_msgs sensor_msgs visualization_msgs pcl_conversions PCL pcl_ros)

install(TARGETS
  go_under_trolley
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
