cmake_minimum_required(VERSION 3.8)
project(skin_utilities)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(skin_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)

install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)

ament_export_dependencies(
  rclcpp
  sensor_msgs
  geometry_msgs
  skin_msgs
  tf2
  tf2_eigen
)

ament_package()
