cmake_minimum_required(VERSION 3.8)
project(mujoco_skin LANGUAGES C CXX)

# special installation
set(ENGINEAI_ROBOTICS_THIRD_PARTY /opt/engineai_robotics_third_party)
list(APPEND CMAKE_PREFIX_PATH ${ENGINEAI_ROBOTICS_THIRD_PARTY})

# Set ROS2 build type
set(CMAKE_BUILD_TYPE Release CACHE STRING "Set build type as Release." FORCE)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mujoco REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# PCL and pcl dependencies
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

set(MUJOCO_LIB "${ENGINEAI_ROBOTICS_THIRD_PARTY}/lib/libmujoco.so")

# --- Library

add_library(mujoco_skin 
  src/skin_sensor.cpp
  src/skin_sensor_plugin.cpp

  src/ray_sensor.cpp
  src/ray_sensor_plugin.cpp
  
  src/ray_camera_sensor.cpp
  src/ray_camera_sensor_plugin.cpp
  
  src/plugin_utilities.cpp
)

target_include_directories(mujoco_skin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${ENGINEAI_ROBOTICS_THIRD_PARTY}/include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(mujoco_skin
  ${MUJOCO_LIB}
  yaml-cpp::yaml-cpp
  ${EIGEN3_LIBRARIES}
  ${PCL_LIBRARIES}
  ${CMAKE_DL_LIBS}
)

## --- ROS2 dependency

ament_target_dependencies(mujoco_skin
  rclcpp
  ament_index_cpp
  pcl_conversions
  pcl_ros
)

## --- EXPORTS

ament_export_include_directories(include)
ament_export_dependencies(rclcpp ament_index_cpp pcl_conversions pcl_ros)
ament_export_targets(export_${PROJECT_NAME})

install(TARGETS mujoco_skin
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)

ament_package()