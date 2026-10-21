cmake_minimum_required(VERSION 3.8)
project(skin_visualizer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(skin_utilities REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(skin_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)

# Include PCL and Eigen headers
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(visualizer_node 
    src/skin_visualizer.cpp
    src/skin_visualizer_node.cpp)

target_include_directories(visualizer_node PRIVATE
    include
    ${CMAKE_CURRENT_SOURCE_DIR}
)   

ament_target_dependencies(visualizer_node
  rclcpp
  skin_utilities
  sensor_msgs
  tf2
  tf2_ros
  tf2_eigen
  tf2_sensor_msgs
  pcl_conversions
  skin_msgs
)

target_link_libraries(visualizer_node 
  ${PCL_LIBRARIES} 
  yaml-cpp
  Eigen3::Eigen
)

install(TARGETS
  visualizer_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY configs
  DESTINATION share/${PROJECT_NAME}
)

ament_package()