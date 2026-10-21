cmake_minimum_required(VERSION 3.8)
project(skin_feature_generator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(skin_utilities REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(skin_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)

find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(skin_feature_generator_node 
    src/skin_feature.cpp
    src/skin_feature_generator.cpp
    src/skin_feature_generator_node.cpp)

target_include_directories(skin_feature_generator_node PRIVATE
    include
    ${CMAKE_CURRENT_SOURCE_DIR}
)  

ament_target_dependencies(skin_feature_generator_node
  rclcpp
  skin_utilities
  sensor_msgs
  geometry_msgs
  skin_msgs
  tf2_eigen
)

target_link_libraries(skin_feature_generator_node 
  yaml-cpp
  Eigen3::Eigen
)

install(TARGETS
  skin_feature_generator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY 
  configs 
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
