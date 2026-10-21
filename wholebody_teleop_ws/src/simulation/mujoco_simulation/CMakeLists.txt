cmake_minimum_required(VERSION 3.16)
project(mujoco_simulation LANGUAGES C CXX)

# --- Path setup
set(ENGINEAI_ROBOTICS_THIRD_PARTY /opt/engineai_robotics_third_party)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/simulate/cmake)
list(APPEND CMAKE_PREFIX_PATH ${ENGINEAI_ROBOTICS_THIRD_PARTY})

set(CMAKE_BUILD_TYPE Release CACHE STRING "Set build type as Release." FORCE)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interface_protocol REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(skin_msgs REQUIRED)
find_package(mujoco_skin)
find_package(mujoco_ray_caster)

# Third-party
find_package(GTest REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(mujoco REQUIRED)
find_package(glfw3 REQUIRED)

# --- Library Setup for mujoco simulate target
set(MUJOCO_LIB "${ENGINEAI_ROBOTICS_THIRD_PARTY}/lib/libmujoco.so")
set(SIMULATE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/simulate)
add_subdirectory(${SIMULATE_DIR})

# --- Set path to mujoco lib dir
link_directories(${ENGINEAI_ROBOTICS_THIRD_PARTY}/lib)

# --- Executable
add_executable(mujoco_simulator
  src/main.cpp
  src/sensors.cpp
  src/config_loader.cpp
  src/robot_interface.cpp
  src/ros_interface.cpp
  src/sim_manager.cpp
  src/ros_glfw_adapter.cpp
)

target_include_directories(mujoco_simulator PRIVATE
  include
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${SIMULATE_DIR}
  ${ENGINEAI_ROBOTICS_THIRD_PARTY}/include
)

target_link_libraries(mujoco_simulator
  ${MUJOCO_LIB}
  libsimulate
  yaml-cpp::yaml-cpp
  Eigen3::Eigen
  glfw
  pthread
  ${CMAKE_DL_LIBS}
)

ament_target_dependencies(mujoco_simulator
  rclcpp
  interface_protocol
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  skin_msgs
  mujoco_skin
)

# --- Installation

install(TARGETS
  mujoco_simulator
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()