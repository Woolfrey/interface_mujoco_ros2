cmake_minimum_required(VERSION 3.5)
project(mujoco_ros2_interface)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# MuJoCo
set(MUJOCO_PATH "/opt/mujoco/mujoco-3.2.0")  # Update this path to your MuJoCo installation
include_directories(${MUJOCO_PATH}/include)
include_directories(${MUJOCO_PATH}/include/mujoco)
link_directories(${MUJOCO_PATH}/lib)

add_executable(mujoco_interface_node src/mujoco_interface_node.cpp)
ament_target_dependencies(mujoco_interface_node rclcpp sensor_msgs)

target_link_libraries(mujoco_interface_node mujoco)

install(TARGETS
  mujoco_interface_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()

