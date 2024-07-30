cmake_minimum_required(VERSION 3.5)
project(mujoco_ros2_interface)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Specify the path to MuJoCo
set(MUJOCO_PATH /opt/mujoco/mujoco-3.2.0)

# Include the MuJoCo headers
include_directories(${MUJOCO_PATH}/include)
include_directories(${MUJOCO_PATH}/include/mujoco)
# Link the MuJoCo libraries
link_directories(${MUJOCO_PATH}/lib)

# Find GLFW
find_package(PkgConfig REQUIRED)
pkg_search_module(GLFW REQUIRED glfw3)

# Include GLFW headers
include_directories(${GLFW_INCLUDE_DIRS})

add_executable(mujoco_interface_node src/mujoco_interface_node.cpp)

ament_target_dependencies(mujoco_interface_node rclcpp std_msgs)

# Link against MuJoCo and GLFW libraries
target_link_libraries(mujoco_interface_node mujoco ${GLFW_LIBRARIES})

install(TARGETS
  mujoco_interface_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()

