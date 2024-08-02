cmake_minimum_required(VERSION 3.5)
project(mujoco_interface)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(glfw3 REQUIRED)

# MuJoCo
set(MUJOCO_PATH "/opt/mujoco/mujoco-3.2.0")                                                         # Update this path to your MuJoCo installation
include_directories(include)                                                                        # Local header files
include_directories(${MUJOCO_PATH}/include/mujoco)                                                  # MuJoCo header files
include_directories(${MUJOCO_PATH}/include)                                                         # Additional MuJoCo header files
link_directories(${MUJOCO_PATH}/lib)                                                                # Location of MuJoCo libraries

add_executable(mujoco_interface_node src/mujoco_interface_node.cpp
                                     src/MuJoCoInterface.cpp)
    
target_link_libraries(mujoco_interface_node mujoco glfw)
                                     
ament_target_dependencies(mujoco_interface_node rclcpp sensor_msgs)

install(TARGETS mujoco_interface_node
        DESTINATION lib/${PROJECT_NAME})

ament_package()
