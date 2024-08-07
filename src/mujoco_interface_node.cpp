/**
 * @file   mujoco_interface_node.cpp
 * @author Jon Woolfrey
 * @data   August 2024
 * @brief  Creates a ROS2 node that interfaces with a MuJoCo simulation.
 */

#include <MuJoCoInterface.h>
#include <iostream>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MuJoCoInterface>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

