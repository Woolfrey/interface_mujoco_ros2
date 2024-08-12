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

    // Create a node to access parameters
    auto node = std::make_shared<rclcpp::Node>("mujoco_interface_parameters");

    // Load parameters from the parameter server
    std::string xml_location = node->declare_parameter<std::string>("xml_location", "");
    std::string control_mode_str = node->declare_parameter<std::string>("control_mode", "TORQUE");
    std::string publisher_name = node->declare_parameter<std::string>("publisher_name", "joint_states");
    std::string subscriber_name = node->declare_parameter<std::string>("subscriber_name", "joint_controls");

    // Set the control mode
    ControlMode control_mode;
         if (control_mode_str == "POSITION") control_mode = POSITION;
    else if (control_mode_str == "VELOCITY") control_mode = VELOCITY;
    else if (control_mode_str == "TORQUE")   control_mode = TORQUE;
    else                                     control_mode = UNKNOWN;

    // Create MuJoCoInterface object
    auto mujoco_interface = std::make_shared<MuJoCoInterface>(xml_location, publisher_name, subscriber_name, control_mode);

    // Load and set feedback gains
    double proportional_gain = node->declare_parameter<double>("proportional_gain", 0.0);
    double derivative_gain = node->declare_parameter<double>("derivative_gain", 0.0);
    double integral_gain = node->declare_parameter<double>("integral_gain", 0.0);

    mujoco_interface->set_feedback_gains(proportional_gain, integral_gain, derivative_gain);

    // Load and set camera properties
    std::vector<double> camera_focal_point = node->declare_parameter<std::vector<double>>("camera_focal_point", {0.0, 0.0, 0.0});
    double camera_distance = node->declare_parameter<double>("camera_distance", 1.0);
    double camera_azimuth = node->declare_parameter<double>("camera_azimuth", 0.0);
    double camera_elevation = node->declare_parameter<double>("camera_elevation", 0.0);
    bool camera_orthographic = node->declare_parameter<bool>("camera_orthographic", false);

    mujoco_interface->set_camera_properties({camera_focal_point[0], camera_focal_point[1], camera_focal_point[2]}, camera_distance, camera_azimuth, camera_elevation, camera_orthographic);

    // Spin the node
    rclcpp::spin(mujoco_interface);

    rclcpp::shutdown();
    return 0;
}

