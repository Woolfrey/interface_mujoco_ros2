/**
 * @file   mujocoSim_node.cpp
 * @author Jon Woolfrey
 * @data   August 2024
 * @brief  Creates a ROS2 node that interfaces with a MuJoCo simulation.
 */

#include <MuJoCoInterface.h>
#include <iostream>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);                                                                       // Starts up ROS2
    
    ////////////////////////////////////////////////////////////////////////////////////////////////
    
    auto node = std::make_shared<rclcpp::Node>("mujoco_sim_parameters");                            // Create a node to access parameters

    int simulationFrequency    = node->declare_parameter<int>("simulation_frequency", 1);
    int visualizationFrequency = node->declare_parameter<int>("visualization_frequency", 20);
    std::string xmlLocation    = node->declare_parameter<std::string>("xml_path", "");
    std::string controlMode    = node->declare_parameter<std::string>("control_mode", "TORQUE");
    std::string publisherName  = node->declare_parameter<std::string>("publisher_name", "joint_states");
    std::string subscriberName = node->declare_parameter<std::string>("subscriber_name", "joint_commands");

    double proportionalGain = node->declare_parameter<double>("proportional_gain", 1.0);
    double derivativeGain   = node->declare_parameter<double>("derivative_gain", 0.01);
    double integralGain     = node->declare_parameter<double>("integral_gain", 0.0);

    // Load camera parameters
    std::vector<double> camera_focal_point = node->declare_parameter<std::vector<double>>("camera_focal_point", {0.0, 0.0, 0.0});
    double camera_distance = node->declare_parameter<double>("camera_distance", 1.0);
    double camera_azimuth = node->declare_parameter<double>("camera_azimuth", 0.0);
    double camera_elevation = node->declare_parameter<double>("camera_elevation", 0.0);
    bool camera_orthographic = node->declare_parameter<bool>("camera_orthographic", false);
        
    // Set the control mode
    ControlMode control_mode;
         if (controlMode == "POSITION") control_mode = POSITION;
    else if (controlMode == "VELOCITY") control_mode = VELOCITY;
    else if (controlMode == "TORQUE")   control_mode = TORQUE;
    else                                control_mode = UNKNOWN;

    ////////////////////////////////////////////////////////////////////////////////////////////////
    
    try
    {
        auto mujocoSim = std::make_shared<MuJoCoInterface>(xmlLocation,
                                                           publisherName,
                                                           subscriberName,
                                                           control_mode,
                                                           simulationFrequency,
                                                           visualizationFrequency);

        mujocoSim->set_feedback_gains(proportionalGain, integralGain, derivativeGain);

        mujocoSim->set_camera_properties({camera_focal_point[0], camera_focal_point[1], camera_focal_point[2]},
                                          camera_distance,
                                          camera_azimuth, 
                                          camera_elevation,
                                          camera_orthographic);
                                      

        rclcpp::spin(mujocoSim);                                                                    // Run the simulation indefinitely
    }
    catch(const std::exception &exception)
    {
        std::cerr << exception.what() << "\n";
    }
    
    rclcpp::shutdown();                                                                             // Shut down
    
    return 0;
}

