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
    if (argc != 2)
    {
        std::cerr << "Usage: mujoco_interface_node path/to/scene.xml\n";
        return 1;
    }
    
    rclcpp::init(argc, argv);                                                                       // Starts up ROS2
       
    std::string model_path = argv[1];                                                               // Convert char to string
    
    auto node = std::make_shared<MuJoCoInterface>(model_path);                                      // Create the node
    
    rclcpp::spin(node);                                                                             // Run the node
        
    rclcpp::shutdown();                                                                             // Shut down
    
    return 0;
}

