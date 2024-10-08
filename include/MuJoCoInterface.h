/**
 * @file   MuJoCoInterface.h
 * @author Jon Woolfrey
 * @data   August 2024
 * @brief  A class for connecting a MuJoCo simulation with ROS2.
 */
 
#ifndef MUJOCOINTERFACE_H
#define MUJOCOINTERFACE_H

#include <GLFW/glfw3.h>                                                                             // Graphics Library Framework; for visualisation
#include <iostream>                                                                                 // std::cerr, std::cout
#include <mujoco/mujoco.h>                                                                          // Dynamic simulation library
#include <rclcpp/rclcpp.hpp>                                                                        // ROS2 C++ libraries.
#include <sensor_msgs/msg/joint_state.hpp>                                                          // For publishing / subscribing to joint states.
#include <std_msgs/msg/float64_multi_array.hpp>

enum ControlMode {POSITION, VELOCITY, TORQUE, UNKNOWN};                                             // This needs a global scope
        
/**
 * This class launches both a MuJoCo simulation, and ROS2 node for communication.
 */
class MuJoCoInterface : public rclcpp::Node
{
    public:
            
        /**
         * Contructor.
         * @param filePath Location of an .xml file specifying a robot model and/or scene.
         */
        MuJoCoInterface(const std::string &xmlLocation,
                        const std::string &jointStateTopicName,
                        const std::string &jointControlTopicName,
                        ControlMode controlMode = TORQUE,
                        int simulationFrequency = 500,
                        int visualizationFrequency = 20);
        
       /**
        * Deconstructor.
        */
        ~MuJoCoInterface();
        
        /**
         * Sets the viewing properties in the window.
         * @param focalPoint Defines the x, y, z coordinate for the focus of the camera.
         * @param distance The distance from said focal point to the camera.
         * @param azimuth The angle of rotation around the focal point.
         * @param elevation The angle of the line of sight relative to the ground plane.
         * @param orthographic Type of projection. True for orthographics, false for perspective.
         */
        void
        set_camera_properties(const std::array<double,3> &focalPoint,
                              const double &distance = 2.0,
                              const double &azimuth = 140.0,
                              const double &elevation = -45.0,
                              const bool   &orthographic = false);
    private:

        std::vector<double> _torqueInput;                                                           ///< Used to store joint commands in torque mode
        
        ControlMode _controlMode;                                                                   ///< POSITION, VELOCITY, or TORQUE
        
        mjModel *_model;                                                                            ///< Underlying model of the robot.
        mjData  *_jointState;                                                                       ///< Joint state data (position, velocity, acceleration)

        mjvCamera  _camera;                                                                         ///< Camera for viewing
        mjvOption  _renderingOptions;                                                               ///< As it says
        mjvPerturb _perturbation;                                                                   ///< Allows manual interaction
        mjvScene   _scene;                                                                          ///< The environment that the robot is rendered in

        mjrContext _context;                                                                        ///< No idea what this does.

        GLFWwindow *_window;                                                                        ///< This displays the robot and environment.

        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr _jointStatePublisher;            ///< As it says on the label

        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr _jointCommandSubscriber;  ///< Subscriber for joint commands
        
        rclcpp::TimerBase::SharedPtr _simTimer, _visTimer;                                          ///< Regulates the ROS2 node

        sensor_msgs::msg::JointState _jointStateMessage;                                            ///< For publishing joint state data over ROS2
        
        int _simFrequency = 500;                                                                    ///< Speed at which the frequency runs
    
        bool _cameraOrthographic = false;                                                           ///< Sets the type of projection
        double _cameraAzimuth    = 45.0;                                                            ///< Rotation around camera focal point
        double _cameraDistance   = 2.5;                                                             ///< Distance from camera focal point
        double _cameraElevation  = -30;                                                             ///< Dictates height of camera
        std::vector<double> _cameraFocalPoint = {0.0, 0.0, 1.0};                                    ///< Where the camera is directed at
      
        /**
         * Updates the robot state, publishes joint state information.
         */
        void
        update_simulation();
        
        /**
         * Updates the visualisation.
         */
        void
        update_visualization();
        
        /**
         * Callback function to handle incoming joint commands.
         * @param msg The message containing joint commands.
         */
        void
        joint_command_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
};

#endif
