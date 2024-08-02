/**
 * @file   MuJoCoInterface.h
 * @author Jon Woolfrey
 * @data   August 2024
 * @brief  A class for connecting a MuJoCo simulation with ROS2.
 */
 
#ifndef MUJOCOINTERFACE_H
#define MUJOCOINTERFACE_H

#include <rclcpp/rclcpp.hpp>                                                                        // ROS2 C++ libraries.
#include <sensor_msgs/msg/joint_state.hpp>                                                          // For publishing / subscribing to joint states.
#include <mujoco/mujoco.h>                                                                          // Dynamic simulation library
#include <GLFW/glfw3.h>                                                                             // Graphics Library Framework; for visualisation
#include <iostream>                                                                                 // std::cerr, std::cout

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
        MuJoCoInterface(const std::string &filePath);
        
       /**
        * Deconstructor.
        */
        ~MuJoCoInterface();

    private:
    
        mjModel *_model;                                                                            ///< Underlying model of the robot.
        mjData  *_jointState;                                                                       ///< Joint state data (position, velocity, acceleration)

        mjvCamera  _camera;                                                                         ///< Camera for viewing
        mjvOption  _renderingOptions;                                                               ///< As it says
        mjvPerturb _perturbation;                                                                   ///< Allows manual interaction
        mjvScene   _scene;                                                                          ///< The environment that the robot is rendered in

        mjrContext _context;                                                                        ///< No idea what this does.

        GLFWwindow *_window;                                                                        ///< This displays the robot and environment.

        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr _jointStatePublisher;            ///< As it says on the label

        rclcpp::TimerBase::SharedPtr _timer;                                                        ///< Regulates the ROS2 node

        sensor_msgs::msg::JointState _jointStateMessage;                                            ///< For publishing joint state data over ROS2
        
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

        /**
         * Updates the robot state, publishes joint state information, and updates 3D model.
         */
        void
        update();
};

#endif
