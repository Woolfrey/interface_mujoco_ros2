/*
 * @file   MuJoCoInterface.cpp
 * @author Jon Woolfrey
 * @data   August 2024
 * @brief  Source code for the MuJoCoInterface class with ROS2.
 */
 
#include <MuJoCoInterface.h>
 
  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                         Constructor                                            //
////////////////////////////////////////////////////////////////////////////////////////////////////
MuJoCoInterface::MuJoCoInterface(const std::string &xmlLocation,
                                 const std::string &jointStateTopicName,
                                 const std::string &jointControlTopicName,
                                 ControlMode controlMode,
                                 int simulationFrequency,
                                 int visualizationFrequency)
                                 : Node("mujoco_interface_node"),
                                   _controlMode(controlMode),
                                   _simFrequency(simulationFrequency)
{
    // Load the robot model
    
    char error[1000] = "Could not load binary model";
    
    _model = mj_loadXML(xmlLocation.c_str(), nullptr, error, 1000);

    if (not _model)
    {
        RCLCPP_ERROR(this->get_logger(), "Error loading model: %s", error);
        
        rclcpp::shutdown();                                                                         // Shut down action server
    }

    _model->opt.timestep = 1/(double)_simFrequency;                                                 // Match MuJoCo to node frequency
   
    std::string ctrlMode;
    
         if(controlMode == POSITION) ctrlMode = "POSITION";
    else if(controlMode == VELOCITY) ctrlMode = "VELOCITY";
    else if(controlMode == TORQUE)   ctrlMode = "TORQUE";
    
    RCLCPP_INFO(this->get_logger(), "Control mode set to %s.", ctrlMode.c_str());

    // Resize arrays based on the number of joints in the model
    _jointState = mj_makeData(_model);                                                              // Initialize joint state
    _jointStateMessage.name.resize(_model->nq);
    _jointStateMessage.position.resize(_model->nq);
    _jointStateMessage.velocity.resize(_model->nq);
    _jointStateMessage.effort.resize(_model->nq);    
    _torqueInput.resize(_model->nq, 0.0);

    // Record joint names
    for (int i = 0; i < _model->nq; ++i)
    {
        _jointStateMessage.name[i] = mj_id2name(_model, mjOBJ_JOINT, i);
    }

    // Create joint state publisher and joint command subscriber
    _jointStatePublisher = this->create_publisher<sensor_msgs::msg::JointState>(jointStateTopicName, 1);
    
    _jointCommandSubscriber = this->create_subscription<std_msgs::msg::Float64MultiArray>(jointControlTopicName, 1,  std::bind(&MuJoCoInterface::joint_command_callback, this, std::placeholders::_1));

    // Initialize Graphics Library FrameWork (GLFW)
    if (!glfwInit())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize Graphics Library FrameWork (GLFW).");
        rclcpp::shutdown();
        return;
    }

    // Create a GLFW window
    _window = glfwCreateWindow(1200, 900, "MuJoCo Visualization", nullptr, nullptr);
    if (!_window)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to create GLFW window");
        glfwTerminate();
        rclcpp::shutdown();
        return;
    }

    // Make the OpenGL context current
    glfwMakeContextCurrent(_window);
    glfwSwapInterval(1);  // Set swap interval for vsync

    // Initialize MuJoCo rendering context
    mjv_defaultCamera(&_camera);
    mjv_defaultOption(&_renderingOptions);
    mjv_defaultPerturb(&_perturbation);
    mjr_defaultContext(&_context);
    mjv_makeScene(_model, &_scene, 1000);

    // Create MuJoCo rendering context
    glfwMakeContextCurrent(_window);
    mjr_makeContext(_model, &_context, mjFONTSCALE_100);
    
    // Create timers
    _simTimer = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000/simulationFrequency)),
                                        std::bind(&MuJoCoInterface::update_simulation, this));

    _visTimer = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000/visualizationFrequency)),
                                        std::bind(&MuJoCoInterface::update_visualization, this));
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                    Update the simulation                                       //
////////////////////////////////////////////////////////////////////////////////////////////////////
void MuJoCoInterface::update_simulation()
{
    if (not _model and not _jointState)
    {
        RCLCPP_ERROR(this->get_logger(), "MuJoCo model or data is not initialized.");
        return;
    }

    if (_controlMode == TORQUE)
    {
        for (int i = 0; i < _model->nq; ++i)
        {
            _jointState->ctrl[i] = _torqueInput[i]                                                  // Transfer torque input
                                 + _jointState->qfrc_bias[i]                                        // Compensate for gravity
                                 - 0.01*_jointState->qvel[i];                                       // Add some damping
                                 
            _torqueInput[i] = 0.0;                                                                  // Clear value
        }
    }

    mj_step(_model, _jointState);                                                                   // Take a step in the simulation

    // Add joint state data to ROS2 message
    for (int i = 0; i < _model->nq; ++i)
    {
        _jointStateMessage.position[i] = _jointState->qpos[i];
        _jointStateMessage.velocity[i] = _jointState->qvel[i];
        _jointStateMessage.effort[i]   = _jointState->actuator_force[i];
    }

    _jointStateMessage.header.stamp = this->get_clock()->now();                                     // Add current time stamp (for rqt)
    
    _jointStatePublisher->publish(_jointStateMessage);                                              // Publish the joint state message

}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                    Handle joint commands                                       //
////////////////////////////////////////////////////////////////////////////////////////////////////
void
MuJoCoInterface::joint_command_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (msg->data.size() != _model->nq)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Received joint command with incorrect size.");
        return;
    }
    else
    {
        switch(_controlMode)
        {
            case POSITION:
            {
                for(int i = 0; i < _model->nq; ++i)
                {
                    _jointState->ctrl[i] = msg->data[i];                                            // Assign control input directly
                }
                break;
            }
            case VELOCITY:
            {
                for(int i = 0; i < _model->nq; ++i)
                {
                    _jointState->ctrl[i] += msg->data[i] / (double)_simFrequency;                   // Integrate velocity to position level
                }
                break;
            }
            case TORQUE:
            {
                for(int i = 0; i < _model->nq; ++i)
                {
                    _torqueInput[i] = msg->data[i];
                }
                break;
            }
            default:
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                     "Unknown control mode. Unable to set joint commands.");
                break;
            }
        }
    }
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                          Sets camera viewing position & angle                                  //
////////////////////////////////////////////////////////////////////////////////////////////////////
void
MuJoCoInterface::set_camera_properties(const std::array<double, 3> &focalPoint,
                                       const double &distance,
                                       const double &azimuth,
                                       const double &elevation,
                                       const bool   &orthographic)
{
    _camera.lookat[0]    = focalPoint[0];
    _camera.lookat[1]    = focalPoint[1];
    _camera.lookat[2]    = focalPoint[2];
    _camera.distance     = distance; 
    _camera.azimuth      = azimuth;    
    _camera.elevation    = elevation;
    _camera.orthographic = orthographic;
}


  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                    Update the 3D simulation                                    //
////////////////////////////////////////////////////////////////////////////////////////////////////
void MuJoCoInterface::update_visualization()
{
    glfwMakeContextCurrent(_window);                                                                // Ensure OpenGL context is current
    
    mjv_updateScene(_model, _jointState, &_renderingOptions, NULL, &_camera, mjCAT_ALL, &_scene);   // Update 3D rendering

    // Get framebuffer size
    int width, height;
    glfwGetFramebufferSize(_window, &width, &height);
    mjrRect viewport = {0, 0, width, height};

    mjr_render(viewport, &_scene, &_context); // Render scene
    
    // Swap buffers and process events
    glfwSwapBuffers(_window);
    glfwPollEvents();
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                           Destructor                                           //
////////////////////////////////////////////////////////////////////////////////////////////////////
MuJoCoInterface::~MuJoCoInterface()
{
    mj_deleteData(_jointState);
    mj_deleteModel(_model);
    mjv_freeScene(&_scene);
    mjr_freeContext(&_context);
    glfwDestroyWindow(_window);
    glfwTerminate();
}     
