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
                                 ControlMode controlMode)
                                 : Node("mujoco_interface_node"),
                                   _controlMode(controlMode)
{
    // Load the robot model
    char error[1000] = "Could not load binary model";
    _model = mj_loadXML(xmlLocation.c_str(), nullptr, error, 1000);

    if (!_model)
    {
        RCLCPP_ERROR(this->get_logger(), "Error loading model: %s", error);
        rclcpp::shutdown();
    }

    _jointState = mj_makeData(_model);  // Initialize joint state

    // Resize arrays based on the number of joints in the model
    _jointStateMessage.name.resize(_model->nq);
    _jointStateMessage.position.resize(_model->nq);
    _jointStateMessage.velocity.resize(_model->nq);
    _jointStateMessage.effort.resize(_model->nq);
    _error.resize(_model->nq);
    _errorDerivative.resize(_model->nq);
    _errorIntegral.resize(_model->nq);
    
    _controlReference.resize(_model->nq, 0.0);

    // Record joint names
    for (int i = 0; i < _model->nq; i++)
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
    
    _simTimer = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000/_simFrequency)),
                                        std::bind(&MuJoCoInterface::update_simulation, this));

    _visTimer = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000/_vizFrequency)),
                                        std::bind(&MuJoCoInterface::update_visualization, this));
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

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                               Set the gains for feedback control                               //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool
MuJoCoInterface::set_feedback_gains(const double &proportional,
                                    const double &integral,
                                    const double &derivative)
{
    if(proportional < 0 or integral < 0 or derivative < 0)
    {
        RCLCPP_WARN(this->get_logger(), "Gains cannot be negative.");
        
        return false;
    }
    else
    {
        _proportionalGain = proportional;
        _derivativeGain   = derivative;
        _integralGain     = integral;
        
        return true;
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
 //                                    Update the simulation                                       //
////////////////////////////////////////////////////////////////////////////////////////////////////
void
MuJoCoInterface::update_simulation()
{
    if((not _model) and (not _jointState))
    {
        RCLCPP_ERROR(this->get_logger(), "MuJoCo model or data is not initialized.");
        return;
    }
    
    // Compute control input based on mode
    switch(_controlMode)
    {
        case POSITION:
        {
            for(int i = 0; i < _model->nq; i++)
            {     
                double error = _controlReference[i] - _jointState->qpos[i];                         // Position error
                
                _errorIntegral[i] += error / (double)_simFrequency;                                 // Cumulative error
                
                double errorDerivative = (error - _error[i]) * _simFrequency;                       // Change in error over time
                
                     _jointState->ctrl[i] = _proportionalGain * error
                                         + _integralGain * _errorIntegral[i]
                                         + _derivativeGain * errorDerivative;                       // Apply PID control
                    
                    _error[i] = error;                                                              // Update error for next iteration           
            }  
            
            break;
        }
        case VELOCITY:
        {
            for(int i = 0; i < _model->nq; i++)
            {
                _error[i] = _controlReference[i] - _jointState->qvel[i];                            // Velocity error
                 
                _errorIntegral[i] += _error[i] / (double)_simFrequency;                             // Accumulated error (i.e. position error)
                
                _jointState->ctrl[i] = _proportionalGain * _error[i]
                                     + _integralGain * _errorIntegral[i];                           // Apply PI control
            }
            
            break;
        }
        case TORQUE:
        {
            for(int i = 0; i < _model->nq; i++) _jointState->ctrl[i] = _controlReference[i];
            
            break;
        }
        default:
        {
            for(int i = 0; i < _model->nq; i++) _jointState->ctrl[i] = 0.0;
        }
    }

    mj_step(_model, _jointState);                                                                   // Take a step in the simulation

    // Add joint state data to ROS2 message, then publish
    for(int i = 0; i < _model->nq; i++)
    {
        _jointStateMessage.position[i] = _jointState->qpos[i];
        _jointStateMessage.velocity[i] = _jointState->qvel[i];
        _jointStateMessage.effort[i]   = _jointState->actuator_force[i];
    }
    
    _jointStatePublisher->publish(_jointStateMessage);                                              // As it says
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                    Update the 3D simulation                                    //
////////////////////////////////////////////////////////////////////////////////////////////////////
void MuJoCoInterface::update_visualization()
{
    glfwMakeContextCurrent(_window); // Ensure OpenGL context is current
    
    // Update 3D rendering
    mjv_updateScene(_model, _jointState, &_renderingOptions, NULL, &_camera, mjCAT_ALL, &_scene);

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
 //                                    Handle joint commands                                       //
////////////////////////////////////////////////////////////////////////////////////////////////////
void
MuJoCoInterface::joint_command_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (msg->data.size() != _model->nq)                                                             // Expecting one more element for mode
    {
        RCLCPP_WARN(this->get_logger(), "Received joint command with incorrect size.");
        return;
    }
    
    for(int i = 0; i < _model->nq; i++) _controlReference[i] = msg->data[i];                        // Save reference 
}
