/**
 * @file   MuJoCoInterface.cpp
 * @author Jon Woolfrey
 * @data   August 2024
 * @brief  Source code for the MuJoCoInterface class with ROS2.
 */
 
#include <MuJoCoInterface.h>
 
  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                         Constructor                                            //
////////////////////////////////////////////////////////////////////////////////////////////////////
MuJoCoInterface::MuJoCoInterface(const std::string &filePath) : Node("mujoco_interface_node")
{
    // Initialise Graphics Library FrameWork (GLFW)
    if (not glfwInit())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize Graphics Library FrameWork (GLFW).");
        rclcpp::shutdown();                                                                     
        return;
    }

    // Create a GLFW window
    _window = glfwCreateWindow(1200, 900, "MuJoCo Visualization", NULL, NULL);
    if (!_window)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to create GLFW window");
        glfwTerminate();
        rclcpp::shutdown();
        return;
    }

    // Make the OpenGL context current
    glfwMakeContextCurrent(_window);
    glfwSwapInterval(1);                                                                            // Set swap interval for vsync

    // Load the MuJoCo model and create the rendering context
    char error[1000] = "Could not load binary model";
    _model = mj_loadXML(filePath.c_str(), nullptr, error, 1000);                                    // Load the model
    if (not _model)
    {
        RCLCPP_ERROR(this->get_logger(), "Error loading model: %s", error);
        rclcpp::shutdown();                                                                         // Shut down this node
    }
    
    _jointState = mj_makeData(_model);                                                              // Link joint state to this model
    
    // Resize the arrays in the message data
    _jointStateMessage.name.resize(_model->nq);
    _jointStateMessage.position.resize(_model->nq);
    _jointStateMessage.velocity.resize(_model->nq);
    _jointStateMessage.effort.resize(_model->nq);
    
    _controlReference.resize(_model->nq);
    _error.resize(_model->nq);
    _errorDerivative.resize(_model->nq);
    _errorIntegral.resize(_model->nq);
    
    // Record names
    for(int i = 0; i < _model->nq; i++)
    {
        _jointStateMessage.name[i] = mj_id2name(_model, mjOBJ_JOINT, i);
    }

    // Initialize MuJoCo rendering context
    mjv_defaultCamera(&_camera);
    mjv_defaultOption(&_renderingOptions);
    mjv_defaultPerturb(&_perturbation);
    mjr_defaultContext(&_context);
    mjv_makeScene(_model, &_scene, 1000);

    // Set up the camera view
    mjv_defaultCamera(&_camera);
    set_camera_properties({0.0, 0.0, 0.5}, 2.0, 140.0, -45.0, false);

    // Create MuJoCo rendering context
    glfwMakeContextCurrent(_window);
    mjr_makeContext(_model, &_context, mjFONTSCALE_100);

    _jointStatePublisher = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);
    
     _jointCommandSubscriber = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "joint_commands",
        1,
        std::bind(&MuJoCoInterface::joint_command_callback, this, std::placeholders::_1));

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
 //                          Sets camera viewing position & angle                                  //
////////////////////////////////////////////////////////////////////////////////////////////////////
void
MuJoCoInterface::set_camera_properties(const std::array<double,3> &focalPoint,
                                       const double &distance,
                                       const double &azimuth,
                                       const double &elevation,
                                       const bool &orthographic)
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
            // NOT YET PROGRAMMED.
            // Need to implement PID.
            
            for(int i = 0; i < _model->nq; i++) _jointState->ctrl[i] = 0.0;
            break;
        }
        case VELOCITY:
        {
            for(int i = 0; i < _model->nq; i++)
            {
                _error[i] = _controlReference[i] - _jointState->qvel[i];                            // Velocity error
                
                _errorIntegral[i] += _error[i]/(double)_simFrequency;                               // Add up errors
                
                _jointState->ctrl[i] = _proportionalGain*_error[i]
                                     + _integralGain*_errorIntegral[i];                             // Apply PI control
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
void
MuJoCoInterface::update_visualization()
{
    // Update 3D rendering
    glfwMakeContextCurrent(_window);                                                                // Ensure the OpenGL context is current
    mjv_updateScene(_model, _jointState, &_renderingOptions, NULL, &_camera, mjCAT_ALL, &_scene);   // Update scene and render
    
    // Get framebuffer size
    int width, height;
    glfwGetFramebufferSize(_window, &width, &height);
    mjrRect viewport = {0, 0, width, height};

    mjr_render(viewport, &_scene, &_context);                                                       // Render scene
    
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
    if (msg->data.size() != _model->nq + 1)                                                         // Expecting one more element for mode
    {
        RCLCPP_WARN(this->get_logger(), "Received joint command with incorrect size");
        return;
    }
    
    // Set control mode based on first digit
     if(msg->data[0] == 0)
     {
        _controlMode = POSITION;
        RCLCPP_ERROR(this->get_logger(), "Position control not yet programmed!");
     }
    else if(msg->data[0] == 1) _controlMode = VELOCITY;
    else if(msg->data[0] == 2) _controlMode = TORQUE;
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Unknown control mode.");
        _controlMode = UNKNOWN;
    }
    
    for(int i = 0; i < _model->nq; i++) _controlReference[i] = msg->data[i+1];                      // Save reference 
}
