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
MuJoCoInterface::MuJoCoInterface() : Node("mujoco_interface_node")
{
    // Load the robot model
    std::string xmlLocation = "";

    this->declare_parameter<std::string>("xml_location", xmlLocation);

    this->get_parameter("xml_location", xmlLocation);

    if (xmlLocation.empty())
    {
        std::string message = "No XML file located in given path '" + xmlLocation + "'. "
                              "Did you set the parameter correctly in the launch file?";
      
        RCLCPP_ERROR(this->get_logger(), message.c_str());
      
        rclcpp::shutdown();
    }
    
    char blah[1000] = "Could not load binary model";
    _model = mj_loadXML(xmlLocation.c_str(), nullptr, blah, 1000);                                  // Load the model
    
    if (not _model)
    {
        RCLCPP_ERROR(this->get_logger(), "Error loading model: %s", blah);
        rclcpp::shutdown();                                                                         // Shut down this node
    }
    
    _jointState = mj_makeData(_model);                                                              // Link joint state to this model
    
    // Resize arrays based on number of joints in model
    _jointStateMessage.name.resize(_model->nq);
    _jointStateMessage.position.resize(_model->nq);
    _jointStateMessage.velocity.resize(_model->nq);
    _jointStateMessage.effort.resize(_model->nq);
    _controlReference.resize(_model->nq);
    _error.resize(_model->nq);
    _errorDerivative.resize(_model->nq);
    _errorIntegral.resize(_model->nq);
    
    // Record joint names
    for(int i = 0; i < _model->nq; i++)
    {
        _jointStateMessage.name[i] = mj_id2name(_model, mjOBJ_JOINT, i);
    }
    
    // Load control gains
    this->get_parameter("proportional_gain", _proportionalGain);
    this->get_parameter("derivative_gain",   _derivativeGain);
    this->get_parameter("integral_gain",    _integralGain);
    
    // Create joint state publisher, joint command subscriber
     std::string publisher_name = "joint_states";
    this->get_parameter("publisher_name", publisher_name);
    _jointStatePublisher = this->create_publisher<sensor_msgs::msg::JointState>(publisher_name, 1);

    std::string subscriber_name = "joint_controls";
    this->get_parameter("subscriber_name", subscriber_name);
    _jointCommandSubscriber =this->create_subscription<std_msgs::msg::Float64MultiArray>(subscriber_name, 1, std::bind(&MuJoCoInterface::joint_command_callback, this, std::placeholders::_1)); 

    // Create simulation timer
    this->get_parameter("simulation_frequency", _simFrequency);
    
    _simTimer = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000/_simFrequency)),
                                        std::bind(&MuJoCoInterface::update_simulation, this));
    
    this->get_parameter("visualization_frequency", _vizFrequency);
    _visTimer = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000/_vizFrequency)),
                                        std::bind(&MuJoCoInterface::update_visualization, this));    

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

    // Initialize MuJoCo rendering context
    mjv_defaultCamera(&_camera);
    mjv_defaultOption(&_renderingOptions);
    mjv_defaultPerturb(&_perturbation);
    mjr_defaultContext(&_context);
    mjv_makeScene(_model, &_scene, 1000);

    // Create MuJoCo rendering context
    glfwMakeContextCurrent(_window);
    mjr_makeContext(_model, &_context, mjFONTSCALE_100);

    // Get and set camera properties
    this->get_parameter("camera_focal_point", _cameraFocalPoint);
    this->get_parameter("camera_distance", _cameraDistance);
    this->get_parameter("camera_azimuth", _cameraAzimuth);
    this->get_parameter("camera_elevation", _cameraElevation);
    this->get_parameter("camera_orthographic", _cameraOrthographic);

    set_camera_properties({_cameraFocalPoint[0], _cameraFocalPoint[1], _cameraFocalPoint[2]},
                          _cameraDistance,
                          _cameraAzimuth,
                          _cameraElevation,
                          _cameraOrthographic);
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
void MuJoCoInterface::set_camera_properties(const std::array<double, 3> &focalPoint,
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
