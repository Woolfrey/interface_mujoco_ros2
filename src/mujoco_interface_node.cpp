#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <iostream>

class MujocoInterfaceNode : public rclcpp::Node
{
public:
    MujocoInterfaceNode(const std::string &filePath) : Node("mujoco_interface_node")
    {
        // Initialize GLFW
        if (!glfwInit())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize GLFW");
            rclcpp::shutdown();
            return;
        }

        // Create a GLFW window
        window_ = glfwCreateWindow(1200, 900, "MuJoCo Visualization", NULL, NULL);
        if (!window_)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to create GLFW window");
            glfwTerminate();
            rclcpp::shutdown();
            return;
        }

        // Make the OpenGL context current
        glfwMakeContextCurrent(window_);
        glfwSwapInterval(1); // Set swap interval for vsync

        // Load the MuJoCo model and create the rendering context
        if (!loadModel(filePath))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load MuJoCo model");
            rclcpp::shutdown();
            return;
        }

        // Initialize MuJoCo rendering context
        mjv_defaultCamera(&cam_);
        mjv_defaultOption(&opt_);
        mjv_defaultPerturb(&pert_);
        mjr_defaultContext(&con_);
        mjv_makeScene(mjModel_, &scn_, 1000);

        // Set up the camera view
        setupCamera();

        // Create MuJoCo rendering context
        glfwMakeContextCurrent(window_);
        mjr_makeContext(mjModel_, &con_, mjFONTSCALE_100);

        joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&MujocoInterfaceNode::update, this));
    }

    ~MujocoInterfaceNode()
    {
        mj_deleteData(mjData_);
        mj_deleteModel(mjModel_);
        mjv_freeScene(&scn_);
        mjr_freeContext(&con_);
        glfwDestroyWindow(window_);
        glfwTerminate();
    }

private:
    bool loadModel(const std::string &filePath)
    {
        char error[1000] = "Could not load binary model";
        mjModel_ = mj_loadXML(filePath.c_str(), nullptr, error, 1000);
        if (!mjModel_)
        {
            RCLCPP_ERROR(this->get_logger(), "Error loading model: %s", error);
            return false;
        }
        mjData_ = mj_makeData(mjModel_);
        return true;
    }

    void setupCamera()
    {
    // Initialize the camera with default values
    mjv_defaultCamera(&cam_);

    // Set up the camera parameters
    cam_.lookat[0] = 0.0; // X coordinate of the lookat point
    cam_.lookat[1] = 0.0; // Y coordinate of the lookat point
    cam_.lookat[2] = 0.5; // Z coordinate of the lookat point

    cam_.distance = 2.5;  // Distance from the lookat point

    cam_.azimuth = 140.0;  // Azimuth angle in degrees
    cam_.elevation = -20.0; // Elevation angle in degrees (higher value for a higher view)

    cam_.orthographic = 0; // 0 for perspective, 1 for orthographic
}

    void update()
    {
        if (!mjModel_ || !mjData_)
        {
            RCLCPP_ERROR(this->get_logger(), "MuJoCo model or data is not initialized.");
            return;
        }

        // Step the simulation
        mj_step(mjModel_, mjData_);

        // Publish joint states
        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = this->now();
        msg.name.resize(mjModel_->nq);
        msg.position.resize(mjModel_->nq);
        msg.velocity.resize(mjModel_->nv);

        for (int i = 0; i < mjModel_->nq; ++i)
        {
            msg.name[i] = mj_id2name(mjModel_, mjOBJ_JOINT, i);
            msg.position[i] = mjData_->qpos[i];
        }
        for (int i = 0; i < mjModel_->nv; ++i)
        {
            msg.velocity[i] = mjData_->qvel[i];
        }
        joint_state_publisher_->publish(msg);

        // Ensure the OpenGL context is current
        glfwMakeContextCurrent(window_);

        // Update scene and render
        mjv_updateScene(mjModel_, mjData_, &opt_, NULL, &cam_, mjCAT_ALL, &scn_);
        
        // Get framebuffer size
        int width, height;
        glfwGetFramebufferSize(window_, &width, &height);
        mjrRect viewport = {0, 0, width, height};

        // Render scene
        mjr_render(viewport, &scn_, &con_);
        
        // Swap buffers and process events
        glfwSwapBuffers(window_);
        glfwPollEvents();
    }

    mjModel *mjModel_;
    mjData *mjData_;
    mjvCamera cam_;
    mjvOption opt_;
    mjvPerturb pert_;
    mjvScene scn_;
    mjrContext con_;
    GLFWwindow *window_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    if (argc != 2)
    {
        std::cerr << "Usage: mujoco_interface_node <path_to_model.xml>" << std::endl;
        return 1;
    }
    std::string model_path = argv[1];
    auto node = std::make_shared<MujocoInterfaceNode>(model_path);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

