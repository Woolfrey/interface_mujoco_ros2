#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <mujoco.h>
#include <GLFW/glfw3.h>
#include <chrono>  // Include this for the chrono literals

using namespace std::chrono_literals;  // Use this namespace for time literals

class MujocoInterfaceNode : public rclcpp::Node
{
public:
    MujocoInterfaceNode() : Node("mujoco_interface_node")
    {
        // Initialize MuJoCo
        if (!glfwInit())
        {
            RCLCPP_ERROR(this->get_logger(), "Could not initialize GLFW");
            return;
        }

        m = mj_loadXML("path/to/your/model.xml", NULL, error, 1000);
        if (!m)
        {
            RCLCPP_ERROR(this->get_logger(), "Could not load model: %s", error);
            return;
        }

        d = mj_makeData(m);
        if (!d)
        {
            RCLCPP_ERROR(this->get_logger(), "Could not make data");
            return;
        }

        // Publisher
        publisher_ = this->create_publisher<std_msgs::msg::String>("mujoco_output", 10);

        // Timer to publish data periodically
        timer_ = this->create_wall_timer(
            500ms, std::bind(&MujocoInterfaceNode::publish_data, this));

        // Subscriber
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "mujoco_input", 10, std::bind(&MujocoInterfaceNode::handle_input, this, std::placeholders::_1));
    }

    ~MujocoInterfaceNode()
    {
        mj_deleteData(d);
        mj_deleteModel(m);
        glfwTerminate();
    }

private:
    void publish_data()
    {
        auto message = std_msgs::msg::String();
        message.data = "Hello from MuJoCo interface node!";
        publisher_->publish(message);
    }

    void handle_input(const std_msgs::msg::String::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data.c_str());
        // Process the input and communicate with MuJoCo as needed
    }

    mjModel* m = nullptr;
    mjData* d = nullptr;
    char error[1000];

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MujocoInterfaceNode>());
    rclcpp::shutdown();
    return 0;
}

