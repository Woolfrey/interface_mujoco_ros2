#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <mujoco/mujoco.h>
#include <iostream>
#include <vector>

class MujocoInterfaceNode : public rclcpp::Node
{
public:
    MujocoInterfaceNode(const std::string &filePath) : Node("mujoco_interface_node")
    {
        if (!loadModel(filePath))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load MuJoCo model");
            rclcpp::shutdown();
        }
        joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&MujocoInterfaceNode::update, this));
    }

    ~MujocoInterfaceNode()
    {
        mj_deleteData(mjData_);
        mj_deleteModel(mjModel_);
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

    void update()
    {
        mj_step(mjModel_, mjData_);
        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = this->now();
        msg.name.resize(mjModel_->nq);
        msg.position.resize(mjModel_->nq);
        msg.velocity.resize(mjModel_->nq);

        for (int i = 0; i < mjModel_->nq; ++i)
        {
            msg.name[i] = mj_id2name(mjModel_, mjOBJ_JOINT, i); // Get joint name
            msg.position[i] = mjData_->qpos[i];
        }
        for (int i = 0; i < mjModel_->nv; ++i)
        {
            msg.velocity[i] = mjData_->qvel[i];
        }
        joint_state_publisher_->publish(msg);
    }

    mjModel *mjModel_;
    mjData *mjData_;
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

