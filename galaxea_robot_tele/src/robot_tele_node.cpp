#include "robot_tele_node.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "robot_msg_generated.h"

namespace galaxea_robot_tele {

RobotTeleNode::RobotTeleNode() : Node("robot_tele_node") {
    std::string config_path = ament_index_cpp::get_package_share_directory("galaxea_robot_tele") + "/config/udp_config.yaml";
    try {
        udp_config_ = UDPSocket::load_config(config_path);
        if (udp_config_.role != "robot") {
            throw std::runtime_error("Config role is not 'robot'");
        }
        udp_socket_ = std::make_unique<UDPSocket>(udp_config_);
        RCLCPP_INFO(this->get_logger(), "Robot UDP initialized successfully");
    } catch (const std::exception& e) {
        RCLCPP_FATAL(this->get_logger(), "Initialization failed: %s", e.what());
        rclcpp::shutdown();
        return;
    }

    init_subscribers();
}

RobotTeleNode::~RobotTeleNode() {}

void RobotTeleNode::init_subscribers() {
    // 修复：枚举值添加 JointPoseSubType:: 嵌套
    sub_left_arm_joint_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/hdas/feedback_arm_left", 10,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
            this->send_joint_state(robot_msg_fbs::JointPoseSubType_JOINT_STATE_LEFT_ARM, *msg);
        }
    );

    sub_right_arm_joint_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/hdas/feedback_arm_right", 10,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
            this->send_joint_state(robot_msg_fbs::JointPoseSubType_JOINT_STATE_RIGHT_ARM, *msg);
        }
    );

    sub_left_gripper_joint_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/hdas/feedback_gripper_left", 10,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
            this->send_joint_state(robot_msg_fbs::JointPoseSubType_JOINT_STATE_LEFT_GRIPPER, *msg);
        }
    );

    sub_right_gripper_joint_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/hdas/feedback_gripper_right", 10,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
            this->send_joint_state(robot_msg_fbs::JointPoseSubType_JOINT_STATE_RIGHT_GRIPPER, *msg);
        }
    );

    sub_left_arm_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/motion_control/pose_ee_arm_left", 10,
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            this->send_pose_stamped(robot_msg_fbs::JointPoseSubType_POSE_STAMPED_LEFT_ARM, *msg);
        }
    );

    sub_right_arm_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/motion_control/pose_ee_arm_right", 10,
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            this->send_pose_stamped(robot_msg_fbs::JointPoseSubType_POSE_STAMPED_RIGHT_ARM, *msg);
        }
    );

    RCLCPP_INFO(this->get_logger(), "Robot subscribers initialized");
}

void RobotTeleNode::send_joint_state(robot_msg_fbs::JointPoseSubType sub_type, const sensor_msgs::msg::JointState& msg) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto wrapper = UDPFlatbufferUtils::encode_joint_state(builder, sub_type, msg);
    robot_msg_fbs::FinishRobot2PcWrapperBuffer(builder, wrapper);

    if (udp_socket_->send(builder.GetBufferPointer(), builder.GetSize())) {
        RCLCPP_DEBUG(this->get_logger(), "Joint state sent (type: %d)", sub_type);
    } else {
        RCLCPP_WARN(this->get_logger(), "Joint state send failed (type: %d)", sub_type);
    }
}

void RobotTeleNode::send_pose_stamped(robot_msg_fbs::JointPoseSubType sub_type, const geometry_msgs::msg::PoseStamped& msg) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto wrapper = UDPFlatbufferUtils::encode_pose_stamped(builder, sub_type, msg);
    robot_msg_fbs::FinishRobot2PcWrapperBuffer(builder, wrapper);

    if (udp_socket_->send(builder.GetBufferPointer(), builder.GetSize())) {
        RCLCPP_DEBUG(this->get_logger(), "Pose stamped sent (type: %d)", sub_type);
    } else {
        RCLCPP_WARN(this->get_logger(), "Pose stamped send failed (type: %d)", sub_type);
    }
}

}  // namespace galaxea_robot_tele

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<galaxea_robot_tele::RobotTeleNode>());
    rclcpp::shutdown();
    return 0;
}
