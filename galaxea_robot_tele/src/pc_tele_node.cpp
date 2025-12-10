#include "pc_tele_node.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "robot_msg_generated.h"

namespace galaxea_robot_tele {

PCTeleNode::PCTeleNode() : Node("pc_tele_node"), is_running_(true) {
    std::string config_path = ament_index_cpp::get_package_share_directory("galaxea_robot_tele") + "/config/udp_config.yaml";
    try {
        udp_config_ = UDPSocket::load_config(config_path);
        if (udp_config_.role != "pc") {
            throw std::runtime_error("Config role is not 'pc'");
        }
        udp_socket_ = std::make_unique<UDPSocket>(udp_config_);
        RCLCPP_INFO(this->get_logger(), "PC UDP initialized successfully");
    } catch (const std::exception& e) {
        RCLCPP_FATAL(this->get_logger(), "Initialization failed: %s", e.what());
        rclcpp::shutdown();
        return;
    }


    sub_target_joint_state_arm_left_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/hdas/target_joint_state_arm_left", 10,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
            this->send_joint_state(robot_msg_fbs::RobotMsgType_TARGET_JOINT_STATE_ARM_LEFT, *msg);
        }
    );

    sub_target_joint_state_arm_right_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/hdas/target_joint_state_arm_right", 10,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
            this->send_joint_state(robot_msg_fbs::RobotMsgType_TARGET_JOINT_STATE_ARM_RIGHT, *msg);
        }
    );

    sub_target_position_gripper_left_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/hdas/target_position_gripper_left", 10,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
            this->send_joint_state(robot_msg_fbs::RobotMsgType_TARGET_POSITION_GRIPPER_LEFT, *msg);
        }
    );

    sub_target_position_gripper_right_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/hdas/target_position_gripper_right", 10,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
            this->send_joint_state(robot_msg_fbs::RobotMsgType_TARGET_POSITION_GRIPPER_RIGHT, *msg);
        }
    );

    pub_left_arm_joint_ = this->create_publisher<sensor_msgs::msg::JointState>("/hdas/feedback_arm_left", 10);
    pub_right_arm_joint_ = this->create_publisher<sensor_msgs::msg::JointState>("/hdas/feedback_arm_right", 10);
    pub_left_gripper_joint_ = this->create_publisher<sensor_msgs::msg::JointState>("/hdas/feedback_gripper_left", 10);
    pub_right_gripper_joint_ = this->create_publisher<sensor_msgs::msg::JointState>("/hdas/feedback_gripper_right", 10);
    pub_pose_ee_arm_left_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/motion_control/pose_ee_arm_left", 10);
    pub_pose_ee_arm_right_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/motion_control/pose_ee_arm_right", 10);
    pub_pc_target_pose_arm_left_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/motion_target/target_pose_arm_left", 10);
    pub_pc_target_pose_arm_right_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/motion_target/target_pose_arm_right", 10);
        
    RCLCPP_INFO(this->get_logger(), "PC initialized");

    recv_thread_ = std::thread(&PCTeleNode::recv_loop, this);
}

PCTeleNode::~PCTeleNode() {
    is_running_ = false;
    if (recv_thread_.joinable()) {
        recv_thread_.join();
    }
}



void PCTeleNode::recv_loop() {
    const size_t BUF_SIZE = udp_config_.buffer_size;
    std::vector<uint8_t> buffer(BUF_SIZE);

    while (is_running_ && rclcpp::ok()) {
        ssize_t recv_len = udp_socket_->receive(buffer.data(), BUF_SIZE);
        if (recv_len <= 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        auto wrapper = robot_msg_fbs::GetRobot2PcWrapper(buffer.data());
        if (!wrapper) {
            RCLCPP_WARN(this->get_logger(), "Invalid data received");
            continue;
        }

        switch (wrapper->msg_type()) {
            case robot_msg_fbs::Robot2PcMsg_JointState: {
                auto js = wrapper->msg_as_JointState();
                if (!js) break;
                
                // 修复：枚举值添加 RobotMsgType:: 嵌套
                switch (js->msg_type()) {
                    case robot_msg_fbs::RobotMsgType_FEEDBACK_ARM_LEFT:  // 1
                        parse_joint_state(js, pub_left_arm_joint_);
                        break;
                    case robot_msg_fbs::RobotMsgType_FEEDBACK_ARM_RIGHT: // 2
                        parse_joint_state(js, pub_right_arm_joint_);
                        break;
                    case robot_msg_fbs::RobotMsgType_FEEDBACK_GRIPPER_LEFT: // 3
                        parse_joint_state(js, pub_left_gripper_joint_);
                        break;
                    case robot_msg_fbs::RobotMsgType_FEEDBACK_GRIPPER_RIGHT: // 4
                        parse_joint_state(js, pub_right_gripper_joint_);
                        break;
                    default:
                        RCLCPP_WARN(this->get_logger(), "Unknown joint state type: %d", js->msg_type());
                }
                break;
            }
            case robot_msg_fbs::Robot2PcMsg_PoseStamped: {
                auto ps = wrapper->msg_as_PoseStamped();
                if (!ps) break;
                
                // 修复：枚举值添加 RobotMsgType:: 嵌套
                switch (ps->msg_type()) {
                    case robot_msg_fbs::RobotMsgType_POSE_EE_LEFT_ARM: // 5
                        parse_pose_stamped(ps, pub_pose_ee_arm_left_);
                        break;
                    case robot_msg_fbs::RobotMsgType_POSE_EE_RIGHT_ARM: // 6
                        parse_pose_stamped(ps, pub_pose_ee_arm_right_);
                        break;
                    case robot_msg_fbs::RobotMsgType_TARGET_POSE_ARM_LEFT: // 7
                        parse_pose_stamped(ps, pub_pc_target_pose_arm_left_);
                        break;
                    case robot_msg_fbs::RobotMsgType_TARGET_POSE_ARM_RIGHT: // 8
                        parse_pose_stamped(ps, pub_pc_target_pose_arm_right_);
                        break;
                    default:
                        RCLCPP_WARN(this->get_logger(), "Unknown pose stamped type: %d", ps->msg_type());
                }
                break;
            }
            default:
                RCLCPP_WARN(this->get_logger(), "Unknown message type: %d", wrapper->msg_type());
        }
    }
}

void PCTeleNode::parse_joint_state(const robot_msg_fbs::JointState* fb_msg, 
                                  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher) {
    sensor_msgs::msg::JointState ros_msg;
    FlatbufferUtils::decode_joint_state(fb_msg, ros_msg);
    publisher->publish(ros_msg);
}

void PCTeleNode::parse_pose_stamped(const robot_msg_fbs::PoseStamped* fb_msg,
                                  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher) {
    geometry_msgs::msg::PoseStamped ros_msg;
    FlatbufferUtils::decode_pose_stamped(fb_msg, ros_msg);
    publisher->publish(ros_msg);
}

void PCTeleNode::send_joint_state(robot_msg_fbs::RobotMsgType msg_type, const sensor_msgs::msg::JointState& msg) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto wrapper = FlatbufferUtils::encode_joint_state(builder, msg_type, msg);
    robot_msg_fbs::FinishRobot2PcWrapperBuffer(builder, wrapper);

    if (udp_socket_->send(builder.GetBufferPointer(), builder.GetSize())) {
        RCLCPP_DEBUG(this->get_logger(), "Joint state sent (type: %d)", msg_type);
    } else {
        RCLCPP_WARN(this->get_logger(), "Joint state send failed (type: %d)", msg_type);
    }
}

void PCTeleNode::send_pose_stamped(robot_msg_fbs::RobotMsgType msg_type, const geometry_msgs::msg::PoseStamped& msg) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto wrapper = FlatbufferUtils::encode_pose_stamped(builder, msg_type, msg);
    robot_msg_fbs::FinishRobot2PcWrapperBuffer(builder, wrapper);

    if (udp_socket_->send(builder.GetBufferPointer(), builder.GetSize())) {
        RCLCPP_DEBUG(this->get_logger(), "Pose stamped sent (type: %d)", msg_type);
    } else {
        RCLCPP_WARN(this->get_logger(), "Pose stamped send failed (type: %d)", msg_type);
    }
}



}  // namespace galaxea_robot_tele

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<galaxea_robot_tele::PCTeleNode>());
    rclcpp::shutdown();
    return 0;
}
