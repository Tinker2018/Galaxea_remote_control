#ifndef ROBOT_TELE_NODE_HPP_
#define ROBOT_TELE_NODE_HPP_

#include "udp_socket.hpp"
#include "udp_flatbuffer_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <atomic>
#include <memory>

namespace galaxea_robot_tele {

class RobotTeleNode : public rclcpp::Node {
public:
    RobotTeleNode();
    ~RobotTeleNode() override;

private:
    void init_subscribers();
    // 修复：参数类型添加 robot_msg_fbs:: 命名空间
    void send_joint_state(robot_msg_fbs::JointPoseSubType sub_type, const sensor_msgs::msg::JointState& msg);
    void send_pose_stamped(robot_msg_fbs::JointPoseSubType sub_type, const geometry_msgs::msg::PoseStamped& msg);

    std::unique_ptr<UDPSocket> udp_socket_;
    UDPConfig udp_config_;
    
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_left_arm_joint_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_right_arm_joint_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_left_gripper_joint_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_right_gripper_joint_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_left_arm_pose_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_right_arm_pose_;
};

}  // namespace galaxea_robot_tele

#endif  // ROBOT_TELE_NODE_HPP_
