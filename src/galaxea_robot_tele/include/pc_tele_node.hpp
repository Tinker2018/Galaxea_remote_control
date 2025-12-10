#ifndef PC_TELE_NODE_HPP_
#define PC_TELE_NODE_HPP_

#include "udp_socket.hpp"
#include "flatbuffer_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <atomic>
#include <memory>
#include <thread>

namespace galaxea_robot_tele {

class PCTeleNode : public rclcpp::Node {
public:
    PCTeleNode();
    ~PCTeleNode() override;

private:
    void recv_loop();

    void send_joint_state(robot_msg_fbs::RobotMsgType msg_type, const sensor_msgs::msg::JointState& msg);
    void send_pose_stamped(robot_msg_fbs::RobotMsgType msg_type, const geometry_msgs::msg::PoseStamped& msg);
    void send_twist_stamped(robot_msg_fbs::RobotMsgType msg_type, const geometry_msgs::msg::TwistStamped& msg);
    void send_motor_control(robot_msg_fbs::RobotMsgType msg_type, const hdas_msg::msg::MotorControl& msg);

    void parse_joint_state(const robot_msg_fbs::JointState* fb_msg, 
                          rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher);
    void parse_pose_stamped(const robot_msg_fbs::PoseStamped* fb_msg,
                          rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher);

    std::unique_ptr<UDPSocket> udp_socket_;
    UDPConfig udp_config_;
    std::thread recv_thread_;
    std::atomic<bool> is_running_;
    
    //从PC发往ROBOT的消息
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_target_joint_state_arm_left_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_target_joint_state_arm_right_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_target_position_gripper_left_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_target_position_gripper_right_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_target_pose_arm_left_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_target_pose_arm_right_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_target_speed_chassis_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_target_speed_torso_;
    rclcpp::Subscription<hdas_msg::msg::MotorControl>::SharedPtr sub_control_arm_left_;
    rclcpp::Subscription<hdas_msg::msg::MotorControl>::SharedPtr sub_control_arm_right_;
    
    //从ROBOT回到PC的消息
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_left_arm_joint_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_right_arm_joint_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_left_gripper_joint_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_right_gripper_joint_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_ee_arm_left_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_ee_arm_right_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_target_pose_arm_left_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_target_pose_arm_right_;


};

}  // namespace galaxea_robot_tele

#endif  // PC_TELE_NODE_HPP_
