#ifndef ROBOT_TELE_NODE_HPP_
#define ROBOT_TELE_NODE_HPP_

#include "udp_socket.hpp"
#include "flatbuffer_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <atomic>
#include <memory>
#include <thread>
#include "system_manager_msg/srv/teleop_frame.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace galaxea_robot_tele {

class RobotTeleNode : public rclcpp::Node {
public:
    RobotTeleNode();
    ~RobotTeleNode() override;

private:
    void recv_loop();
    void timer_callback(); // 统计定时器
    
    void send_joint_state(robot_msg_fbs::RobotMsgType msg_type, const sensor_msgs::msg::JointState& msg);
    void send_pose_stamped(robot_msg_fbs::RobotMsgType msg_type, const geometry_msgs::msg::PoseStamped& msg);

    void parse_joint_state(const robot_msg_fbs::JointState* fb_msg, rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher);
    void parse_pose_stamped(const robot_msg_fbs::PoseStamped* fb_msg, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher);
    void parse_twist_stamped(const robot_msg_fbs::TwistStamped* fb_msg, rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher);
    void parse_motor_control(const robot_msg_fbs::MotorControl* fb_msg, rclcpp::Publisher<hdas_msg::msg::MotorControl>::SharedPtr publisher);

    rclcpp::Client<system_manager_msg::srv::TeleopFrame>::SharedPtr cli_teleop_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_start_data_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_stop_data_;

    std::unique_ptr<UDPSocket> udp_socket_;
    UDPConfig udp_config_;
    std::thread recv_thread_;
    std::atomic<bool> is_running_;

    // 多线程回调组 (关键修改)
    rclcpp::CallbackGroup::SharedPtr cb_group_reentrant_;

    // 统计相关
    rclcpp::TimerBase::SharedPtr timer_;
    std::atomic<uint64_t> stats_topic_recv_count_; 
    std::atomic<uint64_t> stats_topic_send_count_; 
    std::atomic<uint64_t> stats_srv_count_; 
    
    // PC到ROBOT的消息 (Publisher)
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_target_joint_state_arm_left_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_target_joint_state_arm_right_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_target_position_gripper_left_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_target_position_gripper_right_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_target_pose_arm_left_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_target_pose_arm_right_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_target_speed_chassis_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_target_speed_torso_;
    rclcpp::Publisher<hdas_msg::msg::MotorControl>::SharedPtr pub_control_arm_left_;
    rclcpp::Publisher<hdas_msg::msg::MotorControl>::SharedPtr pub_control_arm_right_;

    // ROBOT到PC的消息 (Subscription)
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_feedback_arm_left_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_feedback_arm_right_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_feedback_gripper_left_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_feedback_gripper_right_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_ee_arm_left_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_ee_arm_right_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_target_pose_arm_left_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_target_pose_arm_right_;
};

}  // namespace galaxea_robot_tele

#endif  // ROBOT_TELE_NODE_HPP_