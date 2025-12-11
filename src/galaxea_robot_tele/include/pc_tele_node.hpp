#ifndef PC_TELE_NODE_HPP_
#define PC_TELE_NODE_HPP_

#include "udp_socket.hpp"
#include "flatbuffer_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "hdas_msg/msg/motor_control.hpp"
#include "system_manager_msg/srv/teleop_frame.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <atomic>
#include <memory>
#include <thread>
#include <future>
#include <map>
#include <mutex>

namespace galaxea_robot_tele {

class PCTeleNode : public rclcpp::Node {
public:
    PCTeleNode();
    ~PCTeleNode() override;

private:
    void recv_loop();
    void timer_callback();

    void send_joint_state(robot_msg_fbs::RobotMsgType msg_type, const sensor_msgs::msg::JointState& msg);
    void send_pose_stamped(robot_msg_fbs::RobotMsgType msg_type, const geometry_msgs::msg::PoseStamped& msg);
    void send_twist_stamped(robot_msg_fbs::RobotMsgType msg_type, const geometry_msgs::msg::TwistStamped& msg);
    void send_motor_control(robot_msg_fbs::RobotMsgType msg_type, const hdas_msg::msg::MotorControl& msg);

    void parse_joint_state(const robot_msg_fbs::JointState* fb_msg, rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher);
    void parse_pose_stamped(const robot_msg_fbs::PoseStamped* fb_msg, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher);

    std::unique_ptr<UDPSocket> udp_socket_;
    UDPConfig udp_config_;
    std::thread recv_thread_;
    std::atomic<bool> is_running_;
    
    // 多线程回调组 (关键修改)
    rclcpp::CallbackGroup::SharedPtr cb_group_services_;

    // 统计相关
    rclcpp::TimerBase::SharedPtr timer_;
    std::atomic<uint64_t> stats_topic_recv_count_; 
    std::atomic<uint64_t> stats_topic_send_count_; 
    std::atomic<uint64_t> stats_srv_count_;        
    
    // Subscriptions
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
    
    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_left_arm_joint_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_right_arm_joint_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_left_gripper_joint_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_right_gripper_joint_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_ee_arm_left_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_ee_arm_right_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_target_pose_arm_left_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_target_pose_arm_right_;

    // Services
    rclcpp::Service<system_manager_msg::srv::TeleopFrame>::SharedPtr srv_server_teleop_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_server_start_data_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_server_stop_data_;
    
    // Async Request Management
    std::atomic<uint64_t> req_id_counter_;
    std::mutex srv_map_mutex_;
    std::map<uint64_t, std::shared_ptr<std::promise<system_manager_msg::srv::TeleopFrame::Response>>> pending_teleop_reqs_;
    std::map<uint64_t, std::shared_ptr<std::promise<std_srvs::srv::Trigger::Response>>> pending_start_reqs_;
    std::map<uint64_t, std::shared_ptr<std::promise<std_srvs::srv::Trigger::Response>>> pending_stop_reqs_;
};

}  // namespace galaxea_robot_tele

#endif  // PC_TELE_NODE_HPP_