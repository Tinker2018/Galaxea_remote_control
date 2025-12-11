#ifndef UDP_FLATBUFFER_UTILS_HPP_
#define UDP_FLATBUFFER_UTILS_HPP_

#include "robot_msg_generated.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "hdas_msg/msg/motor_control.hpp"
#include "system_manager_msg/srv/teleop_frame.hpp"
#include "std_srvs/srv/trigger.hpp" // 已添加
#include "flatbuffers/flatbuffers.h"
#include "rclcpp/rclcpp.hpp"

namespace galaxea_robot_tele {

class FlatbufferUtils {
public:
    // JointState
    static flatbuffers::Offset<robot_msg_fbs::CommWrapper> encode_joint_state(
        flatbuffers::FlatBufferBuilder& builder,
        robot_msg_fbs::RobotMsgType msg_type,
        const sensor_msgs::msg::JointState& ros_msg);
    static void decode_joint_state(
        const robot_msg_fbs::JointState* fb_msg,
        sensor_msgs::msg::JointState& ros_msg);

    // PoseStamped
    static flatbuffers::Offset<robot_msg_fbs::CommWrapper> encode_pose_stamped(
        flatbuffers::FlatBufferBuilder& builder,
        robot_msg_fbs::RobotMsgType msg_type,
        const geometry_msgs::msg::PoseStamped& ros_msg);
    static void decode_pose_stamped(
        const robot_msg_fbs::PoseStamped* fb_msg,
        geometry_msgs::msg::PoseStamped& ros_msg);

    // TwistStamped
    static flatbuffers::Offset<robot_msg_fbs::CommWrapper> encode_twist_stamped(
        flatbuffers::FlatBufferBuilder& builder,
        robot_msg_fbs::RobotMsgType msg_type,
        const geometry_msgs::msg::TwistStamped& ros_msg);
    static void decode_twist_stamped(
        const robot_msg_fbs::TwistStamped* fb_msg,
        geometry_msgs::msg::TwistStamped& ros_msg);

    // MotorControl
    static flatbuffers::Offset<robot_msg_fbs::CommWrapper> encode_motor_control(
        flatbuffers::FlatBufferBuilder& builder,
        robot_msg_fbs::RobotMsgType msg_type,
        const hdas_msg::msg::MotorControl& ros_msg);
    static void decode_motor_control(
        const robot_msg_fbs::MotorControl* fb_msg,
        hdas_msg::msg::MotorControl& ros_msg);

    // --- Teleop Service ---
    static flatbuffers::Offset<robot_msg_fbs::CommWrapper> encode_teleop_req(
        flatbuffers::FlatBufferBuilder& builder,
        uint64_t req_id,
        const system_manager_msg::srv::TeleopFrame::Request& ros_req);
    static void decode_teleop_req(
        const robot_msg_fbs::ServiceData* fb_msg,
        system_manager_msg::srv::TeleopFrame::Request& ros_req);
    static flatbuffers::Offset<robot_msg_fbs::CommWrapper> encode_teleop_resp(
        flatbuffers::FlatBufferBuilder& builder,
        uint64_t req_id,
        const system_manager_msg::srv::TeleopFrame::Response& ros_resp);
    static void decode_teleop_resp(
        const robot_msg_fbs::ServiceData* fb_msg,
        system_manager_msg::srv::TeleopFrame::Response& ros_resp);

    // --- Trigger Service (新增：用于Start/Stop Data) ---
    static flatbuffers::Offset<robot_msg_fbs::CommWrapper> encode_trigger_req(
        flatbuffers::FlatBufferBuilder& builder,
        uint64_t req_id,
        robot_msg_fbs::ServiceType type); // 类型区分Start/Stop

    static void decode_trigger_req(
        const robot_msg_fbs::ServiceData* fb_msg,
        std_srvs::srv::Trigger::Request& ros_req);

    static flatbuffers::Offset<robot_msg_fbs::CommWrapper> encode_trigger_resp(
        flatbuffers::FlatBufferBuilder& builder,
        uint64_t req_id,
        robot_msg_fbs::ServiceType type,
        const std_srvs::srv::Trigger::Response& ros_resp);

    static void decode_trigger_resp(
        const robot_msg_fbs::ServiceData* fb_msg,
        std_srvs::srv::Trigger::Response& ros_resp);

private:
    static flatbuffers::Offset<robot_msg_fbs::Header> encode_header(
        flatbuffers::FlatBufferBuilder& builder,
        const std_msgs::msg::Header& ros_header);
    static void decode_header(
        const robot_msg_fbs::Header* fb_header,
        std_msgs::msg::Header& ros_header);
};

}  // namespace galaxea_robot_tele

#endif  // UDP_FLATBUFFER_UTILS_HPP_