#ifndef UDP_FLATBUFFER_UTILS_HPP_
#define UDP_FLATBUFFER_UTILS_HPP_

#include "robot_msg_generated.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "hdas_msg/msg/motor_control.hpp"
#include "flatbuffers/flatbuffers.h"
#include "rclcpp/rclcpp.hpp"

namespace galaxea_robot_tele {

class FlatbufferUtils {
public:
    // 原有：关节状态编解码
    static flatbuffers::Offset<robot_msg_fbs::Robot2PcWrapper> encode_joint_state(
        flatbuffers::FlatBufferBuilder& builder,
        robot_msg_fbs::RobotMsgType msg_type,
        const sensor_msgs::msg::JointState& ros_msg);

    static void decode_joint_state(
        const robot_msg_fbs::JointState* fb_msg,
        sensor_msgs::msg::JointState& ros_msg);

    // 原有：位姿编解码
    static flatbuffers::Offset<robot_msg_fbs::Robot2PcWrapper> encode_pose_stamped(
        flatbuffers::FlatBufferBuilder& builder,
        robot_msg_fbs::RobotMsgType msg_type,
        const geometry_msgs::msg::PoseStamped& ros_msg);

    static void decode_pose_stamped(
        const robot_msg_fbs::PoseStamped* fb_msg,
        geometry_msgs::msg::PoseStamped& ros_msg);

    // 新增：TwistStamped 编解码
    static flatbuffers::Offset<robot_msg_fbs::Robot2PcWrapper> encode_twist_stamped(
        flatbuffers::FlatBufferBuilder& builder,
        robot_msg_fbs::RobotMsgType msg_type,
        const geometry_msgs::msg::TwistStamped& ros_msg);

    static void decode_twist_stamped(
        const robot_msg_fbs::TwistStamped* fb_msg,
        geometry_msgs::msg::TwistStamped& ros_msg);

    // 新增：MotorControl 编解码
    static flatbuffers::Offset<robot_msg_fbs::Robot2PcWrapper> encode_motor_control(
        flatbuffers::FlatBufferBuilder& builder,
        robot_msg_fbs::RobotMsgType msg_type,
        const hdas_msg::msg::MotorControl& ros_msg);

    static void decode_motor_control(
        const robot_msg_fbs::MotorControl* fb_msg,
        hdas_msg::msg::MotorControl& ros_msg);

private:
    // 内部辅助：Header编解码（供MotorControl使用）
    static flatbuffers::Offset<robot_msg_fbs::Header> encode_header(
        flatbuffers::FlatBufferBuilder& builder,
        const std_msgs::msg::Header& ros_header);

    static void decode_header(
        const robot_msg_fbs::Header* fb_header,
        std_msgs::msg::Header& ros_header);
};

}  // namespace galaxea_robot_tele

#endif  // UDP_FLATBUFFER_UTILS_HPP_
