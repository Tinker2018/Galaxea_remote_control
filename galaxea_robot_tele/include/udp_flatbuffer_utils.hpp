#ifndef UDP_FLATBUFFER_UTILS_HPP_
#define UDP_FLATBUFFER_UTILS_HPP_

#include "robot_msg_generated.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "flatbuffers/flatbuffers.h"
#include "rclcpp/rclcpp.hpp"

namespace galaxea_robot_tele {

class UDPFlatbufferUtils {
public:
    // 编码关节状态消息（修复：参数类型添加 robot_msg_fbs:: 命名空间）
    static flatbuffers::Offset<robot_msg_fbs::Robot2PcWrapper> encode_joint_state(
        flatbuffers::FlatBufferBuilder& builder,
        robot_msg_fbs::JointPoseSubType sub_type,
        const sensor_msgs::msg::JointState& ros_msg);

    // 编码位姿消息（修复：参数类型添加 robot_msg_fbs:: 命名空间）
    static flatbuffers::Offset<robot_msg_fbs::Robot2PcWrapper> encode_pose_stamped(
        flatbuffers::FlatBufferBuilder& builder,
        robot_msg_fbs::JointPoseSubType sub_type,
        const geometry_msgs::msg::PoseStamped& ros_msg);

    // 解码关节状态消息（修复：参数类型添加 robot_msg_fbs:: 命名空间）
    static void decode_joint_state(
        const robot_msg_fbs::JointState* fb_msg,
        sensor_msgs::msg::JointState& ros_msg);

    // 解码位姿消息（修复：参数类型添加 robot_msg_fbs:: 命名空间）
    static void decode_pose_stamped(
        const robot_msg_fbs::PoseStamped* fb_msg,
        geometry_msgs::msg::PoseStamped& ros_msg);
};

}  // namespace galaxea_robot_tele

#endif  // UDP_FLATBUFFER_UTILS_HPP_
