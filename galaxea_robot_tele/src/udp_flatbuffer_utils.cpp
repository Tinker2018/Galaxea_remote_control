#include "udp_flatbuffer_utils.hpp"

namespace galaxea_robot_tele {

flatbuffers::Offset<robot_msg_fbs::Robot2PcWrapper> UDPFlatbufferUtils::encode_joint_state(
    flatbuffers::FlatBufferBuilder& builder,
    robot_msg_fbs::JointPoseSubType sub_type,
    const sensor_msgs::msg::JointState& ros_msg) {
    
    std::vector<float> positions(ros_msg.position.begin(), ros_msg.position.end());
    std::vector<float> velocities(ros_msg.velocity.begin(), ros_msg.velocity.end());
    std::vector<float> efforts(ros_msg.effort.begin(), ros_msg.effort.end());
    
    auto names_vec = builder.CreateVectorOfStrings(ros_msg.name);
    auto positions_vec = builder.CreateVector(positions);
    auto velocities_vec = builder.CreateVector(velocities);
    auto efforts_vec = builder.CreateVector(efforts);

    // 修复：CreateJointState 添加 robot_msg_fbs:: 命名空间
    auto js = robot_msg_fbs::CreateJointState(
        builder,
        sub_type,
        names_vec, positions_vec, velocities_vec, efforts_vec,
        static_cast<int64_t>(ros_msg.header.stamp.sec),
        static_cast<int64_t>(ros_msg.header.stamp.nanosec),
        builder.CreateString(ros_msg.header.frame_id)
    );

    // 修复：CreateRobot2PcWrapper 添加 robot_msg_fbs:: 命名空间
    return robot_msg_fbs::CreateRobot2PcWrapper(
        builder,
        robot_msg_fbs::Robot2PcMsg_JointState,
        js.Union()
    );
}

flatbuffers::Offset<robot_msg_fbs::Robot2PcWrapper> UDPFlatbufferUtils::encode_pose_stamped(
    flatbuffers::FlatBufferBuilder& builder,
    robot_msg_fbs::JointPoseSubType sub_type,
    const geometry_msgs::msg::PoseStamped& ros_msg) {
    
    // 修复：CreatePose 添加 robot_msg_fbs:: 命名空间
    auto pose = robot_msg_fbs::CreatePose(
        builder,
        ros_msg.pose.position.x, ros_msg.pose.position.y, ros_msg.pose.position.z,
        ros_msg.pose.orientation.x, ros_msg.pose.orientation.y, 
        ros_msg.pose.orientation.z, ros_msg.pose.orientation.w
    );

    // 修复：CreatePoseStamped 添加 robot_msg_fbs:: 命名空间
    auto ps = robot_msg_fbs::CreatePoseStamped(
        builder,
        sub_type,
        builder.CreateString(ros_msg.header.frame_id),
        static_cast<int64_t>(ros_msg.header.stamp.sec),
        static_cast<int64_t>(ros_msg.header.stamp.nanosec),
        pose
    );

    // 修复：CreateRobot2PcWrapper 添加 robot_msg_fbs:: 命名空间
    return robot_msg_fbs::CreateRobot2PcWrapper(
        builder,
        robot_msg_fbs::Robot2PcMsg_PoseStamped,
        ps.Union()
    );
}

void UDPFlatbufferUtils::decode_joint_state(
    const robot_msg_fbs::JointState* fb_msg,
    sensor_msgs::msg::JointState& ros_msg) {
    
    ros_msg.header.stamp.sec = static_cast<int32_t>(fb_msg->stamp_sec());
    ros_msg.header.stamp.nanosec = static_cast<uint32_t>(fb_msg->stamp_nanosec());
    ros_msg.header.frame_id = fb_msg->frame_id()->str();

    ros_msg.name.reserve(fb_msg->names()->size());
    for (const auto& name : *fb_msg->names()) {
        ros_msg.name.push_back(name->str());
    }

    ros_msg.position.reserve(fb_msg->positions()->size());
    for (float pos : *fb_msg->positions()) {
        ros_msg.position.push_back(pos);
    }

    ros_msg.velocity.reserve(fb_msg->velocities()->size());
    for (float vel : *fb_msg->velocities()) {
        ros_msg.velocity.push_back(vel);
    }

    ros_msg.effort.reserve(fb_msg->efforts()->size());
    for (float eff : *fb_msg->efforts()) {
        ros_msg.effort.push_back(eff);
    }
}

void UDPFlatbufferUtils::decode_pose_stamped(
    const robot_msg_fbs::PoseStamped* fb_msg,
    geometry_msgs::msg::PoseStamped& ros_msg) {
    
    ros_msg.header.stamp.sec = static_cast<int32_t>(fb_msg->stamp_sec());
    ros_msg.header.stamp.nanosec = static_cast<uint32_t>(fb_msg->stamp_nanosec());
    ros_msg.header.frame_id = fb_msg->frame_id()->str();

    ros_msg.pose.position.x = fb_msg->pose()->x();
    ros_msg.pose.position.y = fb_msg->pose()->y();
    ros_msg.pose.position.z = fb_msg->pose()->z();
    ros_msg.pose.orientation.x = fb_msg->pose()->qx();
    ros_msg.pose.orientation.y = fb_msg->pose()->qy();
    ros_msg.pose.orientation.z = fb_msg->pose()->qz();
    ros_msg.pose.orientation.w = fb_msg->pose()->qw();
}

}  // namespace galaxea_robot_tele
