#include "flatbuffer_utils.hpp"

namespace galaxea_robot_tele {


flatbuffers::Offset<robot_msg_fbs::Robot2PcWrapper> FlatbufferUtils::encode_joint_state(
    flatbuffers::FlatBufferBuilder& builder,
    robot_msg_fbs::RobotMsgType msg_type,
    const sensor_msgs::msg::JointState& ros_msg) {
    
    std::vector<float> positions(ros_msg.position.begin(), ros_msg.position.end());
    std::vector<float> velocities(ros_msg.velocity.begin(), ros_msg.velocity.end());
    std::vector<float> efforts(ros_msg.effort.begin(), ros_msg.effort.end());
    
    auto names_vec = builder.CreateVectorOfStrings(ros_msg.name);
    auto positions_vec = builder.CreateVector(positions);
    auto velocities_vec = builder.CreateVector(velocities);
    auto efforts_vec = builder.CreateVector(efforts);

    auto js = robot_msg_fbs::CreateJointState(
        builder,
        msg_type,
        names_vec, positions_vec, velocities_vec, efforts_vec,
        static_cast<int64_t>(ros_msg.header.stamp.sec),
        static_cast<int64_t>(ros_msg.header.stamp.nanosec),
        builder.CreateString(ros_msg.header.frame_id)
    );

    return robot_msg_fbs::CreateRobot2PcWrapper(
        builder,
        robot_msg_fbs::Robot2PcMsg_JointState,
        js.Union()
    );
}

flatbuffers::Offset<robot_msg_fbs::Robot2PcWrapper> FlatbufferUtils::encode_pose_stamped(
    flatbuffers::FlatBufferBuilder& builder,
    robot_msg_fbs::RobotMsgType msg_type,
    const geometry_msgs::msg::PoseStamped& ros_msg) {
    
    auto pose = robot_msg_fbs::CreatePose(
        builder,
        ros_msg.pose.position.x, ros_msg.pose.position.y, ros_msg.pose.position.z,
        ros_msg.pose.orientation.x, ros_msg.pose.orientation.y, 
        ros_msg.pose.orientation.z, ros_msg.pose.orientation.w
    );

    auto ps = robot_msg_fbs::CreatePoseStamped(
        builder,
        msg_type,
        builder.CreateString(ros_msg.header.frame_id),
        static_cast<int64_t>(ros_msg.header.stamp.sec),
        static_cast<int64_t>(ros_msg.header.stamp.nanosec),
        pose
    );

    return robot_msg_fbs::CreateRobot2PcWrapper(
        builder,
        robot_msg_fbs::Robot2PcMsg_PoseStamped,
        ps.Union()
    );
}

void FlatbufferUtils::decode_joint_state(
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

void FlatbufferUtils::decode_pose_stamped(
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


flatbuffers::Offset<robot_msg_fbs::Header> FlatbufferUtils::encode_header(
    flatbuffers::FlatBufferBuilder& builder,
    const std_msgs::msg::Header& ros_header) {
    return robot_msg_fbs::CreateHeader(
        builder,
        ros_header.stamp.sec,  // ROS Header.sec 是 int32，直接匹配
        ros_header.stamp.nanosec,  // ROS Header.nanosec 是 uint32，直接匹配
        builder.CreateString(ros_header.frame_id)
    );
}

void FlatbufferUtils::decode_header(
    const robot_msg_fbs::Header* fb_header,
    std_msgs::msg::Header& ros_header) {
    ros_header.stamp.sec = fb_header->stamp_sec();
    ros_header.stamp.nanosec = fb_header->stamp_nanosec();
    ros_header.frame_id = fb_header->frame_id()->str();
}

// ===================== 新增：TwistStamped 编解码 =====================
flatbuffers::Offset<robot_msg_fbs::Robot2PcWrapper> FlatbufferUtils::encode_twist_stamped(
    flatbuffers::FlatBufferBuilder& builder,
    robot_msg_fbs::RobotMsgType msg_type,
    const geometry_msgs::msg::TwistStamped& ros_msg) {
    
    // 1. 编码 Twist 核心数据
    auto twist = robot_msg_fbs::CreateTwist(
        builder,
        ros_msg.twist.linear.x,
        ros_msg.twist.linear.y,
        ros_msg.twist.linear.z,
        ros_msg.twist.angular.x,
        ros_msg.twist.angular.y,
        ros_msg.twist.angular.z
    );

    // 2. 编码 TwistStamped
    auto ts = robot_msg_fbs::CreateTwistStamped(
        builder,
        msg_type,
        builder.CreateString(ros_msg.header.frame_id),
        static_cast<int64_t>(ros_msg.header.stamp.sec),
        static_cast<int64_t>(ros_msg.header.stamp.nanosec),
        twist
    );

    // 3. 封装到 Wrapper
    return robot_msg_fbs::CreateRobot2PcWrapper(
        builder,
        robot_msg_fbs::Robot2PcMsg_TwistStamped,
        ts.Union()
    );
}

void FlatbufferUtils::decode_twist_stamped(
    const robot_msg_fbs::TwistStamped* fb_msg,
    geometry_msgs::msg::TwistStamped& ros_msg) {
    
    // 1. 解码 Header 相关
    ros_msg.header.stamp.sec = static_cast<int32_t>(fb_msg->stamp_sec());
    ros_msg.header.stamp.nanosec = static_cast<uint32_t>(fb_msg->stamp_nanosec());
    ros_msg.header.frame_id = fb_msg->frame_id()->str();

    // 2. 解码 Twist 核心数据
    auto twist = fb_msg->twist();
    ros_msg.twist.linear.x = twist->linear_x();
    ros_msg.twist.linear.y = twist->linear_y();
    ros_msg.twist.linear.z = twist->linear_z();
    ros_msg.twist.angular.x = twist->angular_x();
    ros_msg.twist.angular.y = twist->angular_y();
    ros_msg.twist.angular.z = twist->angular_z();
}

// ===================== 新增：MotorControl 编解码 =====================
flatbuffers::Offset<robot_msg_fbs::Robot2PcWrapper> FlatbufferUtils::encode_motor_control(
    flatbuffers::FlatBufferBuilder& builder,
    robot_msg_fbs::RobotMsgType msg_type,
    const hdas_msg::msg::MotorControl& ros_msg) {
    
    // 1. 编码 Header
    auto header = encode_header(builder, ros_msg.header);

    // 2. 编码数组字段
    auto p_des_vec = builder.CreateVector(ros_msg.p_des);
    auto v_des_vec = builder.CreateVector(ros_msg.v_des);
    auto kp_vec = builder.CreateVector(ros_msg.kp);
    auto kd_vec = builder.CreateVector(ros_msg.kd);
    auto t_ff_vec = builder.CreateVector(ros_msg.t_ff);

    // 3. 编码 MotorControl
    auto mc = robot_msg_fbs::CreateMotorControl(
        builder,
        msg_type,
        header,
        builder.CreateString(ros_msg.name),
        p_des_vec,
        v_des_vec,
        kp_vec,
        kd_vec,
        t_ff_vec,
        ros_msg.mode
    );

    // 4. 封装到 Wrapper
    return robot_msg_fbs::CreateRobot2PcWrapper(
        builder,
        robot_msg_fbs::Robot2PcMsg_MotorControl,
        mc.Union()
    );
}

void FlatbufferUtils::decode_motor_control(
    const robot_msg_fbs::MotorControl* fb_msg,
    hdas_msg::msg::MotorControl& ros_msg) {
    
    // 1. 解码 Header
    decode_header(fb_msg->header(), ros_msg.header);

    // 2. 解码基础字段
    ros_msg.name = fb_msg->name()->str();
    ros_msg.mode = fb_msg->mode();

    // 3. 解码数组字段
    const auto* p_des = fb_msg->p_des();
    ros_msg.p_des.assign(p_des->begin(), p_des->end());

    const auto* v_des = fb_msg->v_des();
    ros_msg.v_des.assign(v_des->begin(), v_des->end());

    const auto* kp = fb_msg->kp();
    ros_msg.kp.assign(kp->begin(), kp->end());

    const auto* kd = fb_msg->kd();
    ros_msg.kd.assign(kd->begin(), kd->end());

    const auto* t_ff = fb_msg->t_ff();
    ros_msg.t_ff.assign(t_ff->begin(), t_ff->end());
}

}  // namespace galaxea_robot_tele
