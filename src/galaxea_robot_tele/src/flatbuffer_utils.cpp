#include "flatbuffer_utils.hpp"

namespace galaxea_robot_tele {

flatbuffers::Offset<robot_msg_fbs::Header> FlatbufferUtils::encode_header(
    flatbuffers::FlatBufferBuilder& builder,
    const std_msgs::msg::Header& ros_header) {
    auto frame_id = builder.CreateString(ros_header.frame_id);
    return robot_msg_fbs::CreateHeader(
        builder,
        static_cast<int32_t>(ros_header.stamp.sec),
        static_cast<uint32_t>(ros_header.stamp.nanosec),
        frame_id
    );
}

void FlatbufferUtils::decode_header(
    const robot_msg_fbs::Header* fb_header,
    std_msgs::msg::Header& ros_header) {
    if (!fb_header) return;
    ros_header.stamp.sec = fb_header->stamp_sec();
    ros_header.stamp.nanosec = fb_header->stamp_nanosec();
    if (fb_header->frame_id()) ros_header.frame_id = fb_header->frame_id()->str();
}

// JointState
flatbuffers::Offset<robot_msg_fbs::CommWrapper> FlatbufferUtils::encode_joint_state(
    flatbuffers::FlatBufferBuilder& builder,
    robot_msg_fbs::RobotMsgType msg_type,
    const sensor_msgs::msg::JointState& ros_msg) {
    
    auto header = encode_header(builder, ros_msg.header);
    std::vector<float> positions(ros_msg.position.begin(), ros_msg.position.end());
    std::vector<float> velocities(ros_msg.velocity.begin(), ros_msg.velocity.end());
    std::vector<float> efforts(ros_msg.effort.begin(), ros_msg.effort.end());
    auto names_vec = builder.CreateVectorOfStrings(ros_msg.name);
    auto positions_vec = builder.CreateVector(positions);
    auto velocities_vec = builder.CreateVector(velocities);
    auto efforts_vec = builder.CreateVector(efforts);

    auto js = robot_msg_fbs::CreateJointState(builder, msg_type, header, names_vec, positions_vec, velocities_vec, efforts_vec);
    return robot_msg_fbs::CreateCommWrapper(builder, robot_msg_fbs::AnyMsg_JointState, js.Union());
}

void FlatbufferUtils::decode_joint_state(const robot_msg_fbs::JointState* fb_msg, sensor_msgs::msg::JointState& ros_msg) {
    decode_header(fb_msg->header(), ros_msg.header);
    if (fb_msg->names()) {
        ros_msg.name.reserve(fb_msg->names()->size());
        for (const auto& name : *fb_msg->names()) ros_msg.name.push_back(name->str());
    }
    if (fb_msg->positions()) ros_msg.position.assign(fb_msg->positions()->begin(), fb_msg->positions()->end());
    if (fb_msg->velocities()) ros_msg.velocity.assign(fb_msg->velocities()->begin(), fb_msg->velocities()->end());
    if (fb_msg->efforts()) ros_msg.effort.assign(fb_msg->efforts()->begin(), fb_msg->efforts()->end());
}

// PoseStamped
flatbuffers::Offset<robot_msg_fbs::CommWrapper> FlatbufferUtils::encode_pose_stamped(
    flatbuffers::FlatBufferBuilder& builder,
    robot_msg_fbs::RobotMsgType msg_type,
    const geometry_msgs::msg::PoseStamped& ros_msg) {
    auto header = encode_header(builder, ros_msg.header);
    auto pose = robot_msg_fbs::CreatePose(builder, ros_msg.pose.position.x, ros_msg.pose.position.y, ros_msg.pose.position.z,
        ros_msg.pose.orientation.x, ros_msg.pose.orientation.y, ros_msg.pose.orientation.z, ros_msg.pose.orientation.w);
    auto ps = robot_msg_fbs::CreatePoseStamped(builder, msg_type, header, pose);
    return robot_msg_fbs::CreateCommWrapper(builder, robot_msg_fbs::AnyMsg_PoseStamped, ps.Union());
}

void FlatbufferUtils::decode_pose_stamped(const robot_msg_fbs::PoseStamped* fb_msg, geometry_msgs::msg::PoseStamped& ros_msg) {
    decode_header(fb_msg->header(), ros_msg.header);
    if (fb_msg->pose()) {
        ros_msg.pose.position.x = fb_msg->pose()->x();
        ros_msg.pose.position.y = fb_msg->pose()->y();
        ros_msg.pose.position.z = fb_msg->pose()->z();
        ros_msg.pose.orientation.x = fb_msg->pose()->qx();
        ros_msg.pose.orientation.y = fb_msg->pose()->qy();
        ros_msg.pose.orientation.z = fb_msg->pose()->qz();
        ros_msg.pose.orientation.w = fb_msg->pose()->qw();
    }
}

// TwistStamped
flatbuffers::Offset<robot_msg_fbs::CommWrapper> FlatbufferUtils::encode_twist_stamped(
    flatbuffers::FlatBufferBuilder& builder,
    robot_msg_fbs::RobotMsgType msg_type,
    const geometry_msgs::msg::TwistStamped& ros_msg) {
    auto header = encode_header(builder, ros_msg.header);
    auto twist = robot_msg_fbs::CreateTwist(builder, ros_msg.twist.linear.x, ros_msg.twist.linear.y, ros_msg.twist.linear.z,
        ros_msg.twist.angular.x, ros_msg.twist.angular.y, ros_msg.twist.angular.z);
    auto ts = robot_msg_fbs::CreateTwistStamped(builder, msg_type, header, twist);
    return robot_msg_fbs::CreateCommWrapper(builder, robot_msg_fbs::AnyMsg_TwistStamped, ts.Union());
}

void FlatbufferUtils::decode_twist_stamped(const robot_msg_fbs::TwistStamped* fb_msg, geometry_msgs::msg::TwistStamped& ros_msg) {
    decode_header(fb_msg->header(), ros_msg.header);
    if (fb_msg->twist()) {
        ros_msg.twist.linear.x = fb_msg->twist()->linear_x();
        ros_msg.twist.linear.y = fb_msg->twist()->linear_y();
        ros_msg.twist.linear.z = fb_msg->twist()->linear_z();
        ros_msg.twist.angular.x = fb_msg->twist()->angular_x();
        ros_msg.twist.angular.y = fb_msg->twist()->angular_y();
        ros_msg.twist.angular.z = fb_msg->twist()->angular_z();
    }
}

// MotorControl
flatbuffers::Offset<robot_msg_fbs::CommWrapper> FlatbufferUtils::encode_motor_control(
    flatbuffers::FlatBufferBuilder& builder,
    robot_msg_fbs::RobotMsgType msg_type,
    const hdas_msg::msg::MotorControl& ros_msg) {
    auto header = encode_header(builder, ros_msg.header);
    auto p_des_vec = builder.CreateVector(ros_msg.p_des);
    auto v_des_vec = builder.CreateVector(ros_msg.v_des);
    auto kp_vec = builder.CreateVector(ros_msg.kp);
    auto kd_vec = builder.CreateVector(ros_msg.kd);
    auto t_ff_vec = builder.CreateVector(ros_msg.t_ff);
    auto name_str = builder.CreateString(ros_msg.name);
    auto mc = robot_msg_fbs::CreateMotorControl(builder, msg_type, header, name_str, p_des_vec, v_des_vec, kp_vec, kd_vec, t_ff_vec, ros_msg.mode);
    return robot_msg_fbs::CreateCommWrapper(builder, robot_msg_fbs::AnyMsg_MotorControl, mc.Union());
}

void FlatbufferUtils::decode_motor_control(const robot_msg_fbs::MotorControl* fb_msg, hdas_msg::msg::MotorControl& ros_msg) {
    decode_header(fb_msg->header(), ros_msg.header);
    if (fb_msg->name()) ros_msg.name = fb_msg->name()->str();
    ros_msg.mode = fb_msg->mode();
    if(fb_msg->p_des()) ros_msg.p_des.assign(fb_msg->p_des()->begin(), fb_msg->p_des()->end());
    if(fb_msg->v_des()) ros_msg.v_des.assign(fb_msg->v_des()->begin(), fb_msg->v_des()->end());
    if(fb_msg->kp()) ros_msg.kp.assign(fb_msg->kp()->begin(), fb_msg->kp()->end());
    if(fb_msg->kd()) ros_msg.kd.assign(fb_msg->kd()->begin(), fb_msg->kd()->end());
    if(fb_msg->t_ff()) ros_msg.t_ff.assign(fb_msg->t_ff()->begin(), fb_msg->t_ff()->end());
}

// Teleop Service
flatbuffers::Offset<robot_msg_fbs::CommWrapper> FlatbufferUtils::encode_teleop_req(
    flatbuffers::FlatBufferBuilder& builder,
    uint64_t req_id,
    const system_manager_msg::srv::TeleopFrame::Request& ros_req) {
    auto req_data = robot_msg_fbs::CreateTeleopFrameRequest(builder, ros_req.action);
    auto srv_data = robot_msg_fbs::CreateServiceData(builder, req_id, robot_msg_fbs::ServiceType_REQ_TELEOP_FRAME, req_data, 0, 0, 0);
    return robot_msg_fbs::CreateCommWrapper(builder, robot_msg_fbs::AnyMsg_ServiceData, srv_data.Union());
}

void FlatbufferUtils::decode_teleop_req(const robot_msg_fbs::ServiceData* fb_msg, system_manager_msg::srv::TeleopFrame::Request& ros_req) {
    if (fb_msg->teleop_req()) ros_req.action = fb_msg->teleop_req()->action();
}

flatbuffers::Offset<robot_msg_fbs::CommWrapper> FlatbufferUtils::encode_teleop_resp(
    flatbuffers::FlatBufferBuilder& builder,
    uint64_t req_id,
    const system_manager_msg::srv::TeleopFrame::Response& ros_resp) {
    auto msg_str = builder.CreateString(ros_resp.message);
    auto resp_data = robot_msg_fbs::CreateTeleopFrameResponse(builder, ros_resp.success, msg_str);
    auto srv_data = robot_msg_fbs::CreateServiceData(builder, req_id, robot_msg_fbs::ServiceType_RESP_TELEOP_FRAME, 0, resp_data, 0, 0);
    return robot_msg_fbs::CreateCommWrapper(builder, robot_msg_fbs::AnyMsg_ServiceData, srv_data.Union());
}

void FlatbufferUtils::decode_teleop_resp(const robot_msg_fbs::ServiceData* fb_msg, system_manager_msg::srv::TeleopFrame::Response& ros_resp) {
    if (fb_msg->teleop_resp()) {
        ros_resp.success = fb_msg->teleop_resp()->success();
        if (fb_msg->teleop_resp()->message()) ros_resp.message = fb_msg->teleop_resp()->message()->str();
    }
}

// --- Trigger Service (新增) ---
flatbuffers::Offset<robot_msg_fbs::CommWrapper> FlatbufferUtils::encode_trigger_req(
    flatbuffers::FlatBufferBuilder& builder,
    uint64_t req_id,
    robot_msg_fbs::ServiceType type) {
    auto req_data = robot_msg_fbs::CreateTriggerRequest(builder, 0); // 占位
    // type 参数决定是 Start 还是 Stop
    auto srv_data = robot_msg_fbs::CreateServiceData(builder, req_id, type, 0, 0, req_data, 0);
    return robot_msg_fbs::CreateCommWrapper(builder, robot_msg_fbs::AnyMsg_ServiceData, srv_data.Union());
}

void FlatbufferUtils::decode_trigger_req(
    const robot_msg_fbs::ServiceData* /*fb_msg*/,
    std_srvs::srv::Trigger::Request& /*ros_req*/) {
    // Trigger request body is empty
}

flatbuffers::Offset<robot_msg_fbs::CommWrapper> FlatbufferUtils::encode_trigger_resp(
    flatbuffers::FlatBufferBuilder& builder,
    uint64_t req_id,
    robot_msg_fbs::ServiceType type,
    const std_srvs::srv::Trigger::Response& ros_resp) {
    auto msg_str = builder.CreateString(ros_resp.message);
    auto resp_data = robot_msg_fbs::CreateTriggerResponse(builder, ros_resp.success, msg_str);
    // type 参数决定是 Start 还是 Stop 的回复
    auto srv_data = robot_msg_fbs::CreateServiceData(builder, req_id, type, 0, 0, 0, resp_data);
    return robot_msg_fbs::CreateCommWrapper(builder, robot_msg_fbs::AnyMsg_ServiceData, srv_data.Union());
}

void FlatbufferUtils::decode_trigger_resp(
    const robot_msg_fbs::ServiceData* fb_msg,
    std_srvs::srv::Trigger::Response& ros_resp) {
    if (fb_msg->trigger_resp()) {
        ros_resp.success = fb_msg->trigger_resp()->success();
        if (fb_msg->trigger_resp()->message()) {
            ros_resp.message = fb_msg->trigger_resp()->message()->str();
        }
    }
}

}  // namespace galaxea_robot_tele