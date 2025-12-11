#include "pc_tele_node.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "robot_msg_generated.h"

namespace galaxea_robot_tele {

PCTeleNode::PCTeleNode() 
    : Node("pc_tele_node"), 
      is_running_(true), 
      stats_topic_recv_count_(0),
      stats_topic_send_count_(0),
      stats_srv_count_(0),
      req_id_counter_(0) {
    
    // 1. 加载配置
    std::string config_path = ament_index_cpp::get_package_share_directory("galaxea_robot_tele") + "/config/udp_config.yaml";
    try {
        udp_config_ = UDPSocket::load_config(config_path);
        udp_socket_ = std::make_unique<UDPSocket>(udp_config_);
        RCLCPP_INFO(this->get_logger(), "PC UDP initialized successfully");
    } catch (const std::exception& e) {
        RCLCPP_FATAL(this->get_logger(), "Initialization failed: %s", e.what());
        rclcpp::shutdown();
        return;
    }

    // --- 关键修改：创建多线程回调组 ---
    cb_group_services_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // 2. 初始化统计定时器 (2秒一次)
    timer_ = this->create_wall_timer(
        std::chrono::seconds(2),
        std::bind(&PCTeleNode::timer_callback, this));

    // 3. 订阅 (PC -> Robot)
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.ignore_local_publications = true;

    sub_target_joint_state_arm_left_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/motion_target/target_joint_state_arm_left", 10,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) { this->send_joint_state(robot_msg_fbs::RobotMsgType_TARGET_JOINT_STATE_ARM_LEFT, *msg); });
    sub_target_joint_state_arm_right_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/motion_target/target_joint_state_arm_right", 10,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) { this->send_joint_state(robot_msg_fbs::RobotMsgType_TARGET_JOINT_STATE_ARM_RIGHT, *msg); });
    sub_target_position_gripper_left_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/motion_target/target_position_gripper_left", 10,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) { this->send_joint_state(robot_msg_fbs::RobotMsgType_TARGET_POSITION_GRIPPER_LEFT, *msg); });
    sub_target_position_gripper_right_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/motion_target/target_position_gripper_right", 10,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) { this->send_joint_state(robot_msg_fbs::RobotMsgType_TARGET_POSITION_GRIPPER_RIGHT, *msg); });
    
    sub_target_pose_arm_left_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/motion_target/target_pose_arm_left", 10,
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { this->send_pose_stamped(robot_msg_fbs::RobotMsgType_TARGET_POSE_ARM_LEFT, *msg); }, sub_opt);
    sub_target_pose_arm_right_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/motion_target/target_pose_arm_right", 10,
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { this->send_pose_stamped(robot_msg_fbs::RobotMsgType_TARGET_POSE_ARM_RIGHT, *msg); }, sub_opt);

    sub_target_speed_chassis_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "/motion_target/target_speed_chassis", 10,
        [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) { this->send_twist_stamped(robot_msg_fbs::RobotMsgType_TARGET_SPEED_CHASSIS, *msg); });
    sub_target_speed_torso_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "/motion_target/target_speed_torso", 10,
        [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) { this->send_twist_stamped(robot_msg_fbs::RobotMsgType_TARGET_SPEED_TORSO, *msg); });

    sub_control_arm_left_ = this->create_subscription<hdas_msg::msg::MotorControl>(
        "/motion_control/control_arm_left", 10,
        [this](const hdas_msg::msg::MotorControl::SharedPtr msg) { this->send_motor_control(robot_msg_fbs::RobotMsgType_CONTROL_ARM_LEFT, *msg); });
    sub_control_arm_right_ = this->create_subscription<hdas_msg::msg::MotorControl>(
        "/motion_control/control_arm_right", 10,
        [this](const hdas_msg::msg::MotorControl::SharedPtr msg) { this->send_motor_control(robot_msg_fbs::RobotMsgType_CONTROL_ARM_RIGHT, *msg); });

    // 4. 发布者 (Robot -> PC)
    pub_left_arm_joint_ = this->create_publisher<sensor_msgs::msg::JointState>("/hdas/feedback_arm_left", 10);
    pub_right_arm_joint_ = this->create_publisher<sensor_msgs::msg::JointState>("/hdas/feedback_arm_right", 10);
    pub_left_gripper_joint_ = this->create_publisher<sensor_msgs::msg::JointState>("/hdas/feedback_gripper_left", 10);
    pub_right_gripper_joint_ = this->create_publisher<sensor_msgs::msg::JointState>("/hdas/feedback_gripper_right", 10);
    pub_pose_ee_arm_left_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/motion_control/pose_ee_arm_left", 10);
    pub_pose_ee_arm_right_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/motion_control/pose_ee_arm_right", 10);
    pub_target_pose_arm_left_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/motion_target/target_pose_arm_left", 10);
    pub_target_pose_arm_right_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/motion_target/target_pose_arm_right", 10);

    // 5. 服务端 (PC 伪装成 Server) - 关键修改：绑定到多线程回调组

    // (1) Teleop Frame
    srv_server_teleop_ = this->create_service<system_manager_msg::srv::TeleopFrame>(
        "/system_manager/teleop/service",
        [this](const std::shared_ptr<system_manager_msg::srv::TeleopFrame::Request> request,
               std::shared_ptr<system_manager_msg::srv::TeleopFrame::Response> response) {
            
            stats_srv_count_++;
            uint64_t req_id = ++req_id_counter_;
            auto promise = std::make_shared<std::promise<system_manager_msg::srv::TeleopFrame::Response>>();
            auto future = promise->get_future();
            {
                std::lock_guard<std::mutex> lock(srv_map_mutex_);
                pending_teleop_reqs_[req_id] = promise;
            }
            flatbuffers::FlatBufferBuilder builder(1024);
            auto wrapper = FlatbufferUtils::encode_teleop_req(builder, req_id, *request);
            robot_msg_fbs::FinishCommWrapperBuffer(builder, wrapper);
            udp_socket_->send(builder.GetBufferPointer(), builder.GetSize());

            if (future.wait_for(std::chrono::seconds(2)) == std::future_status::ready) {
                *response = future.get();
            } else {
                RCLCPP_ERROR(this->get_logger(), "Teleop Service timeout!");
                response->success = false;
                response->message = "UDP Timeout";
                std::lock_guard<std::mutex> lock(srv_map_mutex_);
                pending_teleop_reqs_.erase(req_id);
            }
        },
        rmw_qos_profile_services_default,
        cb_group_services_); // <--- 这里加了组

    // (2) Start Data Collection
    srv_server_start_data_ = this->create_service<std_srvs::srv::Trigger>(
        "/start_data_collection",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
            
            stats_srv_count_++;
            uint64_t req_id = ++req_id_counter_;
            auto promise = std::make_shared<std::promise<std_srvs::srv::Trigger::Response>>();
            auto future = promise->get_future();
            {
                std::lock_guard<std::mutex> lock(srv_map_mutex_);
                pending_start_reqs_[req_id] = promise;
            }
            flatbuffers::FlatBufferBuilder builder(1024);
            auto wrapper = FlatbufferUtils::encode_trigger_req(builder, req_id, robot_msg_fbs::ServiceType_REQ_START_DATA);
            robot_msg_fbs::FinishCommWrapperBuffer(builder, wrapper);
            udp_socket_->send(builder.GetBufferPointer(), builder.GetSize());

            // 奶奶，这里即使等待，因为是多线程，也不会卡住 Topic 转发了
            if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
                *response = future.get();
            } else {
                RCLCPP_ERROR(this->get_logger(), "StartData Service timeout!");
                response->success = false;
                response->message = "UDP Timeout";
                std::lock_guard<std::mutex> lock(srv_map_mutex_);
                pending_start_reqs_.erase(req_id);
            }
        },
        rmw_qos_profile_services_default,
        cb_group_services_); // <--- 这里加了组

    // (3) Stop Data Collection
    srv_server_stop_data_ = this->create_service<std_srvs::srv::Trigger>(
        "/stop_data_collection",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
            
            stats_srv_count_++;
            uint64_t req_id = ++req_id_counter_;
            auto promise = std::make_shared<std::promise<std_srvs::srv::Trigger::Response>>();
            auto future = promise->get_future();
            {
                std::lock_guard<std::mutex> lock(srv_map_mutex_);
                pending_stop_reqs_[req_id] = promise;
            }
            flatbuffers::FlatBufferBuilder builder(1024);
            auto wrapper = FlatbufferUtils::encode_trigger_req(builder, req_id, robot_msg_fbs::ServiceType_REQ_STOP_DATA);
            robot_msg_fbs::FinishCommWrapperBuffer(builder, wrapper);
            udp_socket_->send(builder.GetBufferPointer(), builder.GetSize());

            if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
                *response = future.get();
            } else {
                RCLCPP_ERROR(this->get_logger(), "StopData Service timeout!");
                response->success = false;
                response->message = "UDP Timeout";
                std::lock_guard<std::mutex> lock(srv_map_mutex_);
                pending_stop_reqs_.erase(req_id);
            }
        },
        rmw_qos_profile_services_default,
        cb_group_services_); // <--- 这里加了组

    RCLCPP_INFO(this->get_logger(), "PC initialized");
    recv_thread_ = std::thread(&PCTeleNode::recv_loop, this);
}

PCTeleNode::~PCTeleNode() {
    is_running_ = false;
    if (recv_thread_.joinable()) {
        recv_thread_.join();
    }
}

void PCTeleNode::timer_callback() {
    uint64_t tx = stats_topic_send_count_.exchange(0);
    uint64_t rx = stats_topic_recv_count_.exchange(0);
    uint64_t srv = stats_srv_count_.exchange(0);
    
    RCLCPP_INFO(this->get_logger(), 
        "[Stats 2s] TX Topics: %lu | RX Topics: %lu | Service Calls: %lu", 
        tx, rx, srv);
}

void PCTeleNode::recv_loop() {
    const size_t BUF_SIZE = udp_config_.buffer_size;
    std::vector<uint8_t> buffer(BUF_SIZE);

    while (is_running_ && rclcpp::ok()) {
        ssize_t recv_len = udp_socket_->receive(buffer.data(), BUF_SIZE);
        if (recv_len <= 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        auto wrapper = robot_msg_fbs::GetCommWrapper(buffer.data());
        if (!wrapper) continue;

        switch (wrapper->msg_type()) {
            case robot_msg_fbs::AnyMsg_JointState: {
                auto js = wrapper->msg_as_JointState();
                if (!js) break;
                switch (js->msg_type()) {
                    case robot_msg_fbs::RobotMsgType_FEEDBACK_ARM_LEFT: parse_joint_state(js, pub_left_arm_joint_); break;
                    case robot_msg_fbs::RobotMsgType_FEEDBACK_ARM_RIGHT: parse_joint_state(js, pub_right_arm_joint_); break;
                    case robot_msg_fbs::RobotMsgType_FEEDBACK_GRIPPER_LEFT: parse_joint_state(js, pub_left_gripper_joint_); break;
                    case robot_msg_fbs::RobotMsgType_FEEDBACK_GRIPPER_RIGHT: parse_joint_state(js, pub_right_gripper_joint_); break;
                    default: break;
                }
                break;
            }
            case robot_msg_fbs::AnyMsg_PoseStamped: {
                auto ps = wrapper->msg_as_PoseStamped();
                if (!ps) break;
                switch (ps->msg_type()) {
                    case robot_msg_fbs::RobotMsgType_POSE_EE_LEFT_ARM: parse_pose_stamped(ps, pub_pose_ee_arm_left_); break;
                    case robot_msg_fbs::RobotMsgType_POSE_EE_RIGHT_ARM: parse_pose_stamped(ps, pub_pose_ee_arm_right_); break;
                    case robot_msg_fbs::RobotMsgType_TARGET_POSE_ARM_LEFT: parse_pose_stamped(ps, pub_target_pose_arm_left_); break;
                    case robot_msg_fbs::RobotMsgType_TARGET_POSE_ARM_RIGHT: parse_pose_stamped(ps, pub_target_pose_arm_right_); break;
                    default: break;
                }
                break;
            }
            case robot_msg_fbs::AnyMsg_ServiceData: {
                auto srv = wrapper->msg_as_ServiceData();
                if (!srv) break;
                
                uint64_t id = srv->req_id();
                std::lock_guard<std::mutex> lock(srv_map_mutex_);

                if (srv->type() == robot_msg_fbs::ServiceType_RESP_TELEOP_FRAME) {
                    if (pending_teleop_reqs_.count(id)) {
                        system_manager_msg::srv::TeleopFrame::Response resp;
                        FlatbufferUtils::decode_teleop_resp(srv, resp);
                        pending_teleop_reqs_[id]->set_value(resp);
                        pending_teleop_reqs_.erase(id);
                    }
                }
                else if (srv->type() == robot_msg_fbs::ServiceType_RESP_START_DATA) {
                    if (pending_start_reqs_.count(id)) {
                        std_srvs::srv::Trigger::Response resp;
                        FlatbufferUtils::decode_trigger_resp(srv, resp);
                        pending_start_reqs_[id]->set_value(resp);
                        pending_start_reqs_.erase(id);
                    }
                }
                else if (srv->type() == robot_msg_fbs::ServiceType_RESP_STOP_DATA) {
                    if (pending_stop_reqs_.count(id)) {
                        std_srvs::srv::Trigger::Response resp;
                        FlatbufferUtils::decode_trigger_resp(srv, resp);
                        pending_stop_reqs_[id]->set_value(resp);
                        pending_stop_reqs_.erase(id);
                    }
                }
                break;
            }
            default: break;
        }
    }
}

void PCTeleNode::parse_joint_state(const robot_msg_fbs::JointState* fb_msg, rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher) {
    stats_topic_recv_count_++;
    sensor_msgs::msg::JointState ros_msg;
    FlatbufferUtils::decode_joint_state(fb_msg, ros_msg);
    publisher->publish(ros_msg);
}

void PCTeleNode::parse_pose_stamped(const robot_msg_fbs::PoseStamped* fb_msg, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher) {
    stats_topic_recv_count_++;
    geometry_msgs::msg::PoseStamped ros_msg;
    FlatbufferUtils::decode_pose_stamped(fb_msg, ros_msg);
    publisher->publish(ros_msg);
}

void PCTeleNode::send_joint_state(robot_msg_fbs::RobotMsgType msg_type, const sensor_msgs::msg::JointState& msg) {
    stats_topic_send_count_++;
    flatbuffers::FlatBufferBuilder builder(1024);
    auto wrapper = FlatbufferUtils::encode_joint_state(builder, msg_type, msg);
    robot_msg_fbs::FinishCommWrapperBuffer(builder, wrapper);
    udp_socket_->send(builder.GetBufferPointer(), builder.GetSize());
}

void PCTeleNode::send_pose_stamped(robot_msg_fbs::RobotMsgType msg_type, const geometry_msgs::msg::PoseStamped& msg) {
    stats_topic_send_count_++;
    flatbuffers::FlatBufferBuilder builder(1024);
    auto wrapper = FlatbufferUtils::encode_pose_stamped(builder, msg_type, msg);
    robot_msg_fbs::FinishCommWrapperBuffer(builder, wrapper);
    udp_socket_->send(builder.GetBufferPointer(), builder.GetSize());
}

void PCTeleNode::send_twist_stamped(robot_msg_fbs::RobotMsgType msg_type, const geometry_msgs::msg::TwistStamped& msg) {
    stats_topic_send_count_++;
    flatbuffers::FlatBufferBuilder builder(1024);
    auto wrapper = FlatbufferUtils::encode_twist_stamped(builder, msg_type, msg);
    robot_msg_fbs::FinishCommWrapperBuffer(builder, wrapper);
    udp_socket_->send(builder.GetBufferPointer(), builder.GetSize());
}

void PCTeleNode::send_motor_control(robot_msg_fbs::RobotMsgType msg_type, const hdas_msg::msg::MotorControl& msg) {
    stats_topic_send_count_++;
    flatbuffers::FlatBufferBuilder builder(1024);
    auto wrapper = FlatbufferUtils::encode_motor_control(builder, msg_type, msg);
    robot_msg_fbs::FinishCommWrapperBuffer(builder, wrapper);
    udp_socket_->send(builder.GetBufferPointer(), builder.GetSize());
}

}  // namespace galaxea_robot_tele

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    // 关键修改：使用多线程执行器，防止服务等待时阻塞其他 Topic 转发
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<galaxea_robot_tele::PCTeleNode>();
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}