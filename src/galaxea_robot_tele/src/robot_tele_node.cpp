#include "robot_tele_node.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "robot_msg_generated.h"

namespace galaxea_robot_tele {

RobotTeleNode::RobotTeleNode() 
    : Node("robot_tele_node"), 
      is_running_(true),
      stats_topic_recv_count_(0),
      stats_topic_send_count_(0),
      stats_srv_count_(0) {
    
    std::string config_path = ament_index_cpp::get_package_share_directory("galaxea_robot_tele") + "/config/udp_config.yaml";
    try {
        udp_config_ = UDPSocket::load_config(config_path);
        udp_socket_ = std::make_unique<UDPSocket>(udp_config_);
        RCLCPP_INFO(this->get_logger(), "Robot UDP initialized successfully");
    } catch (const std::exception& e) {
        RCLCPP_FATAL(this->get_logger(), "Initialization failed: %s", e.what());
        rclcpp::shutdown();
        return;
    }

    // --- 关键修改：创建多线程回调组 ---
    // 这样高频率的 JointState 数据可以并行处理，不会被某个慢的任务卡住
    cb_group_reentrant_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = cb_group_reentrant_;

    // 初始化统计定时器 (2秒一次)
    timer_ = this->create_wall_timer(
        std::chrono::seconds(2),
        std::bind(&RobotTeleNode::timer_callback, this),
        cb_group_reentrant_); // 定时器也放进组里，保证准时

    // 1. 订阅 (Robot -> PC)
    // 注意：把 sub_opt 传进去，让它们使用多线程组
    sub_feedback_arm_left_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/hdas/feedback_arm_left", 10, 
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) { this->send_joint_state(robot_msg_fbs::RobotMsgType_FEEDBACK_ARM_LEFT, *msg); }, sub_opt);
    
    sub_feedback_arm_right_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/hdas/feedback_arm_right", 10, 
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) { this->send_joint_state(robot_msg_fbs::RobotMsgType_FEEDBACK_ARM_RIGHT, *msg); }, sub_opt);
    
    sub_feedback_gripper_left_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/hdas/feedback_gripper_left", 10, 
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) { this->send_joint_state(robot_msg_fbs::RobotMsgType_FEEDBACK_GRIPPER_LEFT, *msg); }, sub_opt);
    
    sub_feedback_gripper_right_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/hdas/feedback_gripper_right", 10, 
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) { this->send_joint_state(robot_msg_fbs::RobotMsgType_FEEDBACK_GRIPPER_RIGHT, *msg); }, sub_opt);

    sub_pose_ee_arm_left_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/motion_control/pose_ee_arm_left", 10, 
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { this->send_pose_stamped(robot_msg_fbs::RobotMsgType_POSE_EE_LEFT_ARM, *msg); }, sub_opt);
    
    sub_pose_ee_arm_right_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/motion_control/pose_ee_arm_right", 10, 
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { this->send_pose_stamped(robot_msg_fbs::RobotMsgType_POSE_EE_RIGHT_ARM, *msg); }, sub_opt);

    // 对于需要忽略本地发布的话题，我们再加一个配置
    auto sub_opt_ignore = rclcpp::SubscriptionOptions();
    sub_opt_ignore.ignore_local_publications = true;
    sub_opt_ignore.callback_group = cb_group_reentrant_; // 同样加入多线程组

    sub_target_pose_arm_left_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/motion_target/target_pose_arm_left", 10, 
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { this->send_pose_stamped(robot_msg_fbs::RobotMsgType_TARGET_POSE_ARM_LEFT, *msg); }, sub_opt_ignore);
    
    sub_target_pose_arm_right_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/motion_target/target_pose_arm_right", 10, 
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { this->send_pose_stamped(robot_msg_fbs::RobotMsgType_TARGET_POSE_ARM_RIGHT, *msg); }, sub_opt_ignore);


    // 2. 发布者 (PC -> Robot)
    pub_target_joint_state_arm_left_ = this->create_publisher<sensor_msgs::msg::JointState>("/motion_target/target_joint_state_arm_left", 10);
    pub_target_joint_state_arm_right_ = this->create_publisher<sensor_msgs::msg::JointState>("/motion_target/target_joint_state_arm_right", 10);
    pub_target_position_gripper_left_ = this->create_publisher<sensor_msgs::msg::JointState>("/motion_target/target_position_gripper_left", 10);
    pub_target_position_gripper_right_ = this->create_publisher<sensor_msgs::msg::JointState>("/motion_target/target_position_gripper_right", 10);
    pub_target_pose_arm_left_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/motion_target/target_pose_arm_left", 10);
    pub_target_pose_arm_right_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/motion_target/target_pose_arm_right", 10);
    
    pub_target_speed_chassis_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/motion_target/target_speed_chassis", 10);
    pub_target_speed_torso_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/motion_target/target_speed_torso", 10);
    
    pub_control_arm_left_ = this->create_publisher<hdas_msg::msg::MotorControl>("/motion_control/control_arm_left", 10);
    pub_control_arm_right_ = this->create_publisher<hdas_msg::msg::MotorControl>("/motion_control/control_arm_right", 10);

    // 3. 客户端
    // 客户端也可以绑定到组里，不过 async_send_request 的回调默认会用 executor 里的空闲线程
    cli_teleop_ = this->create_client<system_manager_msg::srv::TeleopFrame>("/system_manager/teleop/service", rmw_qos_profile_services_default, cb_group_reentrant_);
    cli_start_data_ = this->create_client<std_srvs::srv::Trigger>("/start_data_collection", rmw_qos_profile_services_default, cb_group_reentrant_);
    cli_stop_data_ = this->create_client<std_srvs::srv::Trigger>("/stop_data_collection", rmw_qos_profile_services_default, cb_group_reentrant_);

    RCLCPP_INFO(this->get_logger(), "Robot initialized");
    recv_thread_ = std::thread(&RobotTeleNode::recv_loop, this);
}

RobotTeleNode::~RobotTeleNode() {
    is_running_ = false;
    if (recv_thread_.joinable()) {
        recv_thread_.join();
    }
}

void RobotTeleNode::timer_callback() {
    uint64_t tx = stats_topic_send_count_.exchange(0);
    uint64_t rx = stats_topic_recv_count_.exchange(0);
    uint64_t srv = stats_srv_count_.exchange(0);
    
    RCLCPP_INFO(this->get_logger(), 
        "[Stats 2s] TX Topics: %lu | RX Topics: %lu | Service Exec: %lu", 
        tx, rx, srv);
}

void RobotTeleNode::recv_loop() {
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
                    case robot_msg_fbs::RobotMsgType_TARGET_JOINT_STATE_ARM_LEFT: parse_joint_state(js, pub_target_joint_state_arm_left_); break;
                    case robot_msg_fbs::RobotMsgType_TARGET_JOINT_STATE_ARM_RIGHT: parse_joint_state(js, pub_target_joint_state_arm_right_); break;
                    case robot_msg_fbs::RobotMsgType_TARGET_POSITION_GRIPPER_LEFT: parse_joint_state(js, pub_target_position_gripper_left_); break;
                    case robot_msg_fbs::RobotMsgType_TARGET_POSITION_GRIPPER_RIGHT: parse_joint_state(js, pub_target_position_gripper_right_); break;
                    default: break;
                }
                break;
            }
            case robot_msg_fbs::AnyMsg_PoseStamped: {
                auto ps = wrapper->msg_as_PoseStamped();
                if (!ps) break;
                switch (ps->msg_type()) {
                    case robot_msg_fbs::RobotMsgType_TARGET_POSE_ARM_LEFT: parse_pose_stamped(ps, pub_target_pose_arm_left_); break;
                    case robot_msg_fbs::RobotMsgType_TARGET_POSE_ARM_RIGHT: parse_pose_stamped(ps, pub_target_pose_arm_right_); break;
                    default: break;
                }
                break;
            }
            case robot_msg_fbs::AnyMsg_TwistStamped: {
                auto ts = wrapper->msg_as_TwistStamped();
                if (!ts) break;
                switch (ts->msg_type()) {
                    case robot_msg_fbs::RobotMsgType_TARGET_SPEED_CHASSIS: parse_twist_stamped(ts, pub_target_speed_chassis_); break;
                    case robot_msg_fbs::RobotMsgType_TARGET_SPEED_TORSO: parse_twist_stamped(ts, pub_target_speed_torso_); break;
                    default: break;
                }
                break;
            }
            case robot_msg_fbs::AnyMsg_MotorControl: {
                auto mc = wrapper->msg_as_MotorControl();
                if (!mc) break;
                switch (mc->msg_type()) {
                    case robot_msg_fbs::RobotMsgType_CONTROL_ARM_LEFT: parse_motor_control(mc, pub_control_arm_left_); break;
                    case robot_msg_fbs::RobotMsgType_CONTROL_ARM_RIGHT: parse_motor_control(mc, pub_control_arm_right_); break;
                    default: break;
                }
                break;
            }

            // --- 处理服务请求 ---
            case robot_msg_fbs::AnyMsg_ServiceData: {
                auto srv = wrapper->msg_as_ServiceData();
                if (!srv) break;
                uint64_t req_id = srv->req_id();
                
                stats_srv_count_++; 

                if (srv->type() == robot_msg_fbs::ServiceType_REQ_TELEOP_FRAME) {
                    auto req = std::make_shared<system_manager_msg::srv::TeleopFrame::Request>();
                    FlatbufferUtils::decode_teleop_req(srv, *req);

                    if (cli_teleop_->service_is_ready()) {
                        cli_teleop_->async_send_request(req, 
                            [this, req_id](rclcpp::Client<system_manager_msg::srv::TeleopFrame>::SharedFuture future) {
                                auto response = future.get();
                                flatbuffers::FlatBufferBuilder builder(1024);
                                auto wrapper = FlatbufferUtils::encode_teleop_resp(builder, req_id, *response);
                                robot_msg_fbs::FinishCommWrapperBuffer(builder, wrapper);
                                udp_socket_->send(builder.GetBufferPointer(), builder.GetSize());
                            });
                    }
                }
                else if (srv->type() == robot_msg_fbs::ServiceType_REQ_START_DATA) {
                    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
                    if (cli_start_data_->service_is_ready()) {
                        cli_start_data_->async_send_request(req, 
                            [this, req_id](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
                                auto response = future.get();
                                flatbuffers::FlatBufferBuilder builder(1024);
                                auto wrapper = FlatbufferUtils::encode_trigger_resp(
                                    builder, req_id, robot_msg_fbs::ServiceType_RESP_START_DATA, *response);
                                robot_msg_fbs::FinishCommWrapperBuffer(builder, wrapper);
                                udp_socket_->send(builder.GetBufferPointer(), builder.GetSize());
                            });
                    } else {
                        RCLCPP_WARN(this->get_logger(), "Start Data service not ready!");
                    }
                }
                else if (srv->type() == robot_msg_fbs::ServiceType_REQ_STOP_DATA) {
                    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
                    if (cli_stop_data_->service_is_ready()) {
                        cli_stop_data_->async_send_request(req, 
                            [this, req_id](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
                                auto response = future.get();
                                flatbuffers::FlatBufferBuilder builder(1024);
                                auto wrapper = FlatbufferUtils::encode_trigger_resp(
                                    builder, req_id, robot_msg_fbs::ServiceType_RESP_STOP_DATA, *response);
                                robot_msg_fbs::FinishCommWrapperBuffer(builder, wrapper);
                                udp_socket_->send(builder.GetBufferPointer(), builder.GetSize());
                            });
                    } else {
                        RCLCPP_WARN(this->get_logger(), "Stop Data service not ready!");
                    }
                }
                break;
            }
            default: break;
        }
    }
}

void RobotTeleNode::parse_joint_state(const robot_msg_fbs::JointState* fb_msg, rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher) {
    stats_topic_recv_count_++;
    sensor_msgs::msg::JointState ros_msg;
    FlatbufferUtils::decode_joint_state(fb_msg, ros_msg);
    publisher->publish(ros_msg);
}

void RobotTeleNode::parse_pose_stamped(const robot_msg_fbs::PoseStamped* fb_msg, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher) {
    stats_topic_recv_count_++;
    geometry_msgs::msg::PoseStamped ros_msg;
    FlatbufferUtils::decode_pose_stamped(fb_msg, ros_msg);
    publisher->publish(ros_msg);
}

void RobotTeleNode::parse_twist_stamped(const robot_msg_fbs::TwistStamped* fb_msg, rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher) {
    stats_topic_recv_count_++;
    geometry_msgs::msg::TwistStamped ros_msg;
    FlatbufferUtils::decode_twist_stamped(fb_msg, ros_msg);
    publisher->publish(ros_msg);
}

void RobotTeleNode::parse_motor_control(const robot_msg_fbs::MotorControl* fb_msg, rclcpp::Publisher<hdas_msg::msg::MotorControl>::SharedPtr publisher) {
    stats_topic_recv_count_++;
    hdas_msg::msg::MotorControl ros_msg;
    FlatbufferUtils::decode_motor_control(fb_msg, ros_msg);
    publisher->publish(ros_msg);
}

void RobotTeleNode::send_joint_state(robot_msg_fbs::RobotMsgType msg_type, const sensor_msgs::msg::JointState& msg) {
    stats_topic_send_count_++;
    flatbuffers::FlatBufferBuilder builder(1024);
    auto wrapper = FlatbufferUtils::encode_joint_state(builder, msg_type, msg);
    robot_msg_fbs::FinishCommWrapperBuffer(builder, wrapper);
    udp_socket_->send(builder.GetBufferPointer(), builder.GetSize());
}

void RobotTeleNode::send_pose_stamped(robot_msg_fbs::RobotMsgType msg_type, const geometry_msgs::msg::PoseStamped& msg) {
    stats_topic_send_count_++;
    flatbuffers::FlatBufferBuilder builder(1024);
    auto wrapper = FlatbufferUtils::encode_pose_stamped(builder, msg_type, msg);
    robot_msg_fbs::FinishCommWrapperBuffer(builder, wrapper);
    udp_socket_->send(builder.GetBufferPointer(), builder.GetSize());
}

}  // namespace galaxea_robot_tele

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    // 关键修改：使用多线程执行器
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<galaxea_robot_tele::RobotTeleNode>();
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}