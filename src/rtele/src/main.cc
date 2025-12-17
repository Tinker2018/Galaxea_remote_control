#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <atomic>
#include <map>
#include <mutex>
#include <future>
#include <iomanip> // 用于格式化输出

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "hdas_msg/msg/motor_control.hpp"
#include "system_manager_msg/srv/teleop_frame.hpp"

// Project Headers
#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "src/config/rtc_config.h"
#include "src/rtc/rtc_engine.h"

// 引用 galaxea_robot_tele 中的工具
#include "flatbuffer_utils.hpp"
#include "robot_msg_generated.h"

// OpenCV
#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "spdlog/spdlog.h"

// ================== Flags 定义 ==================
ABSL_FLAG(std::string, role, "pc", "Role: pc or robot");
ABSL_FLAG(std::string, room_id, "test_room", "Room ID");
ABSL_FLAG(std::string, user_id, "user_default", "User ID");
ABSL_FLAG(std::string, app_id, "", "App ID");
ABSL_FLAG(std::string, app_key, "", "App Key");
ABSL_FLAG(std::string, token, "", "Token");
ABSL_FLAG(int, video_width, 1280, "Video Width");
ABSL_FLAG(int, video_height, 720, "Video Height");
ABSL_FLAG(std::string, remote_user_id, "", "Remote User ID (for video sub)");

using namespace galaxea_robot_tele;

// ================== 图像转换辅助函数 ==================

// PC端使用: I420 (YUV) -> BGR
cv::Mat ConvertI420ToBGR(const std::vector<uint8_t>& i420_data, int width, int height) {
    if (i420_data.size() < static_cast<size_t>(width * height * 1.5)) return cv::Mat();
    cv::Mat yuv_mat(height + height / 2, width, CV_8UC1, const_cast<uint8_t*>(i420_data.data()));
    cv::Mat bgr_mat;
    cv::cvtColor(yuv_mat, bgr_mat, cv::COLOR_YUV2BGR_I420);
    return bgr_mat;
}

// Robot端使用: BGR -> I420 (YUV)
std::vector<uint8_t> ConvertBGRToI420(const cv::Mat& bgr_mat) {
    if (bgr_mat.empty()) return {};
    
    // 确保宽高是偶数（I420要求）
    if (bgr_mat.cols % 2 != 0 || bgr_mat.rows % 2 != 0) {
        spdlog::error("Image dims must be even for I420. Current: {}x{}", bgr_mat.cols, bgr_mat.rows);
        return {};
    }

    cv::Mat yuv_mat;
    cv::cvtColor(bgr_mat, yuv_mat, cv::COLOR_BGR2YUV_I420);
    std::vector<uint8_t> i420_data(yuv_mat.total() * yuv_mat.elemSize());
    memcpy(i420_data.data(), yuv_mat.data, i420_data.size());
    return i420_data;
}

// ================== 核心节点类 ==================

class RTeleNode : public rclcpp::Node {
public:
    RTeleNode(rtele::rtc::RTCEngine* rtc_engine, const std::string& role, const std::string& remote_uid, int target_w, int target_h)
        : Node("rtele_node"), rtc_engine_(rtc_engine), role_(role), remote_uid_(remote_uid), 
          target_width_(target_w), target_height_(target_h), req_id_counter_(0) {
        
        // 1. 回调组
        cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        // 2. 注册 RTC 消息接收
                auto msg_consumer = std::make_shared<rtele::rtc::MessageConsumer>(
                    [this](std::shared_ptr<rtele::rtc::MessageData> msg) {
                        // 注意：MessageData::Type 定义在 rtc_engine.h 的 MessageData 结构体里
                        if (msg->type == rtele::rtc::MessageData::Type::BINARY) { 
                            this->OnRTCBinaryMessage(msg->GetData(), msg->GetSize());
                        }
                    }); // <--- 删掉了 nullptr
        rtc_engine_->RegisterMessageConsumer(msg_consumer);

        // 3. 统计定时器 (1秒执行一次)
        stats_timer_ = this->create_wall_timer(std::chrono::seconds(1), [this](){ PrintStats(); });

        // 4. 根据角色初始化
        if (role_ == "robot") InitRobot();
        else InitPC();
    }

private:
    rtele::rtc::RTCEngine* rtc_engine_;
    std::string role_;
    std::string remote_uid_;
    int target_width_;
    int target_height_;
    rclcpp::CallbackGroup::SharedPtr cb_group_;
    rclcpp::TimerBase::SharedPtr stats_timer_;
    
    // --- 统计计数器 (Atomic) ---
    std::atomic<size_t> tx_bytes_{0};      // 发送字节数
    std::atomic<size_t> rx_bytes_{0};      // 接收字节数
    std::atomic<size_t> tx_packets_{0};    // 发送包数
    std::atomic<size_t> rx_packets_{0};    // 接收包数
    std::atomic<size_t> tx_video_frames_{0}; // 视频发送帧数
    std::atomic<size_t> rx_video_frames_{0}; // 视频接收帧数

    // --- PC端特有 ---
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_; 
    
    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_left_, pub_joint_right_, pub_grip_left_, pub_grip_right_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_left_, pub_pose_right_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_twist_chassis_, pub_twist_torso_;
    rclcpp::Publisher<hdas_msg::msg::MotorControl>::SharedPtr pub_ctrl_left_, pub_ctrl_right_;

    // Subscribers
    std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;

    // Services
    rclcpp::Service<system_manager_msg::srv::TeleopFrame>::SharedPtr srv_teleop_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_start_, srv_stop_;

    rclcpp::Client<system_manager_msg::srv::TeleopFrame>::SharedPtr cli_teleop_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_start_, cli_stop_;

    // Async Request Map
    std::mutex srv_mutex_;
    std::atomic<uint64_t> req_id_counter_;
    std::map<uint64_t, std::shared_ptr<std::promise<system_manager_msg::srv::TeleopFrame::Response>>> pending_teleop_;
    std::map<uint64_t, std::shared_ptr<std::promise<std_srvs::srv::Trigger::Response>>> pending_trigger_; 

    // ================== 统计打印 ==================
    void PrintStats() {
        // 原子地读取并重置为0
        size_t tx_b = tx_bytes_.exchange(0);
        size_t rx_b = rx_bytes_.exchange(0);
        size_t tx_p = tx_packets_.exchange(0);
        size_t rx_p = rx_packets_.exchange(0);
        size_t tx_v = tx_video_frames_.exchange(0);
        size_t rx_v = rx_video_frames_.exchange(0);

        // 如果完全没有活动，不刷屏
        if (tx_p == 0 && rx_p == 0 && tx_v == 0 && rx_v == 0) return;

        // 格式化流量单位
        auto format_bytes = [](size_t bytes) -> std::string {
            if (bytes < 1024) return std::to_string(bytes) + " B";
            else if (bytes < 1024 * 1024) return std::to_string(bytes / 1024) + " KB";
            else return std::to_string(bytes / (1024 * 1024)) + " MB";
        };

        RCLCPP_INFO(this->get_logger(), 
            "--- [Stats 1s] Role: %s ---\n"
            "  Data TX: %zu pkts (%s/s)\n"
            "  Data RX: %zu pkts (%s/s)\n"
            "  Video: TX %zu fps | RX %zu fps",
            role_.c_str(), 
            tx_p, format_bytes(tx_b).c_str(), 
            rx_p, format_bytes(rx_b).c_str(),
            tx_v, rx_v);
    }

    // ================== 发送辅助 ==================
    void SendFlatbuffer(flatbuffers::FlatBufferBuilder& builder, flatbuffers::Offset<robot_msg_fbs::CommWrapper> wrapper) {
        robot_msg_fbs::FinishCommWrapperBuffer(builder, wrapper);
        std::vector<uint8_t> data(builder.GetBufferPointer(), builder.GetBufferPointer() + builder.GetSize());
        
        // --- 统计 TX ---
        tx_bytes_ += data.size();
        tx_packets_++;

        rtc_engine_->SendRoomBinaryMessage(data);
    }

    // ================== Robot 初始化 ==================
    void InitRobot() {
        RCLCPP_INFO(this->get_logger(), "Role: ROBOT. Image Target: %dx%d", target_width_, target_height_);
        
        // 1. 视频推流
        subs_.push_back(this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "/hdas/camera_head/left_raw/image_raw_color/compressed", rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::CompressedImage::SharedPtr msg){
                try {
                    cv::Mat bgr_raw = cv::imdecode(msg->data, cv::IMREAD_COLOR);
                    if (bgr_raw.empty()) return;

                    cv::Mat bgr_resized;
                    if (bgr_raw.cols != target_width_ || bgr_raw.rows != target_height_) {
                        cv::resize(bgr_raw, bgr_resized, cv::Size(target_width_, target_height_));
                    } else {
                        bgr_resized = bgr_raw;
                    }

                    auto i420_data = ConvertBGRToI420(bgr_resized);
                    if (!i420_data.empty()) {
                        rtc_engine_->PushVideoFrame(i420_data);
                        // --- 统计 Video TX ---
                        tx_video_frames_++;
                    }
                } catch(const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Video encoding error: %s", e.what());
                }
            }));

        // 2. 订阅反馈 (Robot -> PC)
        auto sub_opt = rclcpp::SubscriptionOptions(); sub_opt.callback_group = cb_group_;
        
        auto sub_js = [&](const std::string& topic, robot_msg_fbs::RobotMsgType type) {
            subs_.push_back(this->create_subscription<sensor_msgs::msg::JointState>(topic, 10,
                [this, type](const sensor_msgs::msg::JointState::SharedPtr m){
                    flatbuffers::FlatBufferBuilder fbb(1024);
                    SendFlatbuffer(fbb, FlatbufferUtils::encode_joint_state(fbb, type, *m));
                }, sub_opt));
        };
        auto sub_pose = [&](const std::string& topic, robot_msg_fbs::RobotMsgType type, bool ignore_local) {
             auto opt = sub_opt; if(ignore_local) opt.ignore_local_publications = true;
             subs_.push_back(this->create_subscription<geometry_msgs::msg::PoseStamped>(topic, 10,
                [this, type](const geometry_msgs::msg::PoseStamped::SharedPtr m){
                    flatbuffers::FlatBufferBuilder fbb(1024);
                    SendFlatbuffer(fbb, FlatbufferUtils::encode_pose_stamped(fbb, type, *m));
                }, opt));
        };

        sub_js("/hdas/feedback_arm_left", robot_msg_fbs::RobotMsgType_FEEDBACK_ARM_LEFT);
        sub_js("/hdas/feedback_arm_right", robot_msg_fbs::RobotMsgType_FEEDBACK_ARM_RIGHT);
        sub_js("/hdas/feedback_gripper_left", robot_msg_fbs::RobotMsgType_FEEDBACK_GRIPPER_LEFT);
        sub_js("/hdas/feedback_gripper_right", robot_msg_fbs::RobotMsgType_FEEDBACK_GRIPPER_RIGHT);
        
        sub_pose("/motion_control/pose_ee_arm_left", robot_msg_fbs::RobotMsgType_POSE_EE_LEFT_ARM, false);
        sub_pose("/motion_control/pose_ee_arm_right", robot_msg_fbs::RobotMsgType_POSE_EE_RIGHT_ARM, false);
        sub_pose("/motion_target/target_pose_arm_left", robot_msg_fbs::RobotMsgType_TARGET_POSE_ARM_LEFT, true);
        sub_pose("/motion_target/target_pose_arm_right", robot_msg_fbs::RobotMsgType_TARGET_POSE_ARM_RIGHT, true);

        // 3. 发布控制 (PC -> Robot)
        pub_joint_left_  = this->create_publisher<sensor_msgs::msg::JointState>("/motion_target/target_joint_state_arm_left", 10);
        pub_joint_right_ = this->create_publisher<sensor_msgs::msg::JointState>("/motion_target/target_joint_state_arm_right", 10);
        pub_grip_left_   = this->create_publisher<sensor_msgs::msg::JointState>("/motion_target/target_position_gripper_left", 10);
        pub_grip_right_  = this->create_publisher<sensor_msgs::msg::JointState>("/motion_target/target_position_gripper_right", 10);
        pub_pose_left_   = this->create_publisher<geometry_msgs::msg::PoseStamped>("/motion_target/target_pose_arm_left", 10);
        pub_pose_right_  = this->create_publisher<geometry_msgs::msg::PoseStamped>("/motion_target/target_pose_arm_right", 10);
        pub_twist_chassis_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/motion_target/target_speed_chassis", 10);
        pub_twist_torso_   = this->create_publisher<geometry_msgs::msg::TwistStamped>("/motion_target/target_speed_torso", 10);
        pub_ctrl_left_   = this->create_publisher<hdas_msg::msg::MotorControl>("/motion_control/control_arm_left", 10);
        pub_ctrl_right_  = this->create_publisher<hdas_msg::msg::MotorControl>("/motion_control/control_arm_right", 10);

        // 4. Clients
        cli_teleop_ = this->create_client<system_manager_msg::srv::TeleopFrame>("/system_manager/teleop/service", rmw_qos_profile_services_default, cb_group_);
        cli_start_  = this->create_client<std_srvs::srv::Trigger>("/start_data_collection", rmw_qos_profile_services_default, cb_group_);
        cli_stop_   = this->create_client<std_srvs::srv::Trigger>("/stop_data_collection", rmw_qos_profile_services_default, cb_group_);
    }

    // ================== PC 初始化 ==================
    void InitPC() {
        RCLCPP_INFO(this->get_logger(), "Role: PC. Video Sub: %s", remote_uid_.c_str());

        // 1. 视频接收
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/rtele/remote_video", 10);
        if (!remote_uid_.empty()) {
            rtc_engine_->RegisterVideoConsumer(remote_uid_, std::make_shared<rtele::rtc::VideoFrameConsumer>(remote_uid_, 
                [this](std::shared_ptr<rtele::rtc::VideoFrameData> f){
                    // --- 统计 Video RX ---
                    rx_video_frames_++;
                    
                    auto bgr = ConvertI420ToBGR(f->data, f->width, f->height);
                    if(!bgr.empty()) {
                        std_msgs::msg::Header h; h.stamp=this->now(); h.frame_id="remote_video";
                        try{ image_pub_->publish(*cv_bridge::CvImage(h,"bgr8",bgr).toImageMsg()); }catch(...){}
                    }
                }));
        }

        // 2. 订阅控制 (PC -> Robot)
        auto sub_opt = rclcpp::SubscriptionOptions(); sub_opt.ignore_local_publications = true;
        
        auto sub_js = [&](const std::string& topic, robot_msg_fbs::RobotMsgType type) {
            subs_.push_back(this->create_subscription<sensor_msgs::msg::JointState>(topic, 10,
                [this, type](const sensor_msgs::msg::JointState::SharedPtr m){
                    flatbuffers::FlatBufferBuilder fbb(1024);
                    SendFlatbuffer(fbb, FlatbufferUtils::encode_joint_state(fbb, type, *m));
                }, sub_opt));
        };
        
        sub_js("/motion_target/target_joint_state_arm_left", robot_msg_fbs::RobotMsgType_TARGET_JOINT_STATE_ARM_LEFT);
        sub_js("/motion_target/target_joint_state_arm_right", robot_msg_fbs::RobotMsgType_TARGET_JOINT_STATE_ARM_RIGHT);
        sub_js("/motion_target/target_position_gripper_left", robot_msg_fbs::RobotMsgType_TARGET_POSITION_GRIPPER_LEFT);
        sub_js("/motion_target/target_position_gripper_right", robot_msg_fbs::RobotMsgType_TARGET_POSITION_GRIPPER_RIGHT);
        
        subs_.push_back(this->create_subscription<geometry_msgs::msg::PoseStamped>("/motion_target/target_pose_arm_left", 10,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr m){
                flatbuffers::FlatBufferBuilder fbb(1024);
                SendFlatbuffer(fbb, FlatbufferUtils::encode_pose_stamped(fbb, robot_msg_fbs::RobotMsgType_TARGET_POSE_ARM_LEFT, *m));
            }, sub_opt));
            
        subs_.push_back(this->create_subscription<geometry_msgs::msg::PoseStamped>("/motion_target/target_pose_arm_right", 10,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr m){
                flatbuffers::FlatBufferBuilder fbb(1024);
                SendFlatbuffer(fbb, FlatbufferUtils::encode_pose_stamped(fbb, robot_msg_fbs::RobotMsgType_TARGET_POSE_ARM_RIGHT, *m));
            }, sub_opt));

        auto sub_twist = [&](const std::string& topic, robot_msg_fbs::RobotMsgType type) {
             subs_.push_back(this->create_subscription<geometry_msgs::msg::TwistStamped>(topic, 10,
                [this, type](const geometry_msgs::msg::TwistStamped::SharedPtr m){
                    flatbuffers::FlatBufferBuilder fbb(1024);
                    SendFlatbuffer(fbb, FlatbufferUtils::encode_twist_stamped(fbb, type, *m));
                }));
        };
        sub_twist("/motion_target/target_speed_chassis", robot_msg_fbs::RobotMsgType_TARGET_SPEED_CHASSIS);
        sub_twist("/motion_target/target_speed_torso", robot_msg_fbs::RobotMsgType_TARGET_SPEED_TORSO);

        auto sub_mc = [&](const std::string& topic, robot_msg_fbs::RobotMsgType type) {
             subs_.push_back(this->create_subscription<hdas_msg::msg::MotorControl>(topic, 10,
                [this, type](const hdas_msg::msg::MotorControl::SharedPtr m){
                    flatbuffers::FlatBufferBuilder fbb(1024);
                    SendFlatbuffer(fbb, FlatbufferUtils::encode_motor_control(fbb, type, *m));
                }));
        };
        sub_mc("/motion_control/control_arm_left", robot_msg_fbs::RobotMsgType_CONTROL_ARM_LEFT);
        sub_mc("/motion_control/control_arm_right", robot_msg_fbs::RobotMsgType_CONTROL_ARM_RIGHT);

        // 3. 发布反馈 (Robot -> PC)
        pub_joint_left_  = this->create_publisher<sensor_msgs::msg::JointState>("/hdas/feedback_arm_left", 10);
        pub_joint_right_ = this->create_publisher<sensor_msgs::msg::JointState>("/hdas/feedback_arm_right", 10);
        pub_grip_left_   = this->create_publisher<sensor_msgs::msg::JointState>("/hdas/feedback_gripper_left", 10);
        pub_grip_right_  = this->create_publisher<sensor_msgs::msg::JointState>("/hdas/feedback_gripper_right", 10);
        pub_pose_left_   = this->create_publisher<geometry_msgs::msg::PoseStamped>("/motion_control/pose_ee_arm_left", 10);
        pub_pose_right_  = this->create_publisher<geometry_msgs::msg::PoseStamped>("/motion_control/pose_ee_arm_right", 10);

        // 4. Servers
        srv_teleop_ = this->create_service<system_manager_msg::srv::TeleopFrame>(
            "/system_manager/teleop/service",
            [this](const std::shared_ptr<system_manager_msg::srv::TeleopFrame::Request> req,
                   std::shared_ptr<system_manager_msg::srv::TeleopFrame::Response> resp) {
                uint64_t id = ++req_id_counter_;
                auto prom = std::make_shared<std::promise<system_manager_msg::srv::TeleopFrame::Response>>();
                auto fut = prom->get_future();
                { std::lock_guard<std::mutex> lk(srv_mutex_); pending_teleop_[id] = prom; }
                flatbuffers::FlatBufferBuilder fbb(1024);
                SendFlatbuffer(fbb, FlatbufferUtils::encode_teleop_req(fbb, id, *req));
                if(fut.wait_for(std::chrono::seconds(2)) == std::future_status::ready) *resp = fut.get();
                else { 
                    std::lock_guard<std::mutex> lk(srv_mutex_); pending_teleop_.erase(id); 
                    resp->success = false; resp->message = "Timeout"; 
                }
            }, rmw_qos_profile_services_default, cb_group_);

        auto trigger_handler = [this](uint64_t id, robot_msg_fbs::ServiceType type, std_srvs::srv::Trigger::Response& resp) {
            auto prom = std::make_shared<std::promise<std_srvs::srv::Trigger::Response>>();
            auto fut = prom->get_future();
            { std::lock_guard<std::mutex> lk(srv_mutex_); pending_trigger_[id] = prom; }
            flatbuffers::FlatBufferBuilder fbb(1024);
            SendFlatbuffer(fbb, FlatbufferUtils::encode_trigger_req(fbb, id, type));
            if(fut.wait_for(std::chrono::seconds(5)) == std::future_status::ready) resp = fut.get();
            else { 
                std::lock_guard<std::mutex> lk(srv_mutex_); pending_trigger_.erase(id); 
                resp.success = false; resp.message = "Timeout"; 
            }
        };

        srv_start_ = this->create_service<std_srvs::srv::Trigger>("/start_data_collection",
            [this, trigger_handler](const std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr resp){
                trigger_handler(++req_id_counter_, robot_msg_fbs::ServiceType_REQ_START_DATA, *resp);
            }, rmw_qos_profile_services_default, cb_group_);

        srv_stop_ = this->create_service<std_srvs::srv::Trigger>("/stop_data_collection",
            [this, trigger_handler](const std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr resp){
                trigger_handler(++req_id_counter_, robot_msg_fbs::ServiceType_REQ_STOP_DATA, *resp);
            }, rmw_qos_profile_services_default, cb_group_);
    }

    // ================== RTC 接收分发 ==================
    void OnRTCBinaryMessage(const uint8_t* data, size_t size) {
        // --- 统计 RX ---
        rx_bytes_ += size;
        rx_packets_++;

        auto wrapper = robot_msg_fbs::GetCommWrapper(data);
        if (!wrapper) return;

        // --- JointState ---
        if (auto js = wrapper->msg_as_JointState()) {
            sensor_msgs::msg::JointState m; FlatbufferUtils::decode_joint_state(js, m);
            auto type = js->msg_type();
            if (role_ == "pc") { 
                if(type == robot_msg_fbs::RobotMsgType_FEEDBACK_ARM_LEFT && pub_joint_left_) pub_joint_left_->publish(m);
                else if(type == robot_msg_fbs::RobotMsgType_FEEDBACK_ARM_RIGHT && pub_joint_right_) pub_joint_right_->publish(m);
                else if(type == robot_msg_fbs::RobotMsgType_FEEDBACK_GRIPPER_LEFT && pub_grip_left_) pub_grip_left_->publish(m);
                else if(type == robot_msg_fbs::RobotMsgType_FEEDBACK_GRIPPER_RIGHT && pub_grip_right_) pub_grip_right_->publish(m);
            } else { 
                if(type == robot_msg_fbs::RobotMsgType_TARGET_JOINT_STATE_ARM_LEFT && pub_joint_left_) pub_joint_left_->publish(m);
                else if(type == robot_msg_fbs::RobotMsgType_TARGET_JOINT_STATE_ARM_RIGHT && pub_joint_right_) pub_joint_right_->publish(m);
                else if(type == robot_msg_fbs::RobotMsgType_TARGET_POSITION_GRIPPER_LEFT && pub_grip_left_) pub_grip_left_->publish(m);
                else if(type == robot_msg_fbs::RobotMsgType_TARGET_POSITION_GRIPPER_RIGHT && pub_grip_right_) pub_grip_right_->publish(m);
            }
        }
        // --- PoseStamped ---
        else if (auto ps = wrapper->msg_as_PoseStamped()) {
            geometry_msgs::msg::PoseStamped m; FlatbufferUtils::decode_pose_stamped(ps, m);
            auto type = ps->msg_type();
            if (role_ == "pc") {
                if(type == robot_msg_fbs::RobotMsgType_POSE_EE_LEFT_ARM && pub_pose_left_) pub_pose_left_->publish(m);
                else if(type == robot_msg_fbs::RobotMsgType_POSE_EE_RIGHT_ARM && pub_pose_right_) pub_pose_right_->publish(m);
            } else {
                if(type == robot_msg_fbs::RobotMsgType_TARGET_POSE_ARM_LEFT && pub_pose_left_) pub_pose_left_->publish(m);
                else if(type == robot_msg_fbs::RobotMsgType_TARGET_POSE_ARM_RIGHT && pub_pose_right_) pub_pose_right_->publish(m);
            }
        }
        // --- TwistStamped ---
        else if (auto ts = wrapper->msg_as_TwistStamped()) {
            if (role_ == "robot") {
                geometry_msgs::msg::TwistStamped m; FlatbufferUtils::decode_twist_stamped(ts, m);
                if(ts->msg_type() == robot_msg_fbs::RobotMsgType_TARGET_SPEED_CHASSIS && pub_twist_chassis_) pub_twist_chassis_->publish(m);
                else if(ts->msg_type() == robot_msg_fbs::RobotMsgType_TARGET_SPEED_TORSO && pub_twist_torso_) pub_twist_torso_->publish(m);
            }
        }
        // --- MotorControl ---
        else if (auto mc = wrapper->msg_as_MotorControl()) {
            if (role_ == "robot") {
                hdas_msg::msg::MotorControl m; FlatbufferUtils::decode_motor_control(mc, m);
                if(mc->msg_type() == robot_msg_fbs::RobotMsgType_CONTROL_ARM_LEFT && pub_ctrl_left_) pub_ctrl_left_->publish(m);
                else if(mc->msg_type() == robot_msg_fbs::RobotMsgType_CONTROL_ARM_RIGHT && pub_ctrl_right_) pub_ctrl_right_->publish(m);
            }
        }
        // --- ServiceData ---
        else if (auto srv = wrapper->msg_as_ServiceData()) {
            HandleService(srv);
        }
    }

    void HandleService(const robot_msg_fbs::ServiceData* srv) {
        uint64_t id = srv->req_id();
        auto type = srv->type();

        if (role_ == "robot") {
            // Robot作为客户端逻辑
            auto send_resp = [this, id](flatbuffers::FlatBufferBuilder& fbb, auto wrapper) { SendFlatbuffer(fbb, wrapper); };
            
            if (type == robot_msg_fbs::ServiceType_REQ_TELEOP_FRAME && cli_teleop_->service_is_ready()) {
                auto req = std::make_shared<system_manager_msg::srv::TeleopFrame::Request>();
                FlatbufferUtils::decode_teleop_req(srv, *req);
                cli_teleop_->async_send_request(req, 
                    [send_resp, id](rclcpp::Client<system_manager_msg::srv::TeleopFrame>::SharedFuture fut){
                        flatbuffers::FlatBufferBuilder fbb(1024);
                        send_resp(fbb, FlatbufferUtils::encode_teleop_resp(fbb, id, *fut.get()));
                    });
            }
            else if (type == robot_msg_fbs::ServiceType_REQ_START_DATA && cli_start_->service_is_ready()) {
                auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
                cli_start_->async_send_request(req, 
                    [send_resp, id](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture fut){
                        flatbuffers::FlatBufferBuilder fbb(1024);
                        send_resp(fbb, FlatbufferUtils::encode_trigger_resp(fbb, id, robot_msg_fbs::ServiceType_RESP_START_DATA, *fut.get()));
                    });
            }
            else if (type == robot_msg_fbs::ServiceType_REQ_STOP_DATA && cli_stop_->service_is_ready()) {
                auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
                cli_stop_->async_send_request(req, 
                    [send_resp, id](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture fut){
                        flatbuffers::FlatBufferBuilder fbb(1024);
                        send_resp(fbb, FlatbufferUtils::encode_trigger_resp(fbb, id, robot_msg_fbs::ServiceType_RESP_STOP_DATA, *fut.get()));
                    });
            }
        } 
        else if (role_ == "pc") {
            // PC作为服务端逻辑
            std::lock_guard<std::mutex> lk(srv_mutex_);
            if (type == robot_msg_fbs::ServiceType_RESP_TELEOP_FRAME && pending_teleop_.count(id)) {
                system_manager_msg::srv::TeleopFrame::Response r; FlatbufferUtils::decode_teleop_resp(srv, r);
                pending_teleop_[id]->set_value(r); pending_teleop_.erase(id);
            }
            else if ((type == robot_msg_fbs::ServiceType_RESP_START_DATA || type == robot_msg_fbs::ServiceType_RESP_STOP_DATA) && pending_trigger_.count(id)) {
                std_srvs::srv::Trigger::Response r; FlatbufferUtils::decode_trigger_resp(srv, r);
                pending_trigger_[id]->set_value(r); pending_trigger_.erase(id);
            }
        }
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    absl::ParseCommandLine(argc, argv);
    
    rtele::config::RTCConfig config;
    config.room_id = absl::GetFlag(FLAGS_room_id);
    config.user_id = absl::GetFlag(FLAGS_user_id);
    config.app_id = absl::GetFlag(FLAGS_app_id);
    config.app_key = absl::GetFlag(FLAGS_app_key);
    config.token = absl::GetFlag(FLAGS_token);
    config.video_width = absl::GetFlag(FLAGS_video_width);
    config.video_height = absl::GetFlag(FLAGS_video_height);
    config.param = config.BuildParamJSON();
    
    std::string role = absl::GetFlag(FLAGS_role);
    if(role == "robot") config.enable_video = true;

    auto rtc = std::make_unique<rtele::rtc::RTCEngine>(config);
    if(!rtc->Initialize()) return -1;
    
    auto node = std::make_shared<RTeleNode>(rtc.get(), role, absl::GetFlag(FLAGS_remote_user_id), config.video_width, config.video_height);
    
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    
    if(rtc->JoinRoom()) {
        if(role == "robot") rtc->StartPublish();
        exec.spin();
    }
    
    rtc->Destroy();
    rclcpp::shutdown();
    return 0;
}