#include <chrono>
#include <thread>
#include <atomic>
#include <memory>
#include <mutex>
#include <map>
#include <future>
#include <vector>

// ROS2 核心头文件
#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_srvs/srv/trigger.hpp"

// 工具库与配置
#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "src/config/rtc_config.h"
#include "src/rtc/rtc_engine.h"

// 机器人业务逻辑 (FlatBuffers)
#include "flatbuffer_utils.hpp"
#include "robot_msg_generated.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

// OpenCV
#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "spdlog/spdlog.h"

// ================== 命令行参数定义 ==================
// 角色：pc 或 robot
ABSL_FLAG(std::string, role, "pc", "Role: pc or robot");
// 房间信息 (两边必须一致)
ABSL_FLAG(std::string, room_id, "test_room", "Room ID");
// 用户信息 (PC和Robot必须不同，比如 pc_user 和 robot_user)
ABSL_FLAG(std::string, user_id, "user_default", "User ID");
// 对方的用户ID (用于PC订阅视频)
ABSL_FLAG(std::string, remote_user_id, "", "User ID of the remote peer to subscribe");
// 火山引擎鉴权信息
ABSL_FLAG(std::string, app_id, "", "App ID");
ABSL_FLAG(std::string, token, "", "Token");
// 视频分辨率 (要跟相机匹配)
ABSL_FLAG(int, video_width, 1920, "Video Width");
ABSL_FLAG(int, video_height, 1080, "Video Height");

using namespace galaxea_robot_tele;

// ================== 图像格式转换辅助函数 ==================

// 将 RTC 收到的 I420 (YUV) 数据转为 BGR (用于 PC 显示)
cv::Mat ConvertI420ToBGR(const std::vector<uint8_t>& i420_data, int width, int height) {
    int y_size = width * height;
    int uv_size = y_size / 4;
    
    // 安全检查：防止数组越界导致崩溃
    if (i420_data.size() < static_cast<size_t>(y_size + uv_size * 2)) return cv::Mat();
    
    // I420 格式在内存中是连续的 Y... U... V...
    // 我们构造一个高度为 1.5 倍的单通道 Mat 来包裹这些数据
    cv::Mat yuv_mat(height + height / 2, width, CV_8UC1, const_cast<uint8_t*>(i420_data.data()));
    
    cv::Mat bgr_mat;
    cv::cvtColor(yuv_mat, bgr_mat, cv::COLOR_YUV2BGR_I420);
    return bgr_mat;
}

// 将 OpenCV 的 BGR 数据转为 I420 (YUV) (用于 Robot 推流)
std::vector<uint8_t> ConvertBGRToI420(const cv::Mat& bgr_mat) {
    // 转换颜色空间
    cv::Mat yuv_mat;
    cv::cvtColor(bgr_mat, yuv_mat, cv::COLOR_BGR2YUV_I420);
    
    // 将 Mat 里的数据拷贝到 std::vector 中
    // 因为 pushExternalVideoFrame 需要一块连续的内存
    std::vector<uint8_t> i420_data(yuv_mat.total() * yuv_mat.elemSize());
    memcpy(i420_data.data(), yuv_mat.data, i420_data.size());
    
    return i420_data;
}

// ================== 核心节点类 ==================

class RTCTeleNode : public rclcpp::Node {
public:
    RTCTeleNode(rtele::rtc::RTCEngine* rtc_engine, const std::string& role, const std::string& remote_uid)
        : Node("rtc_tele_node"), rtc_engine_(rtc_engine), role_(role), remote_uid_(remote_uid) {
        
        // 1. 创建多线程回调组
        // 这是一个 ROS 高级特性，允许我们在等待 Service 回复时不卡住其他消息
        cb_group_reentrant_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        
        // 2. 注册 RTC 消息回调
        // 当 SDK 收到二进制消息时，会调用这个 lambda 函数
        auto msg_consumer = std::make_shared<rtele::rtc::MessageConsumer>(
            [this](std::shared_ptr<rtele::rtc::MessageData> msg) {
                if (msg->type == rtele::rtc::MessageType::BINARY) {
                    this->OnRTCBinaryMessage(msg->GetData(), msg->GetSize());
                }
            },
            nullptr // 不需要文本消息回调
        );
        rtc_engine_->RegisterMessageConsumer(msg_consumer);

        // 3. 根据启动参数初始化不同的逻辑
        if (role_ == "robot") {
            InitRobot();
        } else {
            InitPC();
        }
    }

    // 辅助函数：把数据包通过 RTC 发送出去
    void SendToRTC(const uint8_t* data, size_t size) {
        if (!rtc_engine_ || !data || size == 0) return;
        std::vector<uint8_t> vec_data(data, data + size);
        rtc_engine_->SendRoomBinaryMessage(vec_data);
    }

private:
    rtele::rtc::RTCEngine* rtc_engine_;
    std::string role_;
    std::string remote_uid_;
    rclcpp::CallbackGroup::SharedPtr cb_group_reentrant_;
    
    // ------------------- PC 端变量 -------------------
    rclcpp::Subscription<hdas_msg::msg::MotorControl>::SharedPtr sub_control_arm_left_;
    rclcpp::Subscription<hdas_msg::msg::MotorControl>::SharedPtr sub_control_arm_right_;
    
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_feedback_left_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_feedback_right_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_feedback_grip_left_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_feedback_grip_right_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_ee_left_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_ee_right_;
    
    // PC 作为服务端，接收 ROS Service 请求
    rclcpp::Service<system_manager_msg::srv::TeleopFrame>::SharedPtr srv_server_teleop_;
    std::mutex srv_map_mutex_;
    std::map<uint64_t, std::shared_ptr<std::promise<system_manager_msg::srv::TeleopFrame::Response>>> pending_teleop_reqs_;
    std::atomic<uint64_t> req_id_counter_{0};

    // ------------------- Robot 端变量 -------------------
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_camera_;
    
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_feedback_arm_left_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_feedback_arm_right_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_feedback_grip_left_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_feedback_grip_right_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_ee_left_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_ee_right_;
    
    rclcpp::Publisher<hdas_msg::msg::MotorControl>::SharedPtr pub_control_arm_left_robot_;
    rclcpp::Publisher<hdas_msg::msg::MotorControl>::SharedPtr pub_control_arm_right_robot_;
    
    // Robot 作为客户端，去请求本地 ROS Service
    rclcpp::Client<system_manager_msg::srv::TeleopFrame>::SharedPtr cli_teleop_;


    // =============================================================
    //                          PC 端初始化逻辑
    // =============================================================
    void InitPC() {
        RCLCPP_INFO(this->get_logger(), "=== Initializing PC Role ===");
        
        // A. 视频接收部分
        // 创建发布者，把 RTC 收到的视频转给 ROS
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/rtele/remote_video", 10);
        
        // 定义视频回调函数：收到一帧 RTC 视频 -> 转码 -> Publish
        auto video_callback = [this](std::shared_ptr<rtele::rtc::VideoFrameData> frame) {
            // 校验帧有效性
            if (frame->width == 0 || frame->height == 0) return;
            
            // 转码
            cv::Mat bgr = ConvertI420ToBGR(frame->data, frame->width, frame->height);
            if (!bgr.empty()) {
                std_msgs::msg::Header header;
                header.stamp = this->now();
                header.frame_id = "remote_video"; // 这个 frame_id 可以随便起
                try {
                    // 转为 ROS 消息并发布
                    auto msg = cv_bridge::CvImage(header, "bgr8", bgr).toImageMsg();
                    image_pub_->publish(*msg);
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Image publish error: %s", e.what());
                }
            }
        };

        // 注册视频监听器
        if (!remote_uid_.empty()) {
            rtc_engine_->RegisterVideoConsumer(remote_uid_, std::make_shared<rtele::rtc::VideoFrameConsumer>(remote_uid_, video_callback));
            RCLCPP_INFO(this->get_logger(), "Subscribed to video from user: %s", remote_uid_.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "No remote user ID provided! Video will NOT be received.");
        }

        // B. 控制指令部分 (PC -> Robot)
        // 订阅本地 ROS 的控制话题，转发给 RTC
        
        // 1. 左臂控制
        sub_control_arm_left_ = this->create_subscription<hdas_msg::msg::MotorControl>(
            "/motion_control/control_arm_left", 10,
            [this](const hdas_msg::msg::MotorControl::SharedPtr msg) {
                flatbuffers::FlatBufferBuilder builder(1024);
                auto wrapper = FlatbufferUtils::encode_motor_control(builder, robot_msg_fbs::RobotMsgType_CONTROL_ARM_LEFT, *msg);
                robot_msg_fbs::FinishCommWrapperBuffer(builder, wrapper);
                this->SendToRTC(builder.GetBufferPointer(), builder.GetSize());
            });
            
        // 2. 右臂控制
        sub_control_arm_right_ = this->create_subscription<hdas_msg::msg::MotorControl>(
            "/motion_control/control_arm_right", 10,
            [this](const hdas_msg::msg::MotorControl::SharedPtr msg) {
                flatbuffers::FlatBufferBuilder builder(1024);
                auto wrapper = FlatbufferUtils::encode_motor_control(builder, robot_msg_fbs::RobotMsgType_CONTROL_ARM_RIGHT, *msg);
                robot_msg_fbs::FinishCommWrapperBuffer(builder, wrapper);
                this->SendToRTC(builder.GetBufferPointer(), builder.GetSize());
            });

        // C. 反馈数据部分 (Robot -> PC)
        // 创建发布者，收到 RTC 数据后发给本地 ROS
        pub_feedback_left_ = this->create_publisher<sensor_msgs::msg::JointState>("/hdas/feedback_arm_left", 10);
        pub_feedback_right_ = this->create_publisher<sensor_msgs::msg::JointState>("/hdas/feedback_arm_right", 10);
        pub_feedback_grip_left_ = this->create_publisher<sensor_msgs::msg::JointState>("/hdas/feedback_gripper_left", 10);
        pub_feedback_grip_right_ = this->create_publisher<sensor_msgs::msg::JointState>("/hdas/feedback_gripper_right", 10);
        pub_pose_ee_left_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/motion_control/pose_ee_arm_left", 10);
        pub_pose_ee_right_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/motion_control/pose_ee_arm_right", 10);

        // D. Service 部分 (PC 伪装成 Server)
        srv_server_teleop_ = this->create_service<system_manager_msg::srv::TeleopFrame>(
            "/system_manager/teleop/service",
            [this](const std::shared_ptr<system_manager_msg::srv::TeleopFrame::Request> request,
                   std::shared_ptr<system_manager_msg::srv::TeleopFrame::Response> response) {
                
                // 生成一个请求ID
                uint64_t req_id = ++req_id_counter_;
                auto promise = std::make_shared<std::promise<system_manager_msg::srv::TeleopFrame::Response>>();
                auto future = promise->get_future();
                
                // 存入 map，等待 RTC 回复
                {
                    std::lock_guard<std::mutex> lock(srv_map_mutex_);
                    pending_teleop_reqs_[req_id] = promise;
                }

                // 发送请求给 Robot
                flatbuffers::FlatBufferBuilder builder(1024);
                auto wrapper = FlatbufferUtils::encode_teleop_req(builder, req_id, *request);
                robot_msg_fbs::FinishCommWrapperBuffer(builder, wrapper);
                this->SendToRTC(builder.GetBufferPointer(), builder.GetSize());

                // 阻塞等待结果 (超时2秒)
                if (future.wait_for(std::chrono::seconds(2)) == std::future_status::ready) {
                    *response = future.get();
                } else {
                    response->success = false;
                    response->message = "RTC Timeout: No response from robot";
                    std::lock_guard<std::mutex> lock(srv_map_mutex_);
                    pending_teleop_reqs_.erase(req_id);
                }
            },
            rmw_qos_profile_services_default,
            cb_group_reentrant_); // 关键：放入多线程组
    }

    // =============================================================
    //                          Robot 端初始化逻辑
    // =============================================================
    void InitRobot() {
        RCLCPP_INFO(this->get_logger(), "=== Initializing Robot Role ===");

        // A. 视频推流部分 (CompressedImage -> Decode -> Push)
        // 订阅相机的压缩图像话题
        sub_camera_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "/hdas/camera_head/left_raw/image_raw_color/compressed", 
            rclcpp::SensorDataQoS(), // 使用 Best Effort 策略，因为视频流允许丢包，不能积压
            [this](const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
                // 1. 解码 JPEG
                cv_bridge::CvImagePtr cv_ptr;
                try {
                    cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
                } catch (cv_bridge::Exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
                    return;
                }
                
                if (cv_ptr->image.empty()) return;

                // 2. 转码 BGR -> I420
                // 这一步比较耗 CPU，但在 ARM 平台上通常是瓶颈所在
                std::vector<uint8_t> i420 = ConvertBGRToI420(cv_ptr->image);

                // 3. 推流
                rtc_engine_->PushVideoFrame(i420);
            });

        // B. 反馈数据订阅 (Robot -> PC)
        auto sub_opt = rclcpp::SubscriptionOptions();
        sub_opt.callback_group = cb_group_reentrant_;
        
        // 1. 左臂关节
        sub_feedback_arm_left_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/hdas/feedback_arm_left", 10,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                flatbuffers::FlatBufferBuilder builder(1024);
                auto wrapper = FlatbufferUtils::encode_joint_state(builder, robot_msg_fbs::RobotMsgType_FEEDBACK_ARM_LEFT, *msg);
                robot_msg_fbs::FinishCommWrapperBuffer(builder, wrapper);
                this->SendToRTC(builder.GetBufferPointer(), builder.GetSize());
            }, sub_opt);
            
        // 2. 右臂关节
        sub_feedback_arm_right_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/hdas/feedback_arm_right", 10,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                flatbuffers::FlatBufferBuilder builder(1024);
                auto wrapper = FlatbufferUtils::encode_joint_state(builder, robot_msg_fbs::RobotMsgType_FEEDBACK_ARM_RIGHT, *msg);
                robot_msg_fbs::FinishCommWrapperBuffer(builder, wrapper);
                this->SendToRTC(builder.GetBufferPointer(), builder.GetSize());
            }, sub_opt);

        // ... 为了节省篇幅，Gripper 和 Pose 的订阅逻辑是一模一样的，只是换个 Topic 和 MsgType ...
        // 您可以按照上面的模式继续补全

        // C. 接收控制发布者 (RTC -> Local ROS)
        pub_control_arm_left_robot_ = this->create_publisher<hdas_msg::msg::MotorControl>("/motion_control/control_arm_left", 10);
        pub_control_arm_right_robot_ = this->create_publisher<hdas_msg::msg::MotorControl>("/motion_control/control_arm_right", 10);

        // D. Service Client (接收 PC 请求 -> 调本地 Service)
        cli_teleop_ = this->create_client<system_manager_msg::srv::TeleopFrame>(
            "/system_manager/teleop/service", rmw_qos_profile_services_default, cb_group_reentrant_);
    }

    // =============================================================
    //                   核心：RTC 消息路由器
    // =============================================================
    void OnRTCBinaryMessage(const uint8_t* data, size_t size) {
        if (!data || size == 0) return;
        
        // 解析 FlatBuffer
        auto wrapper = robot_msg_fbs::GetCommWrapper(data);
        if (!wrapper) {
            RCLCPP_WARN(this->get_logger(), "Received invalid FlatBuffer message");
            return;
        }

        // ------------------------- PC 逻辑 -------------------------
        if (role_ == "pc") {
            switch (wrapper->msg_type()) {
                // 处理关节状态反馈
                case robot_msg_fbs::AnyMsg_JointState: {
                    auto js = wrapper->msg_as_JointState();
                    if (!js) break;
                    sensor_msgs::msg::JointState ros_msg;
                    FlatbufferUtils::decode_joint_state(js, ros_msg);
                    
                    if (js->msg_type() == robot_msg_fbs::RobotMsgType_FEEDBACK_ARM_LEFT) pub_feedback_left_->publish(ros_msg);
                    else if (js->msg_type() == robot_msg_fbs::RobotMsgType_FEEDBACK_ARM_RIGHT) pub_feedback_right_->publish(ros_msg);
                    else if (js->msg_type() == robot_msg_fbs::RobotMsgType_FEEDBACK_GRIPPER_LEFT) pub_feedback_grip_left_->publish(ros_msg);
                    else if (js->msg_type() == robot_msg_fbs::RobotMsgType_FEEDBACK_GRIPPER_RIGHT) pub_feedback_grip_right_->publish(ros_msg);
                    break;
                }
                // 处理位姿反馈
                case robot_msg_fbs::AnyMsg_PoseStamped: {
                    auto ps = wrapper->msg_as_PoseStamped();
                    if (!ps) break;
                    geometry_msgs::msg::PoseStamped ros_msg;
                    FlatbufferUtils::decode_pose_stamped(ps, ros_msg);
                    if (ps->msg_type() == robot_msg_fbs::RobotMsgType_POSE_EE_LEFT_ARM) pub_pose_ee_left_->publish(ros_msg);
                    else if (ps->msg_type() == robot_msg_fbs::RobotMsgType_POSE_EE_RIGHT_ARM) pub_pose_ee_right_->publish(ros_msg);
                    break;
                }
                // 处理服务响应
                case robot_msg_fbs::AnyMsg_ServiceData: {
                    auto srv = wrapper->msg_as_ServiceData();
                    if (srv && srv->type() == robot_msg_fbs::ServiceType_RESP_TELEOP_FRAME) {
                        uint64_t id = srv->req_id();
                        std::lock_guard<std::mutex> lock(srv_map_mutex_);
                        // 找到对应的 Promise，设置返回值
                        if (pending_teleop_reqs_.count(id)) {
                            system_manager_msg::srv::TeleopFrame::Response resp;
                            FlatbufferUtils::decode_teleop_resp(srv, resp);
                            pending_teleop_reqs_[id]->set_value(resp);
                            pending_teleop_reqs_.erase(id);
                        }
                    }
                    break;
                }
                default: break;
            }
        } 
        // ------------------------- Robot 逻辑 -------------------------
        else if (role_ == "robot") {
            switch (wrapper->msg_type()) {
                // 处理电机控制指令
                case robot_msg_fbs::AnyMsg_MotorControl: {
                    auto mc = wrapper->msg_as_MotorControl();
                    if (!mc) break;
                    hdas_msg::msg::MotorControl ros_msg;
                    FlatbufferUtils::decode_motor_control(mc, ros_msg);
                    if (mc->msg_type() == robot_msg_fbs::RobotMsgType_CONTROL_ARM_LEFT) pub_control_arm_left_robot_->publish(ros_msg);
                    else if (mc->msg_type() == robot_msg_fbs::RobotMsgType_CONTROL_ARM_RIGHT) pub_control_arm_right_robot_->publish(ros_msg);
                    break;
                }
                // 处理服务请求
                case robot_msg_fbs::AnyMsg_ServiceData: {
                    auto srv = wrapper->msg_as_ServiceData();
                    if (srv && srv->type() == robot_msg_fbs::ServiceType_REQ_TELEOP_FRAME) {
                        uint64_t req_id = srv->req_id();
                        auto req = std::make_shared<system_manager_msg::srv::TeleopFrame::Request>();
                        FlatbufferUtils::decode_teleop_req(srv, *req);
                        
                        if (cli_teleop_->service_is_ready()) {
                            // 异步调用本地 Service
                            cli_teleop_->async_send_request(req, 
                                [this, req_id](rclcpp::Client<system_manager_msg::srv::TeleopFrame>::SharedFuture future) {
                                    auto response = future.get();
                                    // 拿到结果后，编码并回传给 PC
                                    flatbuffers::FlatBufferBuilder builder(1024);
                                    auto wrapper = FlatbufferUtils::encode_teleop_resp(builder, req_id, *response);
                                    robot_msg_fbs::FinishCommWrapperBuffer(builder, wrapper);
                                    this->SendToRTC(builder.GetBufferPointer(), builder.GetSize());
                            });
                        }
                    }
                    break;
                }
                default: break;
            }
        }
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    absl::ParseCommandLine(argc, argv);

    // 1. 读取命令行配置
    rtele::config::RTCConfig config;
    config.room_id = absl::GetFlag(FLAGS_room_id);
    config.user_id = absl::GetFlag(FLAGS_user_id);
    config.app_id = absl::GetFlag(FLAGS_app_id);
    config.token = absl::GetFlag(FLAGS_token);
    config.video_width = absl::GetFlag(FLAGS_video_width);
    config.video_height = absl::GetFlag(FLAGS_video_height);
    
    std::string role = absl::GetFlag(FLAGS_role);
    std::string remote_uid = absl::GetFlag(FLAGS_remote_user_id);

    // 2. 自动配置推流开关
    if (role == "robot") {
        config.enable_video = true;  // 机器人一定要推流
    } else {
        config.enable_video = false; // PC 只需要拉流
    }
    
    config.param = config.BuildParamJSON();

    // 3. 初始化 RTC 引擎
    auto rtc_engine = std::make_unique<rtele::rtc::RTCEngine>(config);
    if (!rtc_engine->Initialize()) {
        RCLCPP_ERROR(rclcpp::get_logger("rtc_node"), "RTC Engine Initialization Failed!");
        return -1;
    }

    // 4. 创建 ROS 节点
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<RTCTeleNode>(rtc_engine.get(), role, remote_uid);
    executor.add_node(node);

    // 5. 加入 RTC 房间
    if (!rtc_engine->JoinRoom()) {
        RCLCPP_ERROR(rclcpp::get_logger("rtc_node"), "RTC JoinRoom Failed!");
        return -1;
    }

    // 6. 开启推流 (如果是 Robot)
    if (role == "robot") {
        rtc_engine->StartPublish();
        RCLCPP_INFO(node->get_logger(), "Robot joined room and started publishing!");
    } else {
        RCLCPP_INFO(node->get_logger(), "PC joined room and waiting for stream...");
    }

    // 7. 进入循环
    executor.spin();

    // 8. 退出清理
    rtc_engine->Destroy();
    rclcpp::shutdown();
    return 0;
}
