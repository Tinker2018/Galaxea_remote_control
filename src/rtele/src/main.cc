#include <csignal>
#include <iostream>
#include <string>
#include <memory>
#include <thread>
#include <atomic>
#include <chrono>
#include <unistd.h>

// ROS2 Headers
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp" // !!! 新增：用于传输二进制数据
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"

// Abseil & Project Headers
#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "absl/strings/str_format.h"
#include "src/config/rtc_config.h"
#include "src/rtc/rtc_engine.h"
#include "src/util/video_file_reader.h"
#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "spdlog/spdlog.h"

// Flags 保持不变...
ABSL_FLAG(std::string, room_id, "", "Room ID");
ABSL_FLAG(std::string, user_id, "", "User ID");
ABSL_FLAG(std::string, app_id, "", "App ID");
ABSL_FLAG(std::string, app_key, "", "App Key");
ABSL_FLAG(std::string, token, "", "Token");
ABSL_FLAG(std::string, subscribe_user_id, "", "Subscribe User ID");
ABSL_FLAG(bool, enable_publish_video, true, "Enable Publish");
ABSL_FLAG(std::string, video_file_path, "", "Path");
ABSL_FLAG(int, video_width, 1280, "W");
ABSL_FLAG(int, video_height, 720, "H");
ABSL_FLAG(int, video_fps, 30, "FPS");
ABSL_FLAG(int, video_bitrate, 3000, "Bitrate");
ABSL_FLAG(std::string, video_codec, "H264", "Codec");
ABSL_FLAG(int, video_max_bitrate, 5000, "Max");
ABSL_FLAG(int, video_min_bitrate, 1000, "Min");
ABSL_FLAG(std::string, video_pixel_format, "I420", "Format");
ABSL_FLAG(int, audio_sample_rate, 48000, "Rate");
ABSL_FLAG(int, audio_channels, 2, "Ch");
ABSL_FLAG(int, audio_bitrate, 128, "Audio Bitrate");
ABSL_FLAG(std::string, audio_codec, "OPUS", "Codec");
ABSL_FLAG(std::string, subscribe_room_id, "", "Sub Room");
ABSL_FLAG(bool, enable_publish_audio, false, "Enable Audio");
ABSL_FLAG(int, video_device_index, -1, "Video Dev");
ABSL_FLAG(int, audio_device_index, -1, "Audio Dev");
ABSL_FLAG(std::string, stream_config_path, "", "Config");

// 辅助函数
cv::Mat ConvertI420ToBGR(const std::vector<uint8_t>& i420_data, int width, int height) {
    int y_size = width * height;
    int uv_size = y_size / 4;
    cv::Mat yuv_mat(height + height / 2, width, CV_8UC1);
    if (i420_data.size() < static_cast<size_t>(y_size + uv_size * 2)) return cv::Mat();
    memcpy(yuv_mat.data, i420_data.data(), y_size);
    memcpy(yuv_mat.data + y_size, i420_data.data() + y_size, uv_size);
    memcpy(yuv_mat.data + y_size + uv_size, i420_data.data() + y_size + uv_size, uv_size);
    cv::Mat bgr_mat;
    cv::cvtColor(yuv_mat, bgr_mat, cv::COLOR_YUV2BGR_I420);
    return bgr_mat;
}

std::vector<uint8_t> ConvertBGRToI420(const cv::Mat& bgr_mat) {
    int width = bgr_mat.cols;
    int height = bgr_mat.rows;
    int y_size = width * height;
    int uv_size = y_size / 4;
    cv::Mat yuv_mat;
    cv::cvtColor(bgr_mat, yuv_mat, cv::COLOR_BGR2YUV_I420);
    std::vector<uint8_t> i420_data(y_size + uv_size * 2);
    memcpy(i420_data.data(), yuv_mat.data, y_size + uv_size * 2);
    return i420_data;
}

bool ValidateRequiredFlags() {
    return !(absl::GetFlag(FLAGS_room_id).empty() || absl::GetFlag(FLAGS_user_id).empty() ||
             absl::GetFlag(FLAGS_app_id).empty() || absl::GetFlag(FLAGS_app_key).empty());
}

rtele::config::RTCConfig BuildConfigFromFlags() {
    rtele::config::RTCConfig config;
    config.app_id = absl::GetFlag(FLAGS_app_id);
    config.app_key = absl::GetFlag(FLAGS_app_key);
    config.room_id = absl::GetFlag(FLAGS_room_id);
    config.user_id = absl::GetFlag(FLAGS_user_id);
    config.token = absl::GetFlag(FLAGS_token);
    config.subscribe_user_id = absl::GetFlag(FLAGS_subscribe_user_id);
    config.video_file_path = absl::GetFlag(FLAGS_video_file_path);
    config.video_width = absl::GetFlag(FLAGS_video_width);
    config.video_height = absl::GetFlag(FLAGS_video_height);
    config.param = config.BuildParamJSON();
    return config;
}

// ================= ROS2 Node =================
class RTeleNode : public rclcpp::Node {
public:
    RTeleNode(rtele::rtc::RTCEngine* rtc_engine) 
        : Node("rtele_node"), rtc_engine_(rtc_engine) {
        
        // 1. 视频发布者
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/rtele/remote_video", 10);

        // 2. 二进制数据发布者 (接收RTC数据 -> 发给ROS)
        data_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("/rtele/data_rx", 10);
        
        // 3. 二进制数据订阅者 (接收ROS数据 -> 通过RTC发走)
        data_sub_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
            "/rtele/data_tx", 10,
            [this](const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
                if (rtc_engine_) {
                    // 将ROS消息转为vector
                    std::vector<uint8_t> data = msg->data;
                    rtc_engine_->SendRoomBinaryMessage(data);
                    spdlog::info("Sent binary data size: {} bytes via RTC", data.size());
                }
            }
        );

        spdlog::info("ROS2 Data Topics Ready: [Sub] /rtele/data_tx  -> [Pub] /rtele/data_rx");
    }

    void PublishImage(const cv::Mat& bgr_image) {
        if (bgr_image.empty()) return;
        std_msgs::msg::Header header;
        header.stamp = this->get_clock()->now();
        header.frame_id = "remote_video";
        try {
            sensor_msgs::msg::Image::SharedPtr msg = 
                cv_bridge::CvImage(header, "bgr8", bgr_image).toImageMsg();
            image_pub_->publish(*msg);
        } catch (const std::exception& e) {}
    }

    // 新增：发布接收到的二进制数据
    void PublishBinaryData(const std::string& user_id, const uint8_t* data, size_t length) {
        std_msgs::msg::UInt8MultiArray msg;
        msg.data.resize(length);
        memcpy(msg.data.data(), data, length);
        data_pub_->publish(msg);
        spdlog::info("Received {} bytes from {} -> Published to ROS", length, user_id);
    }

private:
    rtele::rtc::RTCEngine* rtc_engine_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr data_pub_;
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr data_sub_;
};

void VideoPublishThread(std::atomic<bool>* running,
                       rtele::rtc::RTCEngine* rtc_engine,
                       const rtele::config::RTCConfig& config) {
    std::string jpg_path = config.video_file_path.empty() ? "resources/20250812-131830.jpg" : config.video_file_path;
    cv::Mat bgr_mat = cv::imread(jpg_path);
    if (bgr_mat.empty()) { spdlog::error("Failed to load image"); return; }
    if (bgr_mat.cols != config.video_width || bgr_mat.rows != config.video_height) 
        cv::resize(bgr_mat, bgr_mat, cv::Size(config.video_width, config.video_height));
    std::vector<uint8_t> i420_data = ConvertBGRToI420(bgr_mat);

    while (*running) {
        rtc_engine->PushVideoFrame(i420_data);
        std::this_thread::sleep_for(std::chrono::milliseconds(33));
    }
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    absl::ParseCommandLine(argc, argv);
    if (!ValidateRequiredFlags()) return -1;

    auto config = BuildConfigFromFlags();
    auto rtc_engine = std::make_unique<rtele::rtc::RTCEngine>(config);
    
    // 初始化 ROS 节点，传入 rtc_engine 指针以便发送数据
    auto ros_node = std::make_shared<RTeleNode>(rtc_engine.get());

    // 1. 注册视频回调
    auto video_callback = [ros_node](std::shared_ptr<rtele::rtc::VideoFrameData> frame_data) {
        cv::Mat bgr_mat = ConvertI420ToBGR(frame_data->data, frame_data->width, frame_data->height);
        ros_node->PublishImage(bgr_mat);
    };
    if (!config.subscribe_user_id.empty()) {
        auto video_consumer = std::make_shared<rtele::rtc::VideoFrameConsumer>(config.subscribe_user_id, video_callback);
        rtc_engine->RegisterVideoConsumer(config.subscribe_user_id, video_consumer);
    }

    // 2. 注册消息回调 (二进制数据)
    auto message_consumer = std::make_shared<rtele::rtc::MessageConsumer>(
        // Binary Callback
        [ros_node](std::shared_ptr<rtele::rtc::MessageData> msg) {
            if (msg->type == rtele::rtc::MessageType::BINARY) {
                ros_node->PublishBinaryData(msg->user_id, msg->GetData(), msg->GetSize());
            }
        },
        // Text Callback (不处理)
        nullptr
    );
    rtc_engine->RegisterMessageConsumer(message_consumer);

    if (!rtc_engine->Initialize() || !rtc_engine->JoinRoom()) return -1;

    std::atomic<bool> publish_running{false};
    std::unique_ptr<std::thread> publish_thread;
    if (absl::GetFlag(FLAGS_enable_publish_video)) {
        rtc_engine->StartPublish();
        publish_running = true;
        publish_thread = std::make_unique<std::thread>(VideoPublishThread, &publish_running, rtc_engine.get(), config);
    }

    rclcpp::spin(ros_node);

    publish_running = false;
    if (publish_thread && publish_thread->joinable()) publish_thread->join();
    rtc_engine->Destroy();
    rclcpp::shutdown();
    return 0;
}