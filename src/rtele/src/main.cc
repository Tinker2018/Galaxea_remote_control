#include <csignal>
#include <iostream>
#include <string>
#include <memory>
#include <thread>
#include <atomic>
#include <chrono>
#include <unistd.h>

// === ROS2 相关头文件 ===
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"

// === Abseil & 项目头文件 ===
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

// ================= 定义命令行参数 (Flags) =================
ABSL_FLAG(std::string, room_id, "", "Room ID for RTC session (required)");
ABSL_FLAG(std::string, user_id, "", "User ID for authentication (required)");
ABSL_FLAG(std::string, app_id, "", "Application ID (required)");
ABSL_FLAG(std::string, app_key, "", "Application Key (required)");
ABSL_FLAG(std::string, token, "", "Token (required)");
ABSL_FLAG(std::string, subscribe_user_id, "", "User ID to subscribe to");
ABSL_FLAG(bool, enable_publish_video, true, "Enable publishing video stream");
ABSL_FLAG(std::string, video_file_path, "", "Path for stream video file");

// 音视频参数默认值
ABSL_FLAG(int, video_width, 1280, "Video width");
ABSL_FLAG(int, video_height, 720, "Video height");
ABSL_FLAG(int, video_fps, 30, "Video FPS");
ABSL_FLAG(int, video_bitrate, 3000, "Video Bitrate");
ABSL_FLAG(std::string, video_codec, "H264", "Video Codec");
ABSL_FLAG(int, video_max_bitrate, 5000, "Max Bitrate");
ABSL_FLAG(int, video_min_bitrate, 1000, "Min Bitrate");
ABSL_FLAG(std::string, video_pixel_format, "I420", "Pixel Format");
ABSL_FLAG(int, audio_sample_rate, 48000, "Audio Sample Rate");
ABSL_FLAG(int, audio_channels, 2, "Audio Channels");
ABSL_FLAG(int, audio_bitrate, 128, "Audio Bitrate");
ABSL_FLAG(std::string, audio_codec, "OPUS", "Audio Codec");

// 占位Flag (您原来的代码有用到，为了防止报错保留)
ABSL_FLAG(std::string, subscribe_room_id, "", "Subscribe Room ID");
ABSL_FLAG(bool, enable_publish_audio, false, "Enable Audio Publish");
ABSL_FLAG(int, video_device_index, -1, "Video Device");
ABSL_FLAG(int, audio_device_index, -1, "Audio Device");
ABSL_FLAG(std::string, stream_config_path, "", "Stream Config Path");

// ================= 辅助函数 =================

// 将 I420 (RTC格式) 转为 BGR (OpenCV/ROS格式)
cv::Mat ConvertI420ToBGR(const std::vector<uint8_t>& i420_data, int width, int height) {
    int y_size = width * height;
    int uv_size = y_size / 4;
    
    cv::Mat yuv_mat(height + height / 2, width, CV_8UC1);
    if (i420_data.size() < static_cast<size_t>(y_size + uv_size * 2)) {
        return cv::Mat();
    }

    memcpy(yuv_mat.data, i420_data.data(), y_size);
    memcpy(yuv_mat.data + y_size, i420_data.data() + y_size, uv_size);
    memcpy(yuv_mat.data + y_size + uv_size, i420_data.data() + y_size + uv_size, uv_size);
    
    cv::Mat bgr_mat;
    cv::cvtColor(yuv_mat, bgr_mat, cv::COLOR_YUV2BGR_I420);
    return bgr_mat;
}

// 将 BGR (OpenCV格式) 转为 I420 (RTC推流格式)
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
    if (absl::GetFlag(FLAGS_room_id).empty() || absl::GetFlag(FLAGS_user_id).empty() ||
        absl::GetFlag(FLAGS_app_id).empty() || absl::GetFlag(FLAGS_app_key).empty()) {
        return false;
    }
    return true;
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
    config.video_fps = absl::GetFlag(FLAGS_video_fps);
    config.video_bitrate = absl::GetFlag(FLAGS_video_bitrate);
    config.video_codec = absl::GetFlag(FLAGS_video_codec);
    
    // 构造 param json
    config.param = config.BuildParamJSON();
    return config;
}

// ================= ROS2 节点类 =================
class RTeleNode : public rclcpp::Node {
public:
    RTeleNode() : Node("rtele_node") {
        // 创建一个话题发布者，名字叫 /rtele/remote_video
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/rtele/remote_video", 10);
        spdlog::info("ROS2 Node initialized. Topic: /rtele/remote_video");
    }

    // 当收到 RTC 视频时，调用这个函数发给 ROS
    void PublishImage(const cv::Mat& bgr_image) {
        if (bgr_image.empty()) return;

        std_msgs::msg::Header header;
        header.stamp = this->get_clock()->now();
        header.frame_id = "remote_video";

        try {
            // 使用 cv_bridge 转换并发布
            sensor_msgs::msg::Image::SharedPtr msg = 
                cv_bridge::CvImage(header, "bgr8", bgr_image).toImageMsg();
            image_pub_->publish(*msg);
        } catch (const std::exception& e) {
            spdlog::error("Failed to publish image: {}", e.what());
        }
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
};

// ================= 视频推流线程 =================
void VideoPublishThread(std::atomic<bool>* running,
                       rtele::rtc::RTCEngine* rtc_engine,
                       const rtele::config::RTCConfig& config) {
    spdlog::info("[Publish Thread] Starting...");
    
    // 默认图片路径
    std::string jpg_path = config.video_file_path;
    if (jpg_path.empty()) {
        jpg_path = "resources/20250812-131830.jpg"; 
    }
    
    cv::Mat bgr_mat = cv::imread(jpg_path);
    if (bgr_mat.empty()) {
        spdlog::error("Failed to load image: {}", jpg_path);
        return;
    }

    // 调整大小以匹配 RTC 配置
    if (bgr_mat.cols != config.video_width || bgr_mat.rows != config.video_height) {
        cv::resize(bgr_mat, bgr_mat, cv::Size(config.video_width, config.video_height));
    }

    // 转为 RTC 需要的 I420
    std::vector<uint8_t> i420_data = ConvertBGRToI420(bgr_mat);
    int frame_count = 0;

    while (*running) {
        if (rtc_engine->PushVideoFrame(i420_data)) {
            frame_count++;
            if (frame_count % 30 == 0) {
                 spdlog::debug("Sent frame #{}", frame_count);
            }
        }
        // 模拟 30 FPS
        std::this_thread::sleep_for(std::chrono::milliseconds(33));
    }
}

// ================= 主函数 =================
int main(int argc, char* argv[]) {
    // 1. 初始化 ROS2
    rclcpp::init(argc, argv);
    
    // 2. 解析命令行参数
    absl::ParseCommandLine(argc, argv);

    if (!ValidateRequiredFlags()) {
        spdlog::error("Error: Missing required flags (room_id, user_id, app_id, app_key)");
        return -1;
    }
    
    // 3. 创建 ROS 节点
    auto ros_node = std::make_shared<RTeleNode>();

    // 4. 配置并初始化 RTC
    auto config = BuildConfigFromFlags();
    auto rtc_engine = std::make_unique<rtele::rtc::RTCEngine>(config);

    // 5. 设置接收回调：RTC收到视频 -> 转给 ROS 节点发布
    auto video_callback = [ros_node](std::shared_ptr<rtele::rtc::VideoFrameData> frame_data) {
        // I420 转 BGR
        cv::Mat bgr_mat = ConvertI420ToBGR(frame_data->data, frame_data->width, frame_data->height);
        
        // 发送给 ROS
        ros_node->PublishImage(bgr_mat);
        
        static int count = 0;
        if (count++ % 30 == 0) {
            spdlog::info("Received remote frame {}x{} -> Published to ROS", 
                         frame_data->width, frame_data->height);
        }
    };

    // 注册回调 (如果指定了订阅用户)
    if (!config.subscribe_user_id.empty()) {
        auto video_consumer = std::make_shared<rtele::rtc::VideoFrameConsumer>(
            config.subscribe_user_id, video_callback
        );
        rtc_engine->RegisterVideoConsumer(config.subscribe_user_id, video_consumer);
        spdlog::info("Subscribing to user: {}", config.subscribe_user_id);
    }

    // 6. 启动 RTC 引擎
    if (!rtc_engine->Initialize()) {
        spdlog::error("RTC Initialize failed");
        return -1;
    }
    if (!rtc_engine->JoinRoom()) {
        spdlog::error("RTC JoinRoom failed");
        return -1;
    }

    // 7. 启动本地推流 (如果开启)
    std::atomic<bool> publish_running{false};
    std::unique_ptr<std::thread> publish_thread;
    
    if (absl::GetFlag(FLAGS_enable_publish_video)) {
        if (rtc_engine->StartPublish()) {
            publish_running = true;
            publish_thread = std::make_unique<std::thread>(
                VideoPublishThread, &publish_running, rtc_engine.get(), std::ref(config));
        }
    }

    // 8. 让程序一直运行 (ROS Spin)
    spdlog::info("Application started. Press Ctrl+C to stop.");
    rclcpp::spin(ros_node);

    // 9. 退出清理
    spdlog::info("Shutting down...");
    
    // 停止推流线程
    publish_running = false;
    if (publish_thread && publish_thread->joinable()) {
        publish_thread->join();
    }

    // 销毁引擎
    rtc_engine->Destroy();
    
    // 关闭 ROS
    rclcpp::shutdown();
    
    return 0;
}