#include "src/rtc/rtc_engine.h"
#include <chrono>
#include <thread>
#include "absl/strings/str_format.h"
#include "src/util/token_generator.h"
#include "src/util/video_file_reader.h"
#include "rtc/bytertc_video_frame.h"
#include "spdlog/spdlog.h"

namespace rtele {
namespace rtc {

// ==================== VideoFrameConsumer 实现 ====================

VideoFrameConsumer::VideoFrameConsumer(const std::string& user_id, VideoFrameCallback callback)
    : user_id_(user_id), callback_(std::move(callback)) {
    spdlog::info("VideoFrameConsumer created for user: {}", user_id_);
}

bool VideoFrameConsumer::onFrame(bytertc::IVideoFrame* video_frame) {
    if (!video_frame) {
        spdlog::warn("[VideoConsumer] onFrame called with null video_frame");
        return false;
    }
    
    if (!callback_) {
        // spdlog::warn("[VideoConsumer] No callback registered");
        return true;
    }
    
    // 每30帧打印一次日志
    static std::atomic<int> frame_count{0};
    int count = frame_count.fetch_add(1);
    // if (count % 30 == 0) {
    //     spdlog::info("[VideoConsumer:{}] Frame #{}: {}x{}, format: {}",
    //                 user_id_, count, video_frame->width(), video_frame->height(), 
    //                 static_cast<int>(video_frame->pixelFormat()));
    // }
    
    // 准备帧数据
    auto frame_data = std::make_shared<VideoFrameData>();
    frame_data->width = video_frame->width();
    frame_data->height = video_frame->height();
    frame_data->timestamp_us = video_frame->timestampUs();
    frame_data->user_id = user_id_;
    
    // 转换为 I420 格式
    auto format = video_frame->pixelFormat();
    if (format == bytertc::kVideoPixelFormatI420) {
        int width = frame_data->width;
        int height = frame_data->height;
        int y_size = width * height;
        int uv_size = y_size / 4;
        frame_data->data.resize(y_size + uv_size * 2);
        
        uint8_t* y_plane = video_frame->planeData(0);
        uint8_t* u_plane = video_frame->planeData(1);
        uint8_t* v_plane = video_frame->planeData(2);
        int y_stride = video_frame->planeStride(0);
        int u_stride = video_frame->planeStride(1);
        int v_stride = video_frame->planeStride(2);
        
        if (y_plane && u_plane && v_plane) {
            // 复制Y平面（考虑stride）
            for (int row = 0; row < height; row++) {
                memcpy(frame_data->data.data() + row * width, 
                      y_plane + row * y_stride, width);
            }
            // 复制U平面
            int uv_width = width / 2;
            int uv_height = height / 2;
            for (int row = 0; row < uv_height; row++) {
                memcpy(frame_data->data.data() + y_size + row * uv_width,
                      u_plane + row * u_stride, uv_width);
            }
            // 复制V平面
            for (int row = 0; row < uv_height; row++) {
                memcpy(frame_data->data.data() + y_size + uv_size + row * uv_width,
                      v_plane + row * v_stride, uv_width);
            }
        }
    } else {
        spdlog::warn("[VideoConsumer] Unsupported pixel format: {}", static_cast<int>(format));
        return true;
    }
    
    // 调用回调函数
    callback_(frame_data);
    
    return true;
}

// ==================== MessageConsumer 实现 ====================

MessageConsumer::MessageConsumer(MessageCallback binary_callback, MessageCallback text_callback)
    : binary_callback_(std::move(binary_callback)), text_callback_(std::move(text_callback)) {
    spdlog::info("MessageConsumer created");
}

void MessageConsumer::OnTextMessage(const std::string& user_id, const char* message, size_t length) {
    if (!text_callback_) {
        spdlog::warn("[MessageConsumer] No callback registered");
        return;
    }
    
    auto msg_data = std::make_shared<MessageData>();
    msg_data->user_id = user_id;
    msg_data->type = MessageType::TEXT;
    // 零拷贝：使用shared_ptr管理数据
    msg_data->data = std::make_shared<std::vector<uint8_t>>(
        reinterpret_cast<const uint8_t*>(message),
        reinterpret_cast<const uint8_t*>(message) + length
    );
    msg_data->timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    
    text_callback_(msg_data);
}

void MessageConsumer::OnBinaryMessage(const std::string& user_id, const uint8_t* data, size_t length) {
    if (!binary_callback_) {
        spdlog::warn("[MessageConsumer] No callback registered");
        return;
    }
    
    auto msg_data = std::make_shared<MessageData>();
    msg_data->user_id = user_id;
    msg_data->type = MessageType::BINARY;
    // 零拷贝：使用shared_ptr管理数据
    msg_data->data = std::make_shared<std::vector<uint8_t>>(data, data + length);
    msg_data->timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    
    binary_callback_(msg_data);
}

// ==================== RTCEngine 实现 ====================

RTCEngine::RTCEngine(const config::RTCConfig& config) 
    : config_(config) {
    spdlog::info("RTCEngine created with config:");
    config_.Print();
}

RTCEngine::~RTCEngine() {
    Destroy();
}

bool RTCEngine::Initialize() {
    if (initialized_) {
        spdlog::warn("RTCEngine already initialized");
        return true;
    }
    
    spdlog::info("\n=== Initializing RTC Engine ===");
    
    const char* version = bytertc::IRTCEngine::getSDKVersion();
    spdlog::info("VolcEngine RTC SDK version: {}", version ? version : "unknown");
    
    bytertc::EngineConfig engine_config;
    engine_config.app_id = config_.app_id.c_str();
    engine_config.parameters = config_.param.c_str();
    
    rtc_engine_ = bytertc::IRTCEngine::createRTCEngine(engine_config, this);
    if (!rtc_engine_) {
        spdlog::error("Failed to create RTC engine");
        return false;
    }
    
    spdlog::info("✓ RTC Engine initialized, app_id: {}", config_.app_id);
    
    // 初始化设备
    if (!InitVideoDevice()) {
        spdlog::error("Failed to initialize video device");
        return false;
    }
    
    if (!InitAudioDevice()) {
        spdlog::error("Failed to initialize audio device");
        return false;
    }
    
    // 初始化音视频配置
    if (!InitVideoConfig()) {
        spdlog::error("Failed to init video config");
        return false;
    }
    
    if (!InitAudioConfig()) {
        spdlog::error("Failed to init audio config");
        return false;
    }
    
    initialized_ = true;
    return true;
}

bool RTCEngine::JoinRoom() {
    if (!initialized_) {
        spdlog::error("RTCEngine not initialized");
        return false;
    }
    
    spdlog::info("Joinning Room, Room ID: {}, User ID: {}", 
        config_.room_id, config_.user_id);
    
    rtc_room_ = rtc_engine_->createRTCRoom(config_.room_id.c_str());
    if (!rtc_room_) {
        spdlog::error("Failed to create RTC room");
        return false;
    }
    
    rtc_room_->setRTCRoomEventHandler(this);
    
    std::string token = config_.token;
    
    bytertc::UserInfo user_info;
    user_info.uid = config_.user_id.c_str();
    
    bytertc::RTCRoomConfig room_config;
    room_config.is_auto_subscribe_audio = false;
    room_config.is_auto_subscribe_video = false;
    
    int ret = rtc_room_->joinRoom(token.c_str(), user_info, true, room_config);
    if (ret != 0) {
        rtc_room_->destroy();
        rtc_room_ = nullptr;
        spdlog::error("Failed to join room, error code: {}", ret);
        return false;
    }

    running_ = true;
    spdlog::info("Joined room successfully");
    
    return true;
}

bool RTCEngine::StartPublish() {
    if (rtc_room_ == nullptr) {
        spdlog::error("Not in room, cannot publish");
        return false;
    }
    
    spdlog::info("=== Starting Publish ===");
    
    if (config_.enable_video) {
        spdlog::info("✓ Publishing video stream");
        spdlog::info("  Resolution: {}x{}", config_.video_width, config_.video_height);
        spdlog::info("  FPS: {}", config_.video_fps);
        
        // 启用外部视频源
        rtc_engine_->setVideoSourceType(bytertc::kVideoSourceTypeExternal);
        
        // 发布视频流
        rtc_room_->publishStreamVideo(true);
    }
    
    if (config_.enable_audio) {
        spdlog::info("✓ Publishing audio stream");
        rtc_room_->publishStreamAudio(true);
    }
    
    publishing_ = true;
    
    return true;
}

void RTCEngine::StopPublish() {
    if (!publishing_) {
        return;
    }
    
    spdlog::info("=== Stopping Publish ===");
    
    if (rtc_room_) {
        rtc_room_->publishStreamVideo(false);
        rtc_room_->publishStreamAudio(false);
    }
    
    publishing_ = false;
    spdlog::info("✓ Publish stopped");
}

bool RTCEngine::SubscribeRemoteStream(const bytertc::StreamInfo& stream_info) {
    absl::MutexLock lock(&subscribe_mutex_);
    
    std::string stream_id_str(stream_info.stream_id);
    std::string user_id_str(stream_info.user_id);
    
    // 检查是否已经订阅
    if (subscribed_streams_.find(stream_id_str) != subscribed_streams_.end()) {
        spdlog::info("Stream {} already subscribed, skipping", stream_id_str);
        return true;
    }
    
    spdlog::info("=== Subscribing Remote Stream ===");
    spdlog::info("  User ID: {}, Stream ID: {}", user_id_str, stream_id_str);
    
    if (!rtc_room_) {
        spdlog::error("Room not created");
        return false;
    }
    
    // 订阅视频流
    rtc_room_->subscribeStreamVideo(stream_info.stream_id, true);
    
    // 查找对应用户的VideoFrameConsumer并设置为sink
    {
        absl::MutexLock consumer_lock(&consumer_mutex_);
        auto consumer_it = video_consumers_.find(user_id_str);
        if (consumer_it != video_consumers_.end() && consumer_it->second) {
            bytertc::RemoteVideoSinkConfig video_config;
            video_config.pixel_format = bytertc::kVideoPixelFormatI420;
            rtc_engine_->setRemoteVideoSink(stream_info.stream_id, 
                                          consumer_it->second.get(), video_config);
            spdlog::info("✓ Set VideoFrameConsumer as sink for user: {}", user_id_str);
        } else {
            spdlog::warn("No VideoFrameConsumer registered for user: {}, video frames will not be received", 
                        user_id_str);
        }
    }
    
    // 保存订阅信息
    subscribed_streams_[stream_id_str] = stream_info;
    spdlog::info("✓ Subscribed to user: {}, total subscriptions: {}", 
                user_id_str, subscribed_streams_.size());
    
    return true;
}

void RTCEngine::UnsubscribeRemoteStream(const bytertc::StreamInfo& stream_info) {
    absl::MutexLock lock(&subscribe_mutex_);
    
    std::string stream_id_str(stream_info.stream_id);
    
    // 检查是否订阅了该流
    auto it = subscribed_streams_.find(stream_id_str);
    if (it == subscribed_streams_.end()) {
        spdlog::info("Stream {} not subscribed, skipping", stream_id_str);
        return;
    }
    
    spdlog::info("=== Unsubscribing Remote Stream ===");
    spdlog::info("  User ID: {}, Stream ID: {}", stream_info.user_id, stream_id_str);
    
    if (rtc_room_) {
        rtc_room_->subscribeStreamVideo(stream_info.stream_id, false);
    }
    
    subscribed_streams_.erase(it);
    spdlog::info("✓ Unsubscribed from user: {}, remaining subscriptions: {}", 
                stream_info.user_id, subscribed_streams_.size());
}

void RTCEngine::StopSubscribe() {
    absl::MutexLock lock(&subscribe_mutex_);
    
    if (subscribed_streams_.empty()) {
        return;
    }
    
    spdlog::info("=== Stopping All Subscriptions ===");
    spdlog::info("  Total subscriptions to stop: {}", subscribed_streams_.size());
    
    if (rtc_room_) {
        for (const auto& [stream_id, stream_info] : subscribed_streams_) {
            rtc_room_->subscribeStreamVideo(stream_info.stream_id, false);
            spdlog::info("  ✓ Unsubscribed from stream: {} (user: {})", 
                        stream_id, stream_info.user_id);
        }
    }
    
    subscribed_streams_.clear();
    spdlog::info("✓ All subscriptions stopped");
}

void RTCEngine::Destroy() {
    if (!initialized_) {
        return;
    }
    
    spdlog::info("=== Destroying RTC Engine ===");
    
    // 先停止订阅
    StopSubscribe();
    
    // 再停止推流
    StopPublish();
    
    if (rtc_room_) {
        spdlog::info("Leaving room...");
        rtc_room_->leaveRoom();
        spdlog::info("Destroying room...");
        // rtc_room_->destroy();
        spdlog::info("Reset room to nullptr");
        rtc_room_ = nullptr;
    }

    if (rtc_engine_) {
        spdlog::info("Destroy global rtc engine...");
        bytertc::IRTCEngine::destroyRTCEngine();
        rtc_engine_ = nullptr;
    }
    
    initialized_ = false;
    spdlog::info("✓ RTC Engine destroyed");
}

// ==================== 回调处理 ====================

void RTCEngine::onWarning(int warn) {
    spdlog::warn("[Callback] Warning: {}", warn);
}

void RTCEngine::onError(int err) {
    spdlog::error("[Callback] Error: {}", err);
}

void RTCEngine::onRoomStateChanged(const char* room_id, const char* uid, 
                                    int state, const char* extra_info) {
    spdlog::info("[Callback] Room state changed: room={}, user={}, state={}, extra info = {}",
                room_id, uid, state, extra_info);
}

void RTCEngine::onLeaveRoom(const bytertc::RtcRoomStats& stats) {
    spdlog::info("[Callback] Left room");
}

void RTCEngine::onUserJoined(const bytertc::UserInfo& user_info) {
    spdlog::info("[Callback] User joined: {}", user_info.uid);
}

void RTCEngine::onUserLeave(const char* uid, bytertc::UserOfflineReason reason) {
    spdlog::info("[Callback] User left: {}, reason={}", uid, static_cast<int>(reason));
    
    // 移除该用户的所有订阅流
    absl::MutexLock lock(&subscribe_mutex_);
    std::string user_id_str(uid);
    
    // 收集需要移除的流
    std::vector<std::string> streams_to_remove;
    for (const auto& [stream_id, stream_info] : subscribed_streams_) {
        if (stream_info.user_id == user_id_str) {
            streams_to_remove.push_back(stream_id);
        }
    }
    
    // 移除流
    for (const auto& stream_id : streams_to_remove) {
        subscribed_streams_.erase(stream_id);
        spdlog::info("  → Removed subscription for stream: {}", stream_id);
    }
    
    if (!streams_to_remove.empty()) {
        spdlog::info("  → Removed {} subscription(s) for user: {}", 
                    streams_to_remove.size(), user_id_str);
    }
}

void RTCEngine::onUserPublishStreamVideo(const char* stream_id, 
                                         const bytertc::StreamInfo& stream_info, 
                                         bool is_publish) {
    spdlog::info("[Callback] User {} video: {}, stream_id: {}",
                is_publish ? "published" : "unpublished",
                stream_info.user_id, stream_id);
    
    if (is_publish) {
        spdlog::info("  → Video published, triggering subscription check...");
        CheckAndSubscribe(stream_info);
    } else {
        spdlog::info("  → Video unpublished, may need to unsubscribe");
    }
}

void RTCEngine::onUserPublishStreamAudio(const char* stream_id,
                                         const bytertc::StreamInfo& stream_info,
                                         bool is_publish) {
    spdlog::info("[Callback] User {} audio: {}",
                is_publish ? "published" : "unpublished",
                stream_info.user_id);
}

void RTCEngine::onRoomMessageReceived(const char* uid, const char* message) {
    if (!uid || !message) {
        spdlog::warn("[Callback] onRoomMessageReceived called with null parameter");
        return;
    }
    
    size_t msg_length = strlen(message);
    spdlog::info("[Callback] Room message from {}: {} ({} bytes)", uid, message, msg_length);
    
    // 通知所有消息消费者（作为文本消息处理）
    absl::MutexLock lock(&consumer_mutex_);
    for (auto& consumer : message_consumers_) {
        if (consumer) {
            consumer->OnTextMessage(uid, message, msg_length);
        }
    }
}

bool RTCEngine::PushVideoFrame(const std::vector<uint8_t>& frame_data) {
    if (!publishing_ || !rtc_engine_) {
        return false;
    }
    
    if (frame_data.empty()) {
        return false;
    }
    
    // 构造 RTC 视频帧
    bytertc::VideoFrameData video_frame;
    video_frame.pixel_format = bytertc::kVideoPixelFormatI420;
    video_frame.width = config_.video_width;
    video_frame.height = config_.video_height;
    video_frame.rotation = bytertc::kVideoRotation0;
    video_frame.timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()
    ).count();
    
    // 设置 I420 平面数据
    int square = config_.video_width * config_.video_height;
    video_frame.number_of_planes = 3;
    video_frame.plane_data[0] = const_cast<uint8_t*>(frame_data.data());  // Y
    video_frame.plane_data[1] = const_cast<uint8_t*>(frame_data.data()) + square;  // U
    video_frame.plane_data[2] = const_cast<uint8_t*>(frame_data.data()) + square * 5 / 4;  // V
    video_frame.plane_stride[0] = config_.video_width;
    video_frame.plane_stride[1] = config_.video_width >> 1;
    video_frame.plane_stride[2] = config_.video_width >> 1;
    
    // 推送视频帧
    int ret = rtc_engine_->pushExternalVideoFrame(video_frame);
    if (ret != 0) {
        static int error_count = 0;
        if (error_count++ % 30 == 0) {  // 每30帧打印一次错误
            spdlog::error("Failed to push video frame, error: {}", ret);
        }
        return false;
    }
    
    return true;
}

bool RTCEngine::SendRoomMessage(const std::string& message) {
    if (!rtc_room_) {
        spdlog::error("Cannot send message: not in room");
        return false;
    }
    
    int64_t msg_id = rtc_room_->sendRoomMessage(message.c_str());
    if (msg_id > 0) {
        spdlog::debug("Room message sent, id: {}", msg_id);
        return true;
    } else {
        spdlog::error("Failed to send room message, error code: {}", msg_id);
        return false;
    }
}

// ==================== 订阅管理 ====================

void RTCEngine::CheckAndSubscribe(const bytertc::StreamInfo& stream_info) {
    std::string user_id = stream_info.user_id;
    spdlog::info("Checking should subscribe for user: {}, stream: {}", 
                user_id, stream_info.stream_id);
    
    if (!ShouldSubscribe(user_id)) {
        spdlog::info("  → Skip: user {} not in subscription list", user_id);
        return;
    }
    
    spdlog::info("  → Subscribing to user: {}, stream: {}", user_id, stream_info.stream_id);
    SubscribeRemoteStream(stream_info);
}

bool RTCEngine::ShouldSubscribe(const std::string& user_id) const {
    // 不订阅自己
    if (user_id == config_.user_id) {
        spdlog::info("  → Skip: not subscribing to self ({})", user_id);
        return false;
    }
    
    // 检查是否有注册的VideoConsumer
    absl::MutexLock lock(&consumer_mutex_);
    
    // 检查该用户是否在订阅列表中
    if (subscribed_user_ids_.find(user_id) == subscribed_user_ids_.end()) {
        return false;
    }

    return true;
}

// ==================== 设备初始化 ====================

bool RTCEngine::InitVideoDevice() {
    spdlog::info("Initializing video device...");
    
    if (!rtc_engine_) {
        return false;
    }
    
    auto video_device_manager = rtc_engine_->getVideoDeviceManager();
    if (!video_device_manager) {
        spdlog::error("Failed to get video device manager");
        return false;
    }
    
    auto capture_devices = video_device_manager->enumerateVideoCaptureDevices();
    if (!capture_devices) {
        spdlog::error("Failed to enumerate video capture devices");
        return false;
    }
    
    int device_count = capture_devices->getCount();
    spdlog::info("Found {} video capture device(s)", device_count);
    
    if (device_count == 0) {
        spdlog::warn("No video capture device found");
        capture_devices->release();
        return true;  // 不是致命错误，可能使用文件推流
    }
    
    // 枚举并选择设备
    std::string selected_device_id;
    std::string selected_device_name;
    int target_index = config_.video_device_index;
    
    // 如果 index 是 -1，自动选择第一个设备
    if (target_index == -1 && device_count > 0) {
        target_index = 0;
    }
    
    for (int i = 0; i < device_count; i++) {
        char device_id[bytertc::MAX_DEVICE_ID_LENGTH] = {0};
        char device_name[bytertc::MAX_DEVICE_ID_LENGTH] = {0};
        
        int ret = capture_devices->getDevice(i, device_name, device_id);
        if (ret != 0) {
            spdlog::warn("Failed to get device info for index {}", i);
            continue;
        }
        
        spdlog::info("  [{}] {} ({})", i, device_name, device_id);
        
        if (i == target_index) {
            selected_device_id = device_id;
            selected_device_name = device_name;
        }
    }
    
    capture_devices->release();
    
    // 设置选中的设备
    if (!selected_device_id.empty()) {
        int ret = video_device_manager->setVideoCaptureDevice(selected_device_id.c_str());
        if (ret == 0) {
            spdlog::info("✓ Selected video device [{}]: {} ({})", 
                        target_index, selected_device_name, selected_device_id);
        } else {
            spdlog::error("Failed to set video capture device, error: {}", ret);
            return false;
        }
    } else if (config_.video_device_index >= 0) {
        spdlog::error("Video device index {} is invalid (total: {})", 
                     config_.video_device_index, device_count);
        return false;
    }
    
    return true;
}

bool RTCEngine::InitAudioDevice() {
    spdlog::info("Initializing audio device...");
    
    if (!rtc_engine_) {
        return false;
    }
    
    auto audio_device_manager = rtc_engine_->getAudioDeviceManager();
    if (!audio_device_manager) {
        spdlog::error("Failed to get audio device manager");
        return false;
    }
    
    auto recording_devices = audio_device_manager->enumerateAudioCaptureDevices();
    if (!recording_devices) {
        spdlog::error("Failed to enumerate audio capture devices");
        return false;
    }
    
    int device_count = recording_devices->getCount();
    spdlog::info("Found {} audio capture device(s)", device_count);
    
    if (device_count == 0) {
        spdlog::warn("No audio capture device found");
        recording_devices->release();
        return true;  // 不是致命错误
    }
    
    // 枚举并选择设备
    std::string selected_device_id;
    std::string selected_device_name;
    int target_index = config_.audio_device_index;
    
    // 如果 index 是 -1，自动选择第一个设备
    if (target_index == -1 && device_count > 0) {
        target_index = 0;
    }
    
    for (int i = 0; i < device_count; i++) {
        char device_id[bytertc::MAX_DEVICE_ID_LENGTH] = {0};
        char device_name[bytertc::MAX_DEVICE_ID_LENGTH] = {0};
        
        int ret = recording_devices->getDevice(i, device_name, device_id);
        if (ret != 0) {
            spdlog::warn("Failed to get audio device info for index {}", i);
            continue;
        }
        
        spdlog::info("  [{}] {} ({})", i, device_name, device_id);
        
        if (i == target_index) {
            selected_device_id = device_id;
            selected_device_name = device_name;
        }
    }
    
    recording_devices->release();
    
    // 设置选中的设备
    if (!selected_device_id.empty()) {
        int ret = audio_device_manager->setAudioCaptureDevice(selected_device_id.c_str());
        if (ret == 0) {
            spdlog::info("✓ Selected audio device [{}]: {} ({})", 
                        target_index, selected_device_name, selected_device_id);
        } else {
            spdlog::error("Failed to set audio capture device, error: {}", ret);
            return false;
        }
    } else if (config_.audio_device_index >= 0) {
        spdlog::error("Audio device index {} is invalid (total: {})", 
                     config_.audio_device_index, device_count);
        return false;
    }
    
    return true;
}

bool RTCEngine::InitVideoConfig() {
    spdlog::info("Initializing video config...");
    
    if (!config_.enable_video) {
        spdlog::info("Video is disabled");
        return true;
    }
    
    // 设置视频编码器配置（编码格式通过 param JSON 指定，不在这里设置）
    bytertc::VideoEncoderConfig encoder_config;
    encoder_config.width = config_.video_width;
    encoder_config.height = config_.video_height;
    encoder_config.frame_rate = config_.video_fps;
    encoder_config.max_bitrate = config_.video_max_bitrate;
    encoder_config.min_bitrate = config_.video_min_bitrate;
    
    // 设置编码器
    int ret = rtc_engine_->setVideoEncoderConfig(encoder_config);
    if (ret != 0) {
        spdlog::error("Failed to set video encoder config, error: {}", ret);
        return false;
    }
    
    spdlog::info("✓ Video encoder: {}x{} @ {}fps, bitrate: {}-{} kbps",
                config_.video_width, config_.video_height, 
                config_.video_fps, config_.video_min_bitrate, config_.video_max_bitrate);
    
    // 如果有视频文件，使用外部视频源
    if (!config_.video_file_path.empty()) {
        spdlog::info("Using external video source from file: {}", config_.video_file_path);
        rtc_engine_->setVideoSourceType(bytertc::kVideoSourceTypeExternal);
    } else {
        spdlog::info("Using internal video capture (camera)");
        
        // 配置摄像头采集参数
        bytertc::VideoCaptureConfig capture_config;
        capture_config.capture_preference = bytertc::VideoCaptureConfig::kManual;
        capture_config.width = config_.video_width;
        capture_config.height = config_.video_height;
        capture_config.frame_rate = config_.video_fps;
        
        ret = rtc_engine_->setVideoCaptureConfig(capture_config);
        if (ret != 0) {
            spdlog::error("Failed to set video capture config, error: {}", ret);
            return false;
        }
        spdlog::info("✓ Video capture: {}x{} @ {}fps", 
                    config_.video_width, config_.video_height, config_.video_fps);
    }
    
    return true;
}

bool RTCEngine::InitAudioConfig() {
    spdlog::info("Initializing audio config...");
    
    if (!config_.enable_audio) {
        spdlog::info("Audio is disabled");
        return true;
    }
    
    // 注：当前 SDK 版本的音频配置比较简单
    // 采样率、声道数等参数会自动适配
    // 如需要更细致的控制，可以通过 param JSON 设置
    
    spdlog::info("✓ Audio config: {} Hz, {} channel(s), {} kbps (auto-adapted)",
                config_.audio_sample_rate,
                config_.audio_channels,
                config_.audio_bitrate);
    
    return true;
}

// ==================== Consumer 管理 ====================

void RTCEngine::RegisterVideoConsumer(const std::string& user_id, std::shared_ptr<VideoFrameConsumer> consumer) {
    if (!consumer) {
        spdlog::error("Cannot register null VideoFrameConsumer");
        return;
    }
    
    absl::MutexLock lock(&consumer_mutex_);
    video_consumers_[user_id] = consumer;
    // !!! 修复点：这里原来漏了一行，导致用户没加进白名单 !!!
    subscribed_user_ids_.insert(user_id);
    
    spdlog::info("Registered VideoFrameConsumer for user: {} (total registered: {})", 
                user_id, subscribed_user_ids_.size());
}

void RTCEngine::RegisterMessageConsumer(std::shared_ptr<MessageConsumer> consumer) {
    if (!consumer) {
        spdlog::error("Cannot register null MessageConsumer");
        return;
    }
    
    absl::MutexLock lock(&consumer_mutex_);
    message_consumers_.push_back(consumer);
    spdlog::info("Registered MessageConsumer (total: {})", message_consumers_.size());
}

}  // namespace rtc
}  // namespace rtele