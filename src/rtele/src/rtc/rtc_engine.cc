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
    if (!video_frame) return false;
    if (!callback_) return true;
    
    auto frame_data = std::make_shared<VideoFrameData>();
    frame_data->width = video_frame->width();
    frame_data->height = video_frame->height();
    frame_data->timestamp_us = video_frame->timestampUs();
    frame_data->user_id = user_id_;
    
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
            for (int row = 0; row < height; row++) {
                memcpy(frame_data->data.data() + row * width, y_plane + row * y_stride, width);
            }
            int uv_width = width / 2;
            int uv_height = height / 2;
            for (int row = 0; row < uv_height; row++) {
                memcpy(frame_data->data.data() + y_size + row * uv_width, u_plane + row * u_stride, uv_width);
            }
            for (int row = 0; row < uv_height; row++) {
                memcpy(frame_data->data.data() + y_size + uv_size + row * uv_width, v_plane + row * v_stride, uv_width);
            }
        }
    }
    callback_(frame_data);
    return true;
}

// ==================== MessageConsumer 实现 ====================

MessageConsumer::MessageConsumer(MessageCallback binary_callback, MessageCallback text_callback)
    : binary_callback_(std::move(binary_callback)), text_callback_(std::move(text_callback)) {
    spdlog::info("MessageConsumer created");
}

void MessageConsumer::OnTextMessage(const std::string& user_id, const char* message, size_t length) {
    if (!text_callback_) return;
    auto msg_data = std::make_shared<MessageData>();
    msg_data->user_id = user_id;
    msg_data->type = MessageType::TEXT;
    msg_data->data = std::make_shared<std::vector<uint8_t>>(
        reinterpret_cast<const uint8_t*>(message), reinterpret_cast<const uint8_t*>(message) + length);
    msg_data->timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    text_callback_(msg_data);
}

void MessageConsumer::OnBinaryMessage(const std::string& user_id, const uint8_t* data, size_t length) {
    if (!binary_callback_) return;
    auto msg_data = std::make_shared<MessageData>();
    msg_data->user_id = user_id;
    msg_data->type = MessageType::BINARY;
    msg_data->data = std::make_shared<std::vector<uint8_t>>(data, data + length);
    msg_data->timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    binary_callback_(msg_data);
}

// ==================== RTCEngine 实现 ====================

RTCEngine::RTCEngine(const config::RTCConfig& config) : config_(config) {
    config_.Print();
}

RTCEngine::~RTCEngine() { Destroy(); }

bool RTCEngine::Initialize() {
    if (initialized_) return true;
    
    bytertc::EngineConfig engine_config;
    engine_config.app_id = config_.app_id.c_str();
    engine_config.parameters = config_.param.c_str();
    
    rtc_engine_ = bytertc::IRTCEngine::createRTCEngine(engine_config, this);
    if (!rtc_engine_) return false;
    
    InitVideoDevice();
    InitAudioDevice();
    InitVideoConfig();
    InitAudioConfig();
    
    initialized_ = true;
    return true;
}

bool RTCEngine::JoinRoom() {
    if (!initialized_) return false;
    
    rtc_room_ = rtc_engine_->createRTCRoom(config_.room_id.c_str());
    if (!rtc_room_) return false;
    
    rtc_room_->setRTCRoomEventHandler(this);
    
    bytertc::UserInfo user_info;
    user_info.uid = config_.user_id.c_str();
    
    bytertc::RTCRoomConfig room_config;
    room_config.is_auto_subscribe_audio = false;
    room_config.is_auto_subscribe_video = false;
    
    int ret = rtc_room_->joinRoom(config_.token.c_str(), user_info, true, room_config);
    if (ret != 0) {
        rtc_room_->destroy();
        rtc_room_ = nullptr;
        return false;
    }
    running_ = true;
    return true;
}

bool RTCEngine::StartPublish() {
    if (rtc_room_ == nullptr) return false;
    if (config_.enable_video) {
        rtc_engine_->setVideoSourceType(bytertc::kVideoSourceTypeExternal);
        rtc_room_->publishStreamVideo(true);
    }
    if (config_.enable_audio) rtc_room_->publishStreamAudio(true);
    publishing_ = true;
    return true;
}

void RTCEngine::StopPublish() {
    if (!publishing_) return;
    if (rtc_room_) {
        rtc_room_->publishStreamVideo(false);
        rtc_room_->publishStreamAudio(false);
    }
    publishing_ = false;
}

bool RTCEngine::SubscribeRemoteStream(const bytertc::StreamInfo& stream_info) {
    absl::MutexLock lock(&subscribe_mutex_);
    std::string stream_id_str(stream_info.stream_id);
    std::string user_id_str(stream_info.user_id);
    
    if (subscribed_streams_.find(stream_id_str) != subscribed_streams_.end()) return true;
    
    rtc_room_->subscribeStreamVideo(stream_info.stream_id, true);
    
    {
        absl::MutexLock consumer_lock(&consumer_mutex_);
        auto consumer_it = video_consumers_.find(user_id_str);
        if (consumer_it != video_consumers_.end() && consumer_it->second) {
            bytertc::RemoteVideoSinkConfig video_config;
            video_config.pixel_format = bytertc::kVideoPixelFormatI420;
            rtc_engine_->setRemoteVideoSink(stream_info.stream_id, consumer_it->second.get(), video_config);
            spdlog::info("✓ Set sink for user: {}", user_id_str);
        }
    }
    subscribed_streams_[stream_id_str] = stream_info;
    return true;
}

void RTCEngine::UnsubscribeRemoteStream(const bytertc::StreamInfo& stream_info) {
    absl::MutexLock lock(&subscribe_mutex_);
    std::string stream_id_str(stream_info.stream_id);
    auto it = subscribed_streams_.find(stream_id_str);
    if (it == subscribed_streams_.end()) return;
    
    rtc_room_->subscribeStreamVideo(stream_info.stream_id, false);
    subscribed_streams_.erase(it);
}

void RTCEngine::StopSubscribe() {
    absl::MutexLock lock(&subscribe_mutex_);
    if (rtc_room_) {
        for (const auto& [stream_id, stream_info] : subscribed_streams_) {
            rtc_room_->subscribeStreamVideo(stream_info.stream_id, false);
        }
    }
    subscribed_streams_.clear();
}

void RTCEngine::Destroy() {
    if (!initialized_) return;
    StopSubscribe();
    StopPublish();
    if (rtc_room_) {
        rtc_room_->leaveRoom();
        rtc_room_ = nullptr;
    }
    if (rtc_engine_) {
        bytertc::IRTCEngine::destroyRTCEngine();
        rtc_engine_ = nullptr;
    }
    initialized_ = false;
}

// ==================== 消息发送与接收 (修复版) ====================

// 发送文本消息
bool RTCEngine::SendRoomMessage(const std::string& message) {
    if (!rtc_room_) return false;
    return rtc_room_->sendRoomMessage(message.c_str()) > 0;
}

// !!! 发送二进制消息 (已修复参数顺序) !!!
bool RTCEngine::SendRoomBinaryMessage(const std::vector<uint8_t>& data) {
    if (!rtc_room_) {
        spdlog::error("Cannot send binary: not in room");
        return false;
    }
    if (data.empty()) return false;
    
    // 修复：参数顺序应该是 (大小, 指针)
    int64_t msg_id = rtc_room_->sendRoomBinaryMessage(static_cast<int>(data.size()), data.data());
    
    if (msg_id > 0) {
        return true;
    } else {
        spdlog::error("Failed to send binary message, error code: {}", msg_id);
        return false;
    }
}

// 接收文本消息回调
void RTCEngine::onRoomMessageReceived(const char* uid, const char* message) {
    if (!uid || !message) return;
    size_t len = strlen(message);
    absl::MutexLock lock(&consumer_mutex_);
    for (auto& consumer : message_consumers_) {
        if (consumer) consumer->OnTextMessage(uid, message, len);
    }
}

// 接收二进制消息回调
void RTCEngine::onRoomBinaryMessageReceived(const char* uid, int size, const uint8_t* data) {
    if (!uid || !data || size <= 0) return;
    
    // spdlog::info("Received binary message from {}: {} bytes", uid, size);
    
    absl::MutexLock lock(&consumer_mutex_);
    for (auto& consumer : message_consumers_) {
        if (consumer) consumer->OnBinaryMessage(uid, data, static_cast<size_t>(size));
    }
}

// ==================== 视频推流 ====================

bool RTCEngine::PushVideoFrame(const std::vector<uint8_t>& frame_data) {
    if (!publishing_ || !rtc_engine_ || frame_data.empty()) return false;
    
    bytertc::VideoFrameData video_frame;
    video_frame.pixel_format = bytertc::kVideoPixelFormatI420;
    video_frame.width = config_.video_width;
    video_frame.height = config_.video_height;
    video_frame.rotation = bytertc::kVideoRotation0;
    video_frame.timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    
    int square = config_.video_width * config_.video_height;
    video_frame.number_of_planes = 3;
    video_frame.plane_data[0] = const_cast<uint8_t*>(frame_data.data());
    video_frame.plane_data[1] = const_cast<uint8_t*>(frame_data.data()) + square;
    video_frame.plane_data[2] = const_cast<uint8_t*>(frame_data.data()) + square * 5 / 4;
    video_frame.plane_stride[0] = config_.video_width;
    video_frame.plane_stride[1] = config_.video_width >> 1;
    video_frame.plane_stride[2] = config_.video_width >> 1;
    
    return rtc_engine_->pushExternalVideoFrame(video_frame) == 0;
}

// ==================== 回调与其他逻辑 ====================

void RTCEngine::onWarning(int warn) { spdlog::warn("Warn: {}", warn); }
void RTCEngine::onError(int err) { spdlog::error("Err: {}", err); }
void RTCEngine::onRoomStateChanged(const char*, const char*, int, const char*) {}
void RTCEngine::onLeaveRoom(const bytertc::RtcRoomStats&) {}
void RTCEngine::onUserJoined(const bytertc::UserInfo& info) { spdlog::info("User joined: {}", info.uid); }
void RTCEngine::onUserLeave(const char* uid, bytertc::UserOfflineReason) {
    absl::MutexLock lock(&subscribe_mutex_);
    std::string user_id_str(uid);
    std::vector<std::string> to_remove;
    for (const auto& [sid, info] : subscribed_streams_) {
        if (info.user_id == user_id_str) to_remove.push_back(sid);
    }
    for (const auto& sid : to_remove) subscribed_streams_.erase(sid);
}

void RTCEngine::onUserPublishStreamVideo(const char*, const bytertc::StreamInfo& info, bool is_publish) {
    if (is_publish) CheckAndSubscribe(info);
}
void RTCEngine::onUserPublishStreamAudio(const char*, const bytertc::StreamInfo&, bool) {}

void RTCEngine::CheckAndSubscribe(const bytertc::StreamInfo& info) {
    if (ShouldSubscribe(info.user_id)) SubscribeRemoteStream(info);
}

bool RTCEngine::ShouldSubscribe(const std::string& user_id) const {
    if (user_id == config_.user_id) return false;
    absl::MutexLock lock(&consumer_mutex_);
    return subscribed_user_ids_.find(user_id) != subscribed_user_ids_.end();
}

bool RTCEngine::InitVideoDevice() { return true; }
bool RTCEngine::InitAudioDevice() { return true; }
bool RTCEngine::InitVideoConfig() {
    if (!config_.enable_video) return true;
    bytertc::VideoEncoderConfig c;
    c.width = config_.video_width; c.height = config_.video_height;
    c.frame_rate = config_.video_fps; c.max_bitrate = config_.video_max_bitrate; c.min_bitrate = config_.video_min_bitrate;
    rtc_engine_->setVideoEncoderConfig(c);
    
    if (config_.video_file_path.empty()) {
        bytertc::VideoCaptureConfig cc;
        cc.capture_preference = bytertc::VideoCaptureConfig::kManual;
        cc.width = config_.video_width; cc.height = config_.video_height; cc.frame_rate = config_.video_fps;
        rtc_engine_->setVideoCaptureConfig(cc);
    }
    return true;
}
bool RTCEngine::InitAudioConfig() { return true; }

void RTCEngine::RegisterVideoConsumer(const std::string& user_id, std::shared_ptr<VideoFrameConsumer> consumer) {
    if (!consumer) return;
    absl::MutexLock lock(&consumer_mutex_);
    video_consumers_[user_id] = consumer;
    subscribed_user_ids_.insert(user_id); 
    spdlog::info("Registered VideoConsumer for {}", user_id);
}

void RTCEngine::RegisterMessageConsumer(std::shared_ptr<MessageConsumer> consumer) {
    if (!consumer) return;
    absl::MutexLock lock(&consumer_mutex_);
    message_consumers_.push_back(consumer);
}

} // namespace rtc
} // namespace rtele