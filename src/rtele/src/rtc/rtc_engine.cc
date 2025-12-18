#include "src/rtc/rtc_engine.h"
#include "spdlog/spdlog.h"
#include <chrono>
#include <cstring>

namespace rtele {
namespace rtc {

// ==================== VideoFrameConsumer 实现 ====================

VideoFrameConsumer::VideoFrameConsumer(const std::string& user_id, VideoFrameCallback callback)
    : user_id_(user_id), callback_(std::move(callback)) {}

bool VideoFrameConsumer::onFrame(bytertc::IVideoFrame* video_frame) {
    // [调试] 打印接收到的帧信息 (确认是否收到，以及格式是什么)
    // 只有前几帧或每隔100帧打印一次，防止刷屏
    static int frame_count = 0;
    if (frame_count++ % 100 == 0) {
        spdlog::info("Rx Frame: {}x{}, Fmt={}, TS={}", 
            video_frame->width(), video_frame->height(), 
            (int)video_frame->pixelFormat(), video_frame->timestampUs());
    }

    if (!video_frame || !callback_) return false;

    auto frame_data = std::make_shared<VideoFrameData>();
    frame_data->width = video_frame->width();
    frame_data->height = video_frame->height();
    frame_data->timestamp_us = video_frame->timestampUs();
    frame_data->user_id = user_id_;

    // 严查 I420 格式
    if (video_frame->pixelFormat() == bytertc::kVideoPixelFormatI420) {
        int y_size = frame_data->width * frame_data->height;
        int uv_size = y_size / 4;
        frame_data->data.resize(y_size + uv_size * 2);

#ifdef VOLC_RTC_ARM64
        uint8_t* y_src = video_frame->getPlaneData(0);
        uint8_t* u_src = video_frame->getPlaneData(1);
        uint8_t* v_src = video_frame->getPlaneData(2);
        int y_stride = video_frame->getPlaneStride(0);
        int u_stride = video_frame->getPlaneStride(1);
        int v_stride = video_frame->getPlaneStride(2);
#else
        uint8_t* y_src = video_frame->planeData(0);
        uint8_t* u_src = video_frame->planeData(1);
        uint8_t* v_src = video_frame->planeData(2);
        int y_stride = video_frame->planeStride(0);
        int u_stride = video_frame->planeStride(1);
        int v_stride = video_frame->planeStride(2);
#endif

        uint8_t* dst = frame_data->data.data();
        // Y Plane
        if (y_src) {
            for (int i = 0; i < frame_data->height; ++i) {
                memcpy(dst + i * frame_data->width, y_src + i * y_stride, frame_data->width);
            }
        }
        dst += y_size;
        
        // UV Planes
        int uv_w = frame_data->width / 2;
        int uv_h = frame_data->height / 2;
        if (u_src) {
            for (int i = 0; i < uv_h; ++i) {
                memcpy(dst + i * uv_w, u_src + i * u_stride, uv_w);
            }
        }
        dst += uv_size;
        if (v_src) {
            for (int i = 0; i < uv_h; ++i) {
                memcpy(dst + i * uv_w, v_src + i * v_stride, uv_w);
            }
        }
        
        callback_(frame_data);
        return true;
    } else {
        // 如果格式不对，打印警告
        spdlog::warn("Unsupported PixelFormat: {}", (int)video_frame->pixelFormat());
    }
    return false;
}

// ==================== MessageConsumer 实现 ====================

MessageConsumer::MessageConsumer(MessageCallback callback) : callback_(std::move(callback)) {}

void MessageConsumer::OnBinaryMessage(const std::string& user_id, const uint8_t* data, size_t length) {
    if (!callback_) return;
    auto msg = std::make_shared<MessageData>();
    msg->user_id = user_id;
    msg->type = MessageData::Type::BINARY;
    msg->data = std::make_shared<std::vector<uint8_t>>(data, data + length);
    msg->timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::system_clock::now().time_since_epoch()).count();
    callback_(msg);
}

// ==================== RTCEngine 实现 ====================

RTCEngine::RTCEngine(const config::RTCConfig& config) : config_(config) {}

RTCEngine::~RTCEngine() { Destroy(); }

bool RTCEngine::Initialize() {
    if (initialized_) return true;

#ifdef VOLC_RTC_ARM64
    rtc_engine_ = bytertc::createRTCVideo(config_.app_id.c_str(), this, nullptr);
#else
    bytertc::EngineConfig engine_config;
    engine_config.app_id = config_.app_id.c_str();
    rtc_engine_ = bytertc::IRTCEngine::createRTCEngine(engine_config, this);
#endif

    if (!rtc_engine_) {
        spdlog::error("Failed to create RTC Engine");
        return false;
    }

    InitVideoDevice();
    initialized_ = true;
    return true;
}

bool RTCEngine::InitVideoDevice() {
    if (!rtc_engine_) return false;

    bytertc::VideoEncoderConfig c;
    c.width = config_.video_width;
    c.height = config_.video_height;
    c.frame_rate = config_.video_fps;
    c.max_bitrate = config_.video_max_bitrate;

#ifdef VOLC_RTC_ARM64
    rtc_engine_->setVideoEncoderConfig(c);
#else
    rtc_engine_->setVideoEncoderConfig(c);
#endif

    return true;
}

bool RTCEngine::JoinRoom() {
    if (!initialized_ || !rtc_engine_) return false;

#ifdef VOLC_RTC_ARM64
    rtc_room_ = rtc_engine_->createRTCRoom(config_.room_id.c_str());
    if (!rtc_room_) return false;
    rtc_room_->setRTCRoomEventHandler(this);
    
    bytertc::UserInfo user_info;
    user_info.uid = config_.user_id.c_str();
    user_info.extra_info = nullptr;

    bytertc::RTCRoomConfig room_config;
    room_config.is_auto_publish = false;
    room_config.is_auto_subscribe_audio = true;
    room_config.is_auto_subscribe_video = true;

    int ret = rtc_room_->joinRoom(config_.token.c_str(), user_info, room_config);
#else
    rtc_room_ = rtc_engine_->createRTCRoom(config_.room_id.c_str());
    if (!rtc_room_) return false;
    rtc_room_->setRTCRoomEventHandler(this);
    
    bytertc::UserInfo user_info;
    user_info.uid = config_.user_id.c_str();
    
    bytertc::RTCRoomConfig room_config;
    room_config.is_auto_publish_audio = false; 
    room_config.is_auto_publish_video = false;
    room_config.is_auto_subscribe_audio = false; 
    room_config.is_auto_subscribe_video = false;
    room_config.stream_id = config_.user_id.c_str();

    int ret = rtc_room_->joinRoom(config_.token.c_str(), user_info, true, room_config);
#endif

    if (ret != 0) {
        spdlog::error("Join room failed: {}", ret);
        return false;
    }
    return true;
}

bool RTCEngine::StartPublish() {
    if (!rtc_room_ || !rtc_engine_) return false;

#ifdef VOLC_RTC_ARM64
    rtc_engine_->setVideoSourceType(bytertc::kStreamIndexMain, bytertc::kVideoSourceTypeExternal);
    rtc_room_->publishStream(bytertc::kMediaStreamTypeVideo);
#else
    rtc_engine_->setVideoSourceType(bytertc::kVideoSourceTypeExternal);
    rtc_room_->publishStreamVideo(true); 
#endif

    publishing_ = true;
    spdlog::info("Start Publishing Video...");
    return true;
}

bool RTCEngine::PushVideoFrame(const std::vector<uint8_t>& frame_data) {
    if (!publishing_ || !rtc_engine_) return false;
    
    int width = config_.video_width;
    int height = config_.video_height;
    size_t size = frame_data.size();

    // 只支持 I420 (YUV420P)
    size_t size_i420 = width * height * 3 / 2;

    if (size != size_i420) {
        // 数据大小不对，打个日志看看
        // spdlog::warn("PushVideoFrame: Size mismatch! Expected {} got {}", size_i420, size);
        return false;
    }

#ifdef VOLC_RTC_ARM64
    bytertc::VideoFrameBuilder builder;
    builder.frame_type = bytertc::kVideoFrameTypeRawMemory;
    builder.pixel_fmt = bytertc::kVideoPixelFormatI420; // 强制 I420
    builder.width = width;
    builder.height = height;
    builder.timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
                            std::chrono::steady_clock::now().time_since_epoch()).count();
    
    uint8_t* raw_ptr = const_cast<uint8_t*>(frame_data.data());
    int y_size = width * height;
    int uv_size = y_size / 4;

    builder.data[0] = raw_ptr;              
    builder.data[1] = raw_ptr + y_size;     
    builder.data[2] = raw_ptr + y_size + uv_size; 
    
    builder.linesize[0] = width;
    builder.linesize[1] = width / 2;
    builder.linesize[2] = width / 2;

    bytertc::IVideoFrame* frame = bytertc::buildVideoFrame(builder);
    if (frame) {
        rtc_engine_->pushExternalVideoFrame(frame);
    }
#else
    bytertc::VideoFrameData frame;
    frame.pixel_format = bytertc::kVideoPixelFormatI420;
    frame.width = width;
    frame.height = height;
    frame.timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
                            std::chrono::steady_clock::now().time_since_epoch()).count();
    
    uint8_t* raw_ptr = const_cast<uint8_t*>(frame_data.data());
    int y_size = width * height;
    int uv_size = y_size / 4;
    frame.plane_data[0] = raw_ptr;
    frame.plane_data[1] = raw_ptr + y_size;
    frame.plane_data[2] = raw_ptr + y_size + uv_size;
    frame.plane_stride[0] = width;
    frame.plane_stride[1] = width / 2;
    frame.plane_stride[2] = width / 2;
    
    rtc_engine_->pushExternalVideoFrame(frame); 
#endif

    return true;
}

bool RTCEngine::SendRoomBinaryMessage(const std::vector<uint8_t>& data) {
    if (!rtc_room_ || data.empty()) return false;
    int64_t ret = rtc_room_->sendRoomBinaryMessage(data.size(), data.data());
    return ret >= 0;
}

// ==================== 回调处理 ====================

void RTCEngine::onWarning(int warn) { spdlog::warn("RTC Warn: {}", warn); }
void RTCEngine::onError(int err) { spdlog::error("RTC Error: {}", err); }

void RTCEngine::onRoomStateChanged(const char* room_id, const char* uid, int state, const char* extra_info) {
    spdlog::info("Room State Changed: ID={} User={} State={}", room_id, uid, state);
}

#ifdef VOLC_RTC_ARM64
void RTCEngine::onUserJoined(const bytertc::UserInfo& info, int elapsed) {
    spdlog::info("User Joined (ARM): {}", info.uid);
    CheckAndSubscribe(info.uid);
}
#else
void RTCEngine::onUserJoined(const bytertc::UserInfo& info) {
    spdlog::info("User Joined (x86): {}", info.uid);
    CheckAndSubscribe(info.uid);
}
#endif

void RTCEngine::onUserLeave(const char* uid, bytertc::UserOfflineReason) {
    spdlog::info("User Left: {}", uid);
}

#ifdef VOLC_RTC_ARM64
void RTCEngine::onUserPublishStream(const char* uid, bytertc::MediaStreamType type) {
    if (type == bytertc::kMediaStreamTypeVideo || type == bytertc::kMediaStreamTypeBoth) {
        spdlog::info("User Published Video (ARM): {}", uid);
        CheckAndSubscribe(uid);
    }
}
void RTCEngine::onUserUnpublishStream(const char*, bytertc::MediaStreamType, bytertc::StreamRemoveReason) {}
#else
void RTCEngine::onUserPublishStreamVideo(const char* stream_id, const bytertc::StreamInfo& info, bool is_pub) {
    if (is_pub) {
        std::string uid = info.user_id;
        std::string sid = stream_id;
        absl::MutexLock lock(&consumer_mutex_);
        if (video_consumers_.count(uid)) {
            auto consumer = video_consumers_[uid];
            bytertc::RemoteVideoSinkConfig sink_config;
            sink_config.pixel_format = bytertc::kVideoPixelFormatI420;
            rtc_engine_->setRemoteVideoSink(sid.c_str(), consumer.get(), sink_config);
            rtc_room_->subscribeStreamVideo(sid.c_str(), true);
        }
    }
}
#endif

void RTCEngine::onRoomBinaryMessageReceived(const char* uid, int size, const uint8_t* data) {
    if (!uid || !data) return;
    absl::MutexLock lock(&consumer_mutex_);
    for (auto& consumer : message_consumers_) {
        if (consumer) consumer->OnBinaryMessage(uid, data, static_cast<size_t>(size));
    }
}

void RTCEngine::RegisterVideoConsumer(const std::string& user_id, std::shared_ptr<VideoFrameConsumer> consumer) {
    absl::MutexLock lock(&consumer_mutex_);
    video_consumers_[user_id] = consumer;
    if (rtc_engine_) {
#ifdef VOLC_RTC_ARM64
        bytertc::RemoteStreamKey key;
        key.room_id = config_.room_id.c_str();
        key.user_id = user_id.c_str();
        key.stream_index = bytertc::kStreamIndexMain;
        rtc_engine_->setRemoteVideoSink(key, consumer.get(), static_cast<bytertc::IVideoSink::PixelFormat>(bytertc::kVideoPixelFormatI420));
#else
        bytertc::RemoteVideoSinkConfig sink_config;
        sink_config.pixel_format = bytertc::kVideoPixelFormatI420;
        rtc_engine_->setRemoteVideoSink(user_id.c_str(), consumer.get(), sink_config);
#endif
    }
}

void RTCEngine::RegisterMessageConsumer(std::shared_ptr<MessageConsumer> consumer) {
    absl::MutexLock lock(&consumer_mutex_);
    message_consumers_.push_back(consumer);
}

bool RTCEngine::CheckAndSubscribe(const std::string& user_id) {
#ifdef VOLC_RTC_ARM64
    absl::MutexLock lock(&consumer_mutex_);
    if (video_consumers_.count(user_id)) {
        auto consumer = video_consumers_[user_id];
        bytertc::RemoteStreamKey key;
        key.room_id = config_.room_id.c_str();
        key.user_id = user_id.c_str();
        key.stream_index = bytertc::kStreamIndexMain;
        rtc_engine_->setRemoteVideoSink(key, consumer.get(), static_cast<bytertc::IVideoSink::PixelFormat>(bytertc::kVideoPixelFormatI420));
        rtc_room_->subscribeStream(user_id.c_str(), bytertc::kMediaStreamTypeVideo);
        return true;
    }
#endif
    return false;
}

void RTCEngine::StopPublish() {
    if (publishing_) {
#ifdef VOLC_RTC_ARM64
        if (rtc_room_) rtc_room_->unpublishStream(bytertc::kMediaStreamTypeVideo);
#else
        if (rtc_room_) rtc_room_->publishStreamVideo(false);
#endif
        publishing_ = false;
    }
}

void RTCEngine::Destroy() {
    if (rtc_room_) {
        rtc_room_->leaveRoom();
        rtc_room_->destroy();
        rtc_room_ = nullptr;
    }
    if (rtc_engine_) {
#ifdef VOLC_RTC_ARM64
        bytertc::destroyRTCVideo();
#else
        bytertc::IRTCEngine::destroyRTCEngine();
#endif
        rtc_engine_ = nullptr;
    }
    initialized_ = false;
}

} // namespace rtc
} // namespace rtele