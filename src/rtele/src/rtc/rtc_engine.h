#ifndef RTELE_RTC_RTC_ENGINE_H_
#define RTELE_RTC_RTC_ENGINE_H_

#include <atomic>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <vector>
#include "src/config/rtc_config.h"
#include "absl/synchronization/mutex.h"

// VolcEngine RTC SDK
#include "bytertc_engine.h"
#include "bytertc_room.h"

namespace rtele {

// 前向声明
namespace util {
class VideoFileReader;
}

namespace rtc {

// ==================== 视频帧相关 ====================

// 视频帧数据结构
struct VideoFrameData {
    std::vector<uint8_t> data;  // I420 格式的原始数据
    int width;
    int height;
    int64_t timestamp_us;
    std::string user_id;  // 来源用户ID
    std::string stream_id; // 流ID
};

// 视频帧回调函数类型
using VideoFrameCallback = std::function<void(std::shared_ptr<VideoFrameData>)>;

// 视频帧消费者，实现IVideoSink接口
class VideoFrameConsumer : public bytertc::IVideoSink {
public:
    explicit VideoFrameConsumer(const std::string& user_id, VideoFrameCallback callback);
    ~VideoFrameConsumer() override = default;
    
    const std::string& GetUserId() const { return user_id_; }
    
    // IVideoSink 接口
    bool onFrame(bytertc::IVideoFrame* video_frame) override;
    int getRenderElapse() override { return 16; }
    void release() override {}

private:
    std::string user_id_;
    VideoFrameCallback callback_;
};

// ==================== 消息相关 ====================

// 消息类型枚举
enum class MessageType {
    TEXT,    // 文本消息
    BINARY   // 二进制消息
};

// 消息数据结构（零拷贝设计）
struct MessageData {
    std::string user_id;         // 发送者ID
    MessageType type;            // 消息类型
    std::shared_ptr<std::vector<uint8_t>> data;  // 消息数据（零拷贝，共享所有权）
    int64_t timestamp_ms;        // 时间戳
    
    // 便捷方法：将二进制数据转换为字符串（仅当类型为TEXT时）
    std::string AsString() const {
        if (type == MessageType::TEXT && data && !data->empty()) {
            return std::string(reinterpret_cast<const char*>(data->data()), data->size());
        }
        return "";
    }
    
    // 便捷方法：获取数据指针和大小
    const uint8_t* GetData() const { return data ? data->data() : nullptr; }
    size_t GetSize() const { return data ? data->size() : 0; }
};

// 消息回调函数类型
using MessageCallback = std::function<void(std::shared_ptr<MessageData>)>;

// 消息消费者
class MessageConsumer {
public:
    explicit MessageConsumer(MessageCallback binary_callback=nullptr, MessageCallback text_callback=nullptr);
    ~MessageConsumer() = default;
    
    // 处理收到的文本消息
    void OnTextMessage(const std::string& user_id, const char* message, size_t length);
    
    // 处理收到的二进制消息
    void OnBinaryMessage(const std::string& user_id, const uint8_t* data, size_t length);

private:
    MessageCallback binary_callback_;
    MessageCallback text_callback_;
};

// RTC Engine 封装类
// 负责 RTC 初始化、推流、订阅等核心功能
class RTCEngine : public bytertc::IRTCEngineEventHandler,
                  public bytertc::IRTCRoomEventHandler {
public:
    explicit RTCEngine(const config::RTCConfig& config);
    ~RTCEngine();

    // 初始化 RTC 引擎
    bool Initialize();
    
    // 加入房间
    bool JoinRoom();
    
    // 开始推流
    bool StartPublish();
    
    // 停止推流
    void StopPublish();
    
    // 停止所有订阅
    void StopSubscribe();
    
    // 推送视频帧
    bool PushVideoFrame(const std::vector<uint8_t>& frame_data);
    
    // 发送房间广播消息
    bool SendRoomMessage(const std::string& message);
    
    // 订阅远端用户流
    bool SubscribeRemoteStream(const bytertc::StreamInfo& stream_info);
    
    // 取消订阅
    void UnsubscribeRemoteStream(const bytertc::StreamInfo& stream_info);
    
    // ==================== Consumer 管理 ====================
    
    // 注册视频帧消费者（用于接收指定用户的视频流）
    // user_id: 要订阅的远端用户ID
    // consumer: 视频帧消费者
    void RegisterVideoConsumer(const std::string& user_id, std::shared_ptr<VideoFrameConsumer> consumer);
    
    // 注册消息消费者（用于接收房间广播消息）
    void RegisterMessageConsumer(std::shared_ptr<MessageConsumer> consumer);
    
    // 销毁引擎
    void Destroy();
    
    // 检查是否正在运行
    bool IsRunning() const { return running_; }

private:
    // ==================== IRTCEngineEventHandler 回调 ====================
    void onWarning(int warn) override;
    void onError(int err) override;
    
    // ==================== IRTCRoomEventHandler 回调 ====================
    void onRoomStateChanged(const char* room_id, const char* uid, 
                           int state, const char* extra_info) override;
    void onLeaveRoom(const bytertc::RtcRoomStats& stats) override;
    void onUserJoined(const bytertc::UserInfo& user_info) override;
    void onUserLeave(const char* uid, bytertc::UserOfflineReason reason) override;
    void onUserPublishStreamVideo(const char* stream_id, const bytertc::StreamInfo& stream_info, 
                                  bool is_publish) override;
    void onUserPublishStreamAudio(const char* stream_id, const bytertc::StreamInfo& stream_info,
                                  bool is_publish) override;
    void onRoomMessageReceived(const char* uid, const char* message) override;
    
    // 设备初始化
    bool InitVideoDevice();
    bool InitAudioDevice();
    bool InitVideoConfig();
    bool InitAudioConfig();
    
    // 订阅管理
    void CheckAndSubscribe(const bytertc::StreamInfo& stream_info);
    bool ShouldSubscribe(const std::string& user_id) const;

    config::RTCConfig config_;
    
    // 状态管理
    // initialized_: RTC 引擎是否初始化完成
    std::atomic<bool> initialized_{false};
    // publishing_: 是否正在推流
    std::atomic<bool> publishing_{false};
    
    // RTC SDK 对象
    bytertc::IRTCEngine* rtc_engine_ = nullptr;
    bytertc::IRTCRoom* rtc_room_ = nullptr;
    
    mutable absl::Mutex subscribe_mutex_;
    // 订阅状态 - key: stream_id, value: StreamInfo
    std::map<std::string, bytertc::StreamInfo> subscribed_streams_ ABSL_GUARDED_BY(subscribe_mutex_);
    
    // ==================== Consumer 管理 ====================
    mutable absl::Mutex consumer_mutex_;
    // VideoFrameConsumer管理 - key: user_id, value: consumer
    std::map<std::string, std::shared_ptr<VideoFrameConsumer>> video_consumers_ ABSL_GUARDED_BY(consumer_mutex_);
    // 订阅的用户ID集合
    std::set<std::string> subscribed_user_ids_;
    // 消息消费者列表
    std::vector<std::shared_ptr<MessageConsumer>> message_consumers_ ABSL_GUARDED_BY(consumer_mutex_);

    // 是否正在运行
    std::atomic<bool> running_{false};
};

}  // namespace rtc
}  // namespace rtele

#endif  // RTELE_RTC_RTC_ENGINE_H_
