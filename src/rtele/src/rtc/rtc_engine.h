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

struct VideoFrameData {
    std::vector<uint8_t> data;
    int width;
    int height;
    int64_t timestamp_us;
    std::string user_id;
    std::string stream_id;
};

using VideoFrameCallback = std::function<void(std::shared_ptr<VideoFrameData>)>;

class VideoFrameConsumer : public bytertc::IVideoSink {
public:
    explicit VideoFrameConsumer(const std::string& user_id, VideoFrameCallback callback);
    ~VideoFrameConsumer() override = default;
    
    const std::string& GetUserId() const { return user_id_; }
    
    bool onFrame(bytertc::IVideoFrame* video_frame) override;
    int getRenderElapse() override { return 16; }
    void release() override {}

private:
    std::string user_id_;
    VideoFrameCallback callback_;
};

// ==================== 消息相关 ====================

enum class MessageType {
    TEXT,    // 文本消息
    BINARY   // 二进制消息
};

struct MessageData {
    std::string user_id;
    MessageType type;
    std::shared_ptr<std::vector<uint8_t>> data;
    int64_t timestamp_ms;
    
    std::string AsString() const {
        if (type == MessageType::TEXT && data && !data->empty()) {
            return std::string(reinterpret_cast<const char*>(data->data()), data->size());
        }
        return "";
    }
    
    const uint8_t* GetData() const { return data ? data->data() : nullptr; }
    size_t GetSize() const { return data ? data->size() : 0; }
};

using MessageCallback = std::function<void(std::shared_ptr<MessageData>)>;

class MessageConsumer {
public:
    explicit MessageConsumer(MessageCallback binary_callback=nullptr, MessageCallback text_callback=nullptr);
    ~MessageConsumer() = default;
    
    void OnTextMessage(const std::string& user_id, const char* message, size_t length);
    void OnBinaryMessage(const std::string& user_id, const uint8_t* data, size_t length);

private:
    MessageCallback binary_callback_;
    MessageCallback text_callback_;
};

// ==================== RTC Engine ====================

class RTCEngine : public bytertc::IRTCEngineEventHandler,
                  public bytertc::IRTCRoomEventHandler {
public:
    explicit RTCEngine(const config::RTCConfig& config);
    ~RTCEngine();

    bool Initialize();
    bool JoinRoom();
    bool StartPublish();
    void StopPublish();
    void StopSubscribe();
    bool PushVideoFrame(const std::vector<uint8_t>& frame_data);
    
    // 发送文本广播消息
    bool SendRoomMessage(const std::string& message);

    // !!! 新增：发送二进制广播消息 !!!
    bool SendRoomBinaryMessage(const std::vector<uint8_t>& data);
    
    bool SubscribeRemoteStream(const bytertc::StreamInfo& stream_info);
    void UnsubscribeRemoteStream(const bytertc::StreamInfo& stream_info);
    
    void RegisterVideoConsumer(const std::string& user_id, std::shared_ptr<VideoFrameConsumer> consumer);
    void RegisterMessageConsumer(std::shared_ptr<MessageConsumer> consumer);
    void Destroy();
    bool IsRunning() const { return running_; }

private:
    // IRTCEngineEventHandler 回调
    void onWarning(int warn) override;
    void onError(int err) override;
    
    // IRTCRoomEventHandler 回调
    void onRoomStateChanged(const char* room_id, const char* uid, 
                           int state, const char* extra_info) override;
    void onLeaveRoom(const bytertc::RtcRoomStats& stats) override;
    void onUserJoined(const bytertc::UserInfo& user_info) override;
    void onUserLeave(const char* uid, bytertc::UserOfflineReason reason) override;
    void onUserPublishStreamVideo(const char* stream_id, const bytertc::StreamInfo& stream_info, 
                                  bool is_publish) override;
    void onUserPublishStreamAudio(const char* stream_id, const bytertc::StreamInfo& stream_info,
                                  bool is_publish) override;
    
    // 文本消息回调
    void onRoomMessageReceived(const char* uid, const char* message) override;

    // !!! 新增：二进制消息回调 !!!
    void onRoomBinaryMessageReceived(const char* uid, int size, const uint8_t* data) override;
    
    // 设备初始化
    bool InitVideoDevice();
    bool InitAudioDevice();
    bool InitVideoConfig();
    bool InitAudioConfig();
    
    // 订阅管理
    void CheckAndSubscribe(const bytertc::StreamInfo& stream_info);
    bool ShouldSubscribe(const std::string& user_id) const;

    config::RTCConfig config_;
    
    std::atomic<bool> initialized_{false};
    std::atomic<bool> publishing_{false};
    
    bytertc::IRTCEngine* rtc_engine_ = nullptr;
    bytertc::IRTCRoom* rtc_room_ = nullptr;
    
    mutable absl::Mutex subscribe_mutex_;
    std::map<std::string, bytertc::StreamInfo> subscribed_streams_ ABSL_GUARDED_BY(subscribe_mutex_);
    
    mutable absl::Mutex consumer_mutex_;
    std::map<std::string, std::shared_ptr<VideoFrameConsumer>> video_consumers_ ABSL_GUARDED_BY(consumer_mutex_);
    std::set<std::string> subscribed_user_ids_;
    std::vector<std::shared_ptr<MessageConsumer>> message_consumers_ ABSL_GUARDED_BY(consumer_mutex_);

    std::atomic<bool> running_{false};
};

}  // namespace rtc
}  // namespace rtele

#endif  // RTELE_RTC_RTC_ENGINE_H_