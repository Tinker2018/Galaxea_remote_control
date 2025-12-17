#ifndef RTELE_RTC_RTC_ENGINE_H_
#define RTELE_RTC_RTC_ENGINE_H_

#include <memory>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <atomic>
#include <functional>

#include "src/config/rtc_config.h"
#include "absl/synchronization/mutex.h"

// ==================== SDK 头文件包含策略 ====================
#ifdef VOLC_RTC_ARM64
    // [ARM / Orin] 3.58+ 新版 SDK
    #include "bytertc_video.h" 
    #include "rtc/bytertc_video_frame.h"
    #include "bytertc_room.h"
    #include "bytertc_room_event_handler.h"
#else
    // [x86 / PC] 旧版 SDK
    #include "bytertc_engine.h"
    #include "bytertc_room.h"
#endif

namespace rtele {
namespace rtc {

struct VideoFrameData {
    std::vector<uint8_t> data;
    int width;
    int height;
    int64_t timestamp_us;
    std::string user_id;
};

struct MessageData {
    std::string user_id;
    enum class Type { TEXT, BINARY } type;
    std::shared_ptr<std::vector<uint8_t>> data;
    int64_t timestamp_ms;

    const uint8_t* GetData() const { return data ? data->data() : nullptr; }
    size_t GetSize() const { return data ? data->size() : 0; }
};

using VideoFrameCallback = std::function<void(std::shared_ptr<VideoFrameData>)>;
using MessageCallback = std::function<void(std::shared_ptr<MessageData>)>;

#ifdef VOLC_RTC_ARM64
class VideoFrameConsumer : public bytertc::IVideoSink {
#else
class VideoFrameConsumer : public bytertc::IVideoSink {
#endif
public:
    VideoFrameConsumer(const std::string& user_id, VideoFrameCallback callback);
    ~VideoFrameConsumer() override = default;
    bool onFrame(bytertc::IVideoFrame* video_frame) override;
    
    int getRenderElapse() override { return 16; }
    void release() override {}

private:
    std::string user_id_;
    VideoFrameCallback callback_;
};

class MessageConsumer {
public:
    explicit MessageConsumer(MessageCallback callback);
    void OnBinaryMessage(const std::string& user_id, const uint8_t* data, size_t length);
private:
    MessageCallback callback_;
};

class RTCEngine : public bytertc::IRTCEngineEventHandler,
                  public bytertc::IRTCRoomEventHandler {
public:
    explicit RTCEngine(const config::RTCConfig& config);
    ~RTCEngine();

    bool Initialize();
    bool JoinRoom();
    bool StartPublish();
    void StopPublish();
    void Destroy();
    bool PushVideoFrame(const std::vector<uint8_t>& yuv_data);
    bool SendRoomBinaryMessage(const std::vector<uint8_t>& data);
    void RegisterVideoConsumer(const std::string& user_id, std::shared_ptr<VideoFrameConsumer> consumer);
    void RegisterMessageConsumer(std::shared_ptr<MessageConsumer> consumer);

private:
    void onWarning(int warn) override;
    void onError(int err) override;
    
    void onRoomStateChanged(const char* room_id, const char* uid, int state, const char* extra_info) override;
    
    // --- 核心差异点：onUserJoined 签名 ---
#ifdef VOLC_RTC_ARM64
    // ARM: 两个参数 (UserInfo, elapsed)
    void onUserJoined(const bytertc::UserInfo& user_info, int elapsed) override;
#else 
    // x86: 只有一个参数 (UserInfo)
    void onUserJoined(const bytertc::UserInfo& user_info) override;
#endif
    
    void onUserLeave(const char* uid, bytertc::UserOfflineReason reason) override;

    // --- 差异回调声明 ---
#ifdef VOLC_RTC_ARM64
    void onUserPublishStream(const char* uid, bytertc::MediaStreamType type) override;
    void onUserUnPublishStream(const char* uid, bytertc::MediaStreamType type, bytertc::StreamRemoveReason reason) override;
#else 
    // x86 旧版回调
    void onUserPublishStreamVideo(const char* uid, const bytertc::StreamInfo& info, bool is_publish) override;
#endif

    void onRoomBinaryMessageReceived(const char* uid, int size, const uint8_t* data) override;

    bool InitVideoDevice();
    bool CheckAndSubscribe(const std::string& user_id);

    config::RTCConfig config_;
    std::atomic<bool> initialized_{false};
    std::atomic<bool> publishing_{false};

#ifdef VOLC_RTC_ARM64
    bytertc::IRTCVideo* rtc_engine_ = nullptr; 
#else
    bytertc::IRTCEngine* rtc_engine_ = nullptr; 
#endif
    bytertc::IRTCRoom* rtc_room_ = nullptr;

    mutable absl::Mutex consumer_mutex_;
    std::map<std::string, std::shared_ptr<VideoFrameConsumer>> video_consumers_ ABSL_GUARDED_BY(consumer_mutex_);
    std::vector<std::shared_ptr<MessageConsumer>> message_consumers_ ABSL_GUARDED_BY(consumer_mutex_);
};

} // namespace rtc
} // namespace rtele

#endif // RTELE_RTC_RTC_ENGINE_H_