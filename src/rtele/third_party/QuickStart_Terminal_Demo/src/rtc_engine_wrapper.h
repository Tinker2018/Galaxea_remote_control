#include "bytertc_engine.h"
#include "util/thread_loop.h"
#include <vector>
#include <memory>

struct StuVideoCaptureConfig;
class RTCVideoEngineWrapper:public bytertc::IRTCEngineEventHandler,
	public bytertc::IRTCRoomEventHandler
{
public:
	RTCVideoEngineWrapper();
	~RTCVideoEngineWrapper();
	int init();
	int joinRoom();
	int destory();
	static RTCVideoEngineWrapper *instance();
protected:
	int initVideoDevice();
	int initAudioConfig();
	int initVideoConfig();
	void pushExternalVideoFrame();
	void pushExternalAudioFrame();
protected:
	//bytertc::IRTCVideoEventHandler
	void onConnectionStateChanged(bytertc::ConnectionState state) override;
	void onPerformanceAlarms(const char* stream_id, const bytertc::StreamInfo& stream_info, bytertc::PerformanceAlarmMode mode,
            bytertc::PerformanceAlarmReason reason, const bytertc::SourceWantedData& data) override;
	void onRemoteAudioStateChanged(
		const char* stream_id, const bytertc::StreamInfo& stream_info, bytertc::RemoteAudioState state, bytertc::RemoteAudioStateChangeReason reason) override;

	void onAudioDeviceStateChanged(const char* device_id, bytertc::RTCAudioDeviceType device_type,
            bytertc::MediaDeviceState device_state, bytertc::MediaDeviceError device_error) override;
	
	void onVideoDeviceStateChanged(const char* device_id, bytertc::RTCVideoDeviceType device_type,
            bytertc::MediaDeviceState device_state, bytertc::MediaDeviceError device_error) override;
	
	void onFirstLocalVideoFrameCaptured(bytertc::IVideoSource* video_source, const bytertc::VideoFrameInfo& info) override;
	void onLocalVideoSizeChanged(bytertc::IVideoSource* video_source, const bytertc::VideoFrameInfo& info) override;
	void onRemoteVideoSizeChanged(const char* stream_id, const bytertc::StreamInfo& stream_info, const bytertc::VideoFrameInfo& info) override;
	void onFirstRemoteVideoFrameRendered(const char* stream_id, const bytertc::StreamInfo& stream_info, const bytertc::VideoFrameInfo& info) override;
	void onFirstRemoteVideoFrameDecoded(const char* stream_id, const bytertc::StreamInfo& stream_info, const bytertc::VideoFrameInfo& info) override;

protected:
		//bytertc::IRTCRoomEventHandler
	void onRoomStateChanged(const char* room_id, const char* uid, int state, const char* extra_info) override;
	void onWarning(int warn) override;
	void onError(int err) override ;
	void onLeaveRoom(const bytertc::RtcRoomStats& stats) override;
	void onRoomStats(const bytertc::RtcRoomStats& stats) override;
	void onUserJoined(const bytertc::UserInfo& userInfo) override;
	void onUserLeave(const char* uid, bytertc::UserOfflineReason reason)override;
	void onUserPublishStreamVideo(const char* stream_id, const bytertc::StreamInfo& stream_info, bool is_publish) override;
	void onUserPublishStreamAudio(const char* stream_id, const bytertc::StreamInfo& stream_info, bool is_publish) override;
	// void onUserPublishScreenVideo(const char* room_id, const char* uid, bool is_publish) override;
	// void onUserPublishScreenAudio(const char* room_id, const char* uid, bool is_publish) override;
	
	void onUserPublishScreen(const char* uid, bytertc::MediaStreamType type);
	void onUserUnpublishScreen(const char* uid, bytertc::MediaStreamType type, bytertc::StreamRemoveReason reason);
	void onStreamRemove(const bytertc::MediaStreamInfo& bs, bytertc::StreamRemoveReason reason) override;
	void onStreamAdd(const bytertc::MediaStreamInfo& stream) override;
	void onStreamPublishSuccess(const char* user_id, bool is_screen) override;
	void onRoomMessageReceived(const char* uid, const char* message) override;
	void onRoomBinaryMessageReceived(const char* uid, int size, const uint8_t* message) override;
	void onUserMessageReceived(const char* uid, const char* message) override;
	void onUserBinaryMessageReceived(const char* uid, int size, const uint8_t* message) override;
	void onRoomMessageSendResult(int64_t msgid, int error) override;
	void onUserMessageSendResult(int64_t message_id, int error) override;
private:
	bytertc::IRTCEngine *m_pVideoEngine = nullptr;
	bytertc::IRTCRoom  *m_pRtcRoom = nullptr;
	//audio 
	std::vector<uint8_t> m_vecAudioPCMData;
	int				     m_nCurrentAudioFrameIndex;
	int					 m_nTotalAudioFrames;
	//video	
	std::vector<uint8_t> m_vecVideoYUVData;
	int				     m_nCurrentVideoFrameIndex;
	int					 m_nTotalVideoFrames;
	std::unique_ptr<StuVideoCaptureConfig> m_customCaptureConfig;
	std::unique_ptr<ThreadLoop>    m_threadLoop;
	std::mutex						m_exitMutex;
};
