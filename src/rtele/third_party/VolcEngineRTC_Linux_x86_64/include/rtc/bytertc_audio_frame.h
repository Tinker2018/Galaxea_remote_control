/*
 * Copyright (c) 2020 The VolcEngineRTC project authors. All Rights Reserved.
 * @brief VolcEngineRTC audio Frame
 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include <memory>
#include "bytertc_audio_defines.h"

namespace bytertc {

/**
 * @locale zh
 * @type keytype
 * @region 音频管理
 * @brief 音频帧类型
 */
/**
 * @locale en
 * @type keytype
 * @region Audio Management
 * @brief Audio frame type
 */
enum AudioFrameType {
    /**
     * @locale zh
     * @brief PCM 16bit
     */
    /**
     * @locale en
     * @brief PCM 16bit
     */
    kAudioFrameTypePCM16 = 0
};

/**
 * @locale zh
 * @type keytype
 * @region 音频管理
 * @brief 音频帧构建类
 */
/**
 * @locale en
 * @type keytype
 * @region audio management
 * @brief Audio frame construction class
 */
typedef struct AudioFrameBuilder {
    /**
     * @locale zh
     * @brief 音频采样率
     */
    /**
     * @locale en
     * @brief Audio Sampling Rate
     */
    AudioSampleRate sample_rate;

    /**
     * @locale zh
     * @brief 音频通道数
     */
    /**
     * @locale en
     * @brief Number of audio channels
     */
    AudioChannel channel;

    /**
     * @locale zh
     * @brief 音频帧时间戳，单位：微秒
     * @note 此时间戳不是 Linux 时间戳，只是 SDK 内部时间戳，仅保证帧之间时间戳的相对关系。
     */
    /**
     * @locale en
     * @brief Audio frame timestamp in microseconds
     * @note This timestamp is not a Linux timestamp; it is an internal timestamp of the SDK, which only ensures the relative relationship between the timestamps of frames.
     */
    int64_t timestamp_us = 0;

    /**
     * @locale zh
     * @brief 音频帧数据
     */
    /**
     * @locale en
     * @brief Audio frame data
     */
    uint8_t* data;

    /**
     * @locale zh
     * @brief 音频帧数据大小
     */
    /**
     * @locale en
     * @brief Audio frame data size
     */
    int64_t data_size = 0;

    /**
     * @locale zh
     * @brief 是否深拷贝
     */
    /**
     * @locale en
     * @brief Is deep-copy or not
     */
    bool deep_copy = true;
    /**
     * @locale zh
     * @hidden for internal use only
     * @brief 音频帧额外信息数据
     */
    /**
     * @locale en
     * @hidden for internal use only
     * @brief Audio frame extra info data
     */
    uint8_t* extra_info = nullptr;
    /**
     * @locale zh
     * @hidden for internal use only
     * @brief 音频帧额外信息数据大小
     */
    /**
     * @locale en
     * @hidden for internal use only
     * @brief Audio frame extra info data size
     */
    int64_t extra_info_size = 0;
} AudioFrameBuilder;
/**
 * @locale zh
 * @type keytype
 * @brief 音频帧
 */
/**
 * @locale en
 * @type keytype
 * @brief Audio frame
 */
class IAudioFrame {
public:
    /**
     * @locale zh
     * @type api
     * @brief 获取音频帧时间戳。
     * @return 音频帧时间戳，单位：微秒
     * @list 音频管理
     */
    /**
     * @locale en
     * @type api
     * @brief Gets audio frame timestamp.
     * @return Audio frame timestamp in microseconds
     * @list Audio Management
     */
    virtual int64_t timestampUs() const = 0;
    /**
     * @locale zh
     * @type api
     * @brief 获取音频采样率。参看 AudioSampleRate{@link #AudioSampleRate}
     * @return 音频采样率，单位：Hz
     * @list 音频管理
     */
    /**
     * @locale en
     * @type api
     * @brief Gets audio sample rate. See AudioSampleRate{@link #AudioSampleRate}
     * @return Audio sample rate in Hz
     * @list Audio Management
     */
    virtual AudioSampleRate sampleRate() const = 0;
    /**
     * @locale zh
     * @type api
     * @brief 获取音频通道数。参看 AudioChannel{@link #AudioChannel}
     * @return 音频通道数
     * @note 双声道的情况下，左右声道的音频帧数据以 LRLRLR 形式排布。
     * @list 音频管理
     */
    /**
     * @locale en
     * @type api
     * @brief Gets the number of audio channels. See AudioChannel{@link #AudioChannel}
     * @return Number of audio channels
     * @note For dual channels, the audio frames are interleaved.
     * @list Audio Management
     */
    virtual AudioChannel channel() const = 0;
    /**
     * @locale zh
     * @type api
     * @brief 获取音频帧内存块地址
     * @return 音频帧内存块地址
     * @list 音频管理
     */
    /**
     * @locale en
     * @type api
     * @brief Gets audio frame memory address
     * @return Audio frame memory address
     * @list Audio Management
     */
    virtual uint8_t* data() const = 0;
    /**
     * @locale zh
     * @type api
     * @brief 获取音频帧数据大小
     * @return 音频帧数据大小，单位：字节。
     * @list 音频管理
     */
    /**
     * @locale en
     * @type api
     * @brief Getd audio frame data size
     * @return Audio frame data size in bytes.
     * @list Audio Management
     */
    virtual int dataSize() const = 0;
    /**
     * @locale zh
     * @type api
     * @brief 获取音频帧类型
     * @return 音频帧类型，目前只支持 PCM，详见 AudioFrameType{@link #AudioFrameType}
     * @list 音频管理
     */
    /**
     * @locale en
     * @type api
     * @brief Gets audio frame type
     * @return Audio frame type, support PCM only. See AudioFrameType{@link #AudioFrameType}
     * @list Audio Management
     */
    virtual AudioFrameType frameType() const = 0;
    /**
     * @locale zh
     * @type api
     * @hidden for internal use only
     * @brief 获取音频帧额外信息内存块地址
     * @return 音频帧额外信息内存块地址
     * @list 音频管理
     */
    /**
     * @locale en
     * @type api
     * @hidden for internal use only
     * @brief Gets audio frame extra info memory address
     * @return Audio frame extra info memory address
     * @list Audio Management
     */    
    virtual uint8_t* extraInfo() const = 0;
    /**
     * @locale zh
     * @type api
     * @hidden for internal use only
     * @brief 获取音频帧额外信息大小
     * @return 音频帧数据额外信息大小，单位：字节。
     * @list 音频管理
     */
    /**
     * @locale en
     * @type api
     * @hidden for internal use only
     * @brief Getd audio frame extra info data size
     * @return Audio frame extra info data size in bytes.
     * @list Audio Management
     */
    virtual int extraInfoSize() const = 0;
    /**
     * @locale zh
     * @type api
     * @brief 释放音频帧
     * @list 音频管理
     */
    /**
     * @locale en
     * @type api
     * @brief Release audio frames
     * @list Audio Management
     */
    virtual void release() = 0;
    /**
     * @locale zh
     * @type api
     * @brief 获取音频静音标志
     * @return 是否静音数据 <br>
     *        - true: 是
     *        - false: 否
     * @list 音频管理
     */
    /**
     * @locale en
     * @type api
     * @brief Gets audio mute state identifier
     * @return Is the data muted: <br>
     *         - true: Yes
     *         - false: No
     * @list Audio Management
     */
    virtual bool isMutedData() const = 0;
    /**
     * @locale zh
     * @hidden constructor/destructor
     */
    /**
     * @locale en
     * @hidden constructor/destructor
     */
protected:
    /**
     * @locale zh
     * @hidden constructor/destructor
     * @brief 析构函数
     */
    /**
     * @locale en
     * @hidden constructor/destructor
     * @brief Destructor
     */
    virtual ~IAudioFrame() = default;
};
/**
 * @locale zh
 * @type api
 * @brief 创建 IAudioFrame
 * @param builder 音频帧构建实例，参看 AudioFrameBuilder{@link #AudioFrameBuilder}
 * @return 详见 IAudioFrame{@link #IAudioFrame}
 * @list 音频管理
 */
/**
 * @locale en
 * @type api
 * @brief Create IAudioFrame
 * @param builder Audio frame build instance. See AudioFrameBuilder{@link #AudioFrameBuilder}
 * @return Refer to IAudioFrame{@link #IAudioFrame} for more details.
 * @list Audio Management
 */
BYTERTC_API IAudioFrame* buildAudioFrame(const AudioFrameBuilder& builder);

/**
 * @locale zh
 * @type keytype
 * @brief 音频回调方法
 */
/**
 * @locale en
 * @type keytype
 * @brief Audio data callback method
 */
enum class AudioFrameCallbackMethod{
    /**
     * @locale zh
     * @brief 本地麦克风录制的音频数据回调
     */
    /**
     * @locale en
     * @brief The callback of the audio data recorded by local microphone.
     */
    kRecord,
    /**
     * @locale zh
     * @brief 订阅的远端所有用户混音后的音频数据回调
     */
    /**
     * @locale en
     * @brief The callback of the mixed audio data of all remote users subscribed by the local user.
     */
    kPlayback,
    /**
     * @locale zh
     * @brief 本地麦克风录制和订阅的远端所有用户混音后的音频数据回调
     */
    /**
     * @locale en
     * @brief The callback of the mixed audio data including the data recorded by local microphone and that of all remote users subscribed by the local user.
     */
    kMixed,
    /**
     * @locale zh
     * @brief 订阅的远端每个用户混音前的音频数据回调
     */
    /**
     * @locale en
     * @brief The callback of the audio data before mixing of each remote user subscribed by the local user.
     */
    kRemoteUser,
    /**
     * @locale zh
     * @brief 本地屏幕录制的音频数据回调
     */
    /**
     * @locale en
     * @brief The callback of screen audio data captured locally.
     */
    kRecordScreen,
    /**
     * @locale zh
     * @brief 本地麦克风录制和本地 `MediaPlayer`, `EffectPlayer` 播放的音频混音后的音频数据回调
     */
    /**
     * @locale en
     * @brief The callback of the mixed audio data including the data recorded by local microphone and the audio data of `MediaPlayer` and `EffectPlayer`.
     */
    kCaptureMixed,
};
/**
 * @locale zh
 * @type callback
 * @brief 音频数据回调观察者 <br>
 * 注意：回调函数是在 SDK 内部线程（非 UI 线程）同步抛出来的，请不要做耗时操作或直接操作 UI，否则可能导致 app 崩溃。 <br>
 * 本接口类中的回调周期均为 20 ms。
 * @list 自定义流处理
 */
/**
 * @locale en
 * @type callback
 * @brief Audio data callback observer <br>
 * Note: Callback functions are thrown synchronously in a non-UI thread within the SDK. Therefore, you must not perform any time-consuming operations or direct UI operations within the callback function, as this may cause the app to crash. <br>
 * The time interval for all callback functions in this interface is 20 ms.
 * @list Custom Stream Processing
 */
class IAudioFrameObserver {
public:
    /**
     * @locale zh
     * @hidden constructor/destructor
     * @brief 析构函数
     */
    /**
     * @locale en
     * @hidden constructor/destructor
     * @brief Destructor
     */
    virtual ~IAudioFrameObserver() {
    }
    /**
     * @locale zh
     * @hidden for internal use only
     * @valid since 3.50
     */
    /**
     * @locale en
     * @hidden for internal use only
     * @valid since 3.50
     */
    virtual void onRecordAudioFrameOriginal(const IAudioFrame& audio_frame) = 0;
    /**
     * @locale zh
     * @type callback
     * @brief 返回麦克风录制的音频数据
     * @param audio_frame 音频数据, 详见：IAudioFrame{@link #IAudioFrame}
     * @list 自定义流处理
     */
    /**
     * @locale en
     * @type callback
     * @brief Returns audio data recorded by microphone
     * @param audio_frame Audio data. See IAudioFrame{@link #IAudioFrame}
     * @list Custom Stream Processing
     */
    virtual void onRecordAudioFrame(const IAudioFrame& audio_frame) = 0;
    /**
     * @locale zh
     * @type callback
     * @brief 返回订阅的所有远端用户混音后的音频数据。
     * @param audio_frame 音频数据, 详见：IAudioFrame{@link #IAudioFrame}
     * @list 自定义流处理
     */
    /**
     * @locale en
     * @type callback
     * @brief Returns the mixed audio data of all subscribed remote users
     * @param audio_frame Audio data. See IAudioFrame{@link #IAudioFrame}
     * @list Custom Stream Processing
     */
    virtual void onPlaybackAudioFrame(const IAudioFrame& audio_frame) = 0;
    /**
     * @locale zh
     * @type callback
     * @brief 返回远端单个用户的音频数据
     * @param stream_id 远端流 ID。
     * @param stream_info 远端流信息，参看 StreamInfo{@link #StreamInfo}。
     * @param audio_frame 音频数据， 参看 IAudioFrame{@link #IAudioFrame}。
     * @note 此回调在播放线程调用。不要在此回调中做任何耗时的事情，否则可能会影响整个音频播放链路。
     * @list 自定义流处理
     */
    /**
     * @locale en
     * @type callback
     * @brief Returns the audio data of one remote user.
     * @param stream_id Remote stream ID.
     * @param stream_info Remote stream information. See StreamInfo{@link #StreamInfo}.
     * @param audio_frame Audio data. See IAudioFrame{@link #IAudioFrame}
     * @note This callback works on the playback thread. Don't do anything time-consuming in this callback, or it may affect the entire audio playback chain.
     * @list Custom Stream Processing
     */
    virtual void onRemoteUserAudioFrame(const char* stream_id, const StreamInfo& stream_info, const IAudioFrame& audio_frame) = 0;
    /**
     * @locale zh
     * @type callback
     * @brief 返回本地麦克风录制和订阅的所有远端用户混音后的音频数据
     * @param audio_frame 音频数据, 详见：IAudioFrame{@link #IAudioFrame}
     * @list 自定义流处理
     */
    /**
     * @locale en
     * @type callback
     * @brief Returns mixed audio data including both data recorded by the local microphone and data from all subscribed remote users
     * @param audio_frame Audio data. See IAudioFrame{@link #IAudioFrame}
     * @list Custom Stream Processing
     */
    virtual void onMixedAudioFrame(const IAudioFrame& audio_frame) = 0;
    /**
     * @locale zh
     * @type callback
     * @brief 返回本地屏幕录制的音频数据
     * @param audio_frame 音频数据, 详见：IAudioFrame{@link #IAudioFrame}
     * @list 自定义流处理
     */
    /**
     * @locale en
     * @type callback
     * @brief Returns the audio data played locally
     * @param audio_frame Audio data. See IAudioFrame{@link #IAudioFrame}
     * @list Custom Stream Processing
     */
    virtual void onRecordScreenAudioFrame(const IAudioFrame& audio_frame) {
    }
    /**
     * @locale zh
     * @type callback
     * @brief 返回本地麦克风录制的音频数据，本地 `MediaPlayer` / `EffectPlayer` 播放音频文件混音后的音频数据
     * @param audio_frame 音频数据, 详见：IAudioFrame{@link #IAudioFrame}
     * @list 自定义流处理
     */
     /**
      * @locale en
      * @type callback
      * @brief Return the mixed audio data of the data recorded by local microphone and the audio data of `MediaPlayer` and `EffectPlayer`.
      * @param audio_frame Audio data. See IAudioFrame{@link #IAudioFrame}
      * @list Custom Stream Processing
      */
    virtual void onCaptureMixedAudioFrame(const IAudioFrame& audio_frame) = 0;

};
/**
 * @locale zh
 * @type callback
 * @brief 自定义音频处理器。 <br>
 * 注意：回调函数是在 SDK 内部线程（非 UI 线程）同步抛出来的，请不要做耗时操作或直接操作 UI，否则可能导致 app 崩溃。
 * @list Custom Stream Processing
 */
/**
 * @locale en
 * @type callback
 * @brief The custom audio processor. <br>
 * Note: Callback functions are thrown synchronously in a non-UI thread within the SDK. Therefore, you must not perform any time-consuming operations or direct UI operations within the callback function, as this may cause the app to crash.
 * @list Custom Stream Processing
 */
class IAudioFrameProcessor{
public:
    /**
     * @locale zh
     * @type callback
     * @brief 回调本地采集的音频帧地址，供自定义音频处理。
     * @param audio_frame 音频帧地址，参看 IAudioFrame{@link #IAudioFrame}
     * @note
     *        - 完成自定义音频处理后，SDK 会对处理后的音频帧进行编码，并传输到远端。
     *        - 调用 `enableAudioProcessor`，并在参数中选择本地采集的音频时，每 10 ms 收到此回调。
     * @list 自定义流处理
     */
    /**
     * @locale en
     * @type callback
     * @brief Returns the address of the locally captured audio frame for custom processing.
     * @param audio_frame The address of the audio frame. See IAudioFrame{@link #IAudioFrame}
     * @note
     *        - After custom processing, SDK will encode the processed audio frames and transmit to the remote user.
     *        - After calling `enableAudioProcessor` with locally captured audio frame specified, you will receive this callback every 10 ms.
     * @list Custom Stream Processing
     */
    virtual int onProcessRecordAudioFrame(IAudioFrame& audio_frame) = 0;
    /**
     * @locale zh
     * @type callback
     * @brief 回调远端音频混音的音频帧地址，供自定义音频处理。
     * @param audio_frame 音频帧地址，参看 IAudioFrame{@link #IAudioFrame}
     * @note 调用 `enableAudioProcessor`，并在参数中选择远端音频流的的混音音频时，每 10 ms 收到此回调。
     * @list 自定义流处理
     */
    /**
     * @locale en
     * @type callback
     * @brief Returns the address of the locally captured audio frame for custom processing.
     * @param audio_frame The address of the audio frame. See IAudioFrame{@link #IAudioFrame}
     * @note After calling `enableAudioProcessor` with mixed remote audio, you will receive this callback every 10 ms.
     * @list Custom Stream Processing
     */
    virtual int onProcessPlayBackAudioFrame(IAudioFrame& audio_frame) = 0;
    /**
     * @locale zh
     * @type callback
     * @brief 回调单个远端用户的音频帧地址，供自定义音频处理。
     * @param stream_id 远端流 ID。
     * @param stream_info 远端流信息， 参看 StreamInfo{@link #StreamInfo}。
     * @param audio_frame 音频帧地址，参看 IAudioFrame{@link #IAudioFrame}
     * @note 调用 `enableAudioProcessor`，并在参数中选择各个远端音频流时，每 10 ms 收到此回调。
     * @list 自定义流处理
     */
    /**
     * @locale en
     * @type callback
     * @brief Returns the address of the locally captured audio frame for custom processing.
     * @param stream_id Remote stream ID.
     * @param stream_info Remote stream information. See StreamInfo{@link #StreamInfo}.
     * @param audio_frame The address of the audio frame. See IAudioFrame{@link #IAudioFrame}
     * @note After calling `enableAudioProcessor` with audio streams of the remote users. You will receive this callback every 10 ms.
     * @list Custom Stream Processing
     */
    virtual int onProcessRemoteUserAudioFrame(const char* stream_id, const StreamInfo& stream_info, IAudioFrame& audio_frame) = 0;
    /**
     * @locale zh
     * @hidden(macOS, Windows, Linux)
     * @valid since 3.50
     * @type callback
     * @brief 软件耳返音频数据的回调。你可根据此回调自定义处理音频。 <br>
     *        耳返音频中包含通过调用 `setVoiceReverbType` 和 `setVoiceChangerType` 设置的音频特效。
     * @param audio_frame 音频帧地址。参看 IAudioFrame{@link #IAudioFrame}。
     * @return
     *        - 0： 成功。
     *        - < 0： 失败。
     * @note
     *        - 此数据处理只影响软件耳返音频数据。
     *        - 要启用此回调，必须调用 `enableAudioProcessor`，并选择耳返音频，每 10 ms 收到此回调。
     * @list Custom Stream Processing
     */
    /**
     * @locale en
     * @hidden(macOS, Windows, Linux)
     * @valid since 3.50
     * @type callback
     * @brief You will receive the address of SDK-level in-ear monitoring audio frames for custom processing. <br>
     *        The audio effects set by `setVoiceReverbType` and `setVoiceChangerType` are included.
     * @param  audio_frame The address of the in-ear monitoring audio frames. See IAudioFrame{@link #IAudioFrame}.
     * @return
     *        - 0: Success.
     *        - <0: Failure.
     * @note
     *        - Modifying the data affects only SDK-level in-ear monitoring audio.
     *        - To enable this callback, call `enableAudioProcessor`. You will receive this callback every 10 ms.
     * @list Custom Stream Processing
     */
    virtual int onProcessEarMonitorAudioFrame(IAudioFrame& audio_frame) = 0;
    /**
     * @locale zh
     * @type callback
     * @brief 屏幕共享的音频帧地址回调。你可根据此回调自定义处理音频。
     * @param audio_frame 音频帧地址，参看 IAudioFrame{@link #IAudioFrame}。
     * @note 调用 `enableAudioProcessor`，把返回给音频处理器的音频类型设置为屏幕共享音频后，每 10 ms 收到此回调。
     * @list 自定义流处理
     */
    /**
     * @locale en
     * @type callback
     * @brief Returns the address of the shared-screen audio frames for custom processing.
     * @param audio_frame The address of audio frames. See IAudioFrame{@link #IAudioFrame}.
     * @note After calling `enableAudioProcessor` to set the audio input to the shared-screen audio. You will receive this callback every 10 ms.
     * @list Custom Stream Processing
     */
    virtual int onProcessScreenAudioFrame(IAudioFrame& audio_frame) = 0;
    /**
     * @locale zh
     * @hidden constructor/destructor
     * @brief 析构函数
     */
    /**
     * @locale en
     * @hidden constructor/destructor
     * @brief Destructor
     */
    virtual ~IAudioFrameProcessor() {
    }
};
/**
 * @locale zh
 * @type keytype
 * @hidden for internal use only
 * @region 音频管理
 * @brief 音频帧构建类
 */
/**
 * @locale en
 * @type keytype
 * @hidden for internal use only
 * @region audio management
 * @brief Audio frame construction class
 */
typedef struct EncodedAudioFrameData {
    /**
     * @locale zh
     * @brief 音频编码类型
     */
    /**
     * @locale en
     * @brief Audio encoding type
     */
    AudioCodecType codec_type = kAudioCodecTypeNone;
    /**
     * @locale zh
     * @brief 音频数据
     */
    /**
     * @locale en
     * @brief Audio data
     */
    uint8_t* data = nullptr;
    /**
     * @locale zh
     * @brief 音频数据大小
     */
    /**
     * @locale en
     * @brief Audio data size
     */
    int size = 0;
    /**
     * @locale zh
     * @brief 音频时间戳，单位：微秒
     */
    /**
     * @locale en
     * @brief Audio timestamp, in microseconds
     */
    int64_t timestamp_us = 0;
    /**
     * @locale zh
     * @brief 音频采样率
     */
    /**
     * @locale en
     * @brief Audio sample rate
     */
    int sample_rate = 0;
    /**
     * @locale zh
     * @brief 音频通道数
     */
    /**
     * @locale en
     * @brief Audio channel number
     */
    int channel_num = 0;
    /**
     * @locale zh
     * @brief 音频帧大小，单位：毫秒
     */
    /**
     * @locale en
     * @brief Audio frame size, in milliseconds
     */
    int frame_size_ms = 0;
    /**
     * @locale zh
     * @hidden for internal use only
     * @brief 音频帧额外信息数据
     */
    /**
     * @locale en
     * @hidden for internal use only
     * @brief Audio frame extra info data
     */
    uint8_t* extra_info = nullptr;
    /**
     * @locale zh
     * @hidden for internal use only
     * @brief 音频帧额外信息数据大小
     */
    /**
     * @locale en
     * @hidden for internal use only
     * @brief Audio frame extra info data size
     */
    int extra_info_size = 0;
} EncodedAudioFrameData;

/**
 * @locale zh
 * @type callback
 * @hidden for internal use only
 * @brief 远端音频帧监测器
 * @list 自定义流处理
 */
/**
 * @locale en
 * @type callback
 * @hidden for internal use only
 * @brief Remote audio frame monitor
 * @list Custom Stream Processing
 */
class IRemoteEncodedAudioFrameObserver  {
public:
    virtual ~IRemoteEncodedAudioFrameObserver () {
    }
    /**
     * @locale zh
     * @type callback
     * @hidden for internal use only
     * @brief 调用 RegisterRemoteEncodedAudioFrameObserver 后，SDK 收到远端音频帧信息时，回调该事件
     * @param stream_id 远端流 ID
     * @param stream_info 远端流信息，参看 StreamInfo{@link #StreamInfo}
     * @param audio_stream 远端音频帧信息，参看 EncodedAudioFrameData{@link #EncodedAudioFrameData}
     * @list 自定义流处理
     */
    /**
     * @locale en
     * @type callback
     * @hidden for internal use only
     * @brief Call RegisterRemoteEncodedAudioFrameObserver, when the SDK receives the remote audio frame information, callback the event
     * @param stream_id Remote stream ID
     * @param stream_info Remote stream information. See StreamInfo{@link #StreamInfo}
     * @param audio_stream The remote audio frame information. See EncodedAudioFrameData{@link #EncodedAudioFrameData}
     * @list Custom Stream Processing
     */
    virtual void onRemoteEncodedAudioFrame(const char* stream_id, 
            const StreamInfo& stream_info, const EncodedAudioFrameData& audio_stream) = 0;
};

}  // namespace bytertc
