/*
 *  Copyright (c) 2022 The VolcEngineRTC project authors. All Rights Reserved.
 */

#pragma once
#include "bytertc_video_frame.h"
#include "bytertc_audio_frame.h"

namespace bytertc {
/**
 * @locale zh
 * @type keytype
 * @brief 转推任务事件
 */
/**
 * @locale en
 * @type keytype
 * @brief Events related to mixed stream task
 */
enum MixedStreamTaskEvent {
    /**
     * @locale zh
     * @hidden for internal use only
     */
    /**
     * @locale en
     * @hidden for internal use only
     */
    kMixedStreamTaskEventBase = 0,
    /**
     * @locale zh
     * @brief 任务发起成功。
     */
    /**
     * @locale en
     * @brief The task initiated successfully.
     */
    kMixedStreamTaskEventStartSuccess = 1,
    /**
     * @locale zh
     * @brief 任务发起失败。
     */
    /**
     * @locale en
     * @brief Failed to initiate the task.
     */
    kMixedStreamTaskEventStartFailed = 2,
    /**
     * @locale zh
     * @brief 任务更新成功。
     */
    /**
     * @locale en
     * @brief The task updated successfully.
     */
    kMixedStreamTaskEventUpdateSuccess = 3,
    /**
     * @locale zh
     * @brief 任务更新失败。
     */
    /**
     * @locale en
     * @brief Failed to update the task.
     */
    kMixedStreamTaskEventUpdateFailed = 4,
    /**
     * @locale zh
     * @brief 任务停止。
     */
    /**
     * @locale en
     * @brief The task stopped.
     */
    kMixedStreamTaskEventStopSuccess = 5,
    /**
     * @locale zh
     * @brief 结束任务失败。
     */
    /**
     * @locale en
     * @brief Failed to stop the task.
     */
    kMixedStreamTaskEventStopFailed = 6,
    /**
     * @locale zh
     * @brief Warning 事件
     */
    /**
     * @locale en
     * @brief Warnings
     */
    kMixedStreamTaskEventWarning = 7,
};


/**
 * @locale zh
 * @type errorcode
 * @brief 单流转推直播事件
 */
/**
 * @locale en
 * @type errorcode
 * @brief Event of pushing a single stream to CDN.
 */
enum SingleStreamTaskEvent {
    /**
     * @locale zh
     * @hidden for internal use only
     */
    /**
     * @locale en
     * @hidden for internal use only
     */
    kSingleStreamTaskEventBase = 0,
    /**
    * @locale zh
    * @brief 任务发起成功。
    */
   /**
    * @locale en
    * @brief The task initiated successfully.
    */
    kSingleStreamTaskEventStartSuccess = 1,
    /**
    * @locale zh
    * @brief 任务发起失败。
    */
   /**
    * @locale en
    * @brief Failed to initiate the task.
    */
    kSingleStreamTaskEventStartFailed = 2,
    /**
    * @locale zh
    * @brief 任务停止。
    */
   /**
    * @locale en
    * @brief The task stopped.
    */
    kSingleStreamTaskEventStopSuccess = 3,
    /**
     * @locale zh
     * @brief 结束任务失败。
     */
    /**
     * @locale en
     * @brief Failed to stop the task.
     */
    kSingleStreamTaskEventStopFailed = 4,
    /**
    * @locale zh
    * @brief Warning 事件
    */
   /**
    * @locale en
    * @brief Warnings
    */
    kSingleStreamTaskEventWarning = 5,
};

/**
 * @locale zh
 * @type errorcode
 * @brief 合流和 WTN 流任务错误码
 */
/**
 * @locale en
 * @type errorcode
 * @brief Mixing stream errors.
 */
enum MixedStreamTaskErrorCode {
    /**
     * @locale zh
     * @brief 推流成功。
     */
    /**
     * @locale en
     * @brief Everything is OK.
     */
    kMixedStreamTaskErrorCodeOK = 0,
    /**
     * @locale zh
     * @hidden currently not available
     * @brief 预留错误码，未启用
     */
    /**
     * @locale en
     * @hidden currently not available
     * @brief Reserved error code, not accessible.
     */
    kMixedStreamTaskErrorCodeBase= 1090,
    /**
     * @locale zh
     * @brief 任务处理超时，请检查网络状态并重试。
     */
    /**
     * @locale en
     * @brief Request timed out. Please check network status and retry.
     */
    kMixedStreamTaskErrorCodeTimeout = 1091,
    /**
     * @locale zh
     * @brief 服务端检测到错误的推流参数。
     */
    /**
     * @locale en
     * @brief Invalid parameters were detected by the server.
     */
    kMixedStreamTaskErrorCodeInvalidParamByServer = 1092,
    /**
     * @locale zh
     * @brief 对流的订阅超时
     */
    /**
     * @locale en
     * @brief Subscription to the stream has expired.
     */
    kMixedStreamTaskErrorCodeSubTimeoutByServer = 1093,
    /**
     * @locale zh
     * @brief 合流服务端内部错误。
     */
    /**
     * @locale en
     * @brief Internal server error.
     */
    kMixedStreamTaskErrorCodeInvalidStateByServer = 1094,
    /**
     * @locale zh
     * @brief 合流服务端推 CDN 失败。
     */
    /**
     * @locale en
     * @brief The server failed to push the mixing streams to CDN.
     */
    kMixedStreamTaskErrorCodeAuthenticationByCDN  = 1095,
    /**
     * @locale zh
     * @brief 服务端未知错误。
     */
    /**
     * @locale en
     * @brief Unknown error from server.
     */
    kMixedStreamTaskErrorCodeUnKnownByServer = 1096,
    /**
     * @locale zh
     * @brief 服务端接收信令超时，请检查网络状态并重试。
     */
    /**
     * @locale en
     * @brief Signaling connection timeout error. Please check network status and retry.
     */
    kMixedStreamTaskErrorCodeSignalRequestTimeout = 1097,
    /**
     * @locale zh
     * @brief 图片合流失败。
     */
    /**
     * @locale en
     * @brief Failed to mix images.
     */
    kMixedStreamTaskErrorCodeMixImageFailed = 1098,
    /**
     * @locale zh
     * @hidden internal use only
     * @brief 缓存未同步。
     */
    /**
     * @locale en
     * @hidden internal use only
     * @brief The cache is not synchronized.
     */
    kMixedStreamTaskErrorCodeStreamSyncWorse = 1099,
    /**
     * @locale zh
     * @brief 发布 WTN 流失败
     */
    /**
     * @locale en
     * @brief Failed to push the WTN stream.
     */
    kMixedStreamTaskErrorCodePushWTNFailed = 1195,
    /**
     * @locale zh
     * @hidden for internal use only
     */
    /**
     * @locale en
     * @hidden for internal use only
     */
    kMixedStreamTaskErrorCodeMax = 1199,
};

/**
 * @locale zh
 * @type errorcode
 * @brief 单流转推任务错误码
 */
/**
 * @locale en
 * @type errorcode
 * @brief Single stream errors.
 */
enum SingleStreamTaskErrorCode {
    /**
     * @locale zh
     * @brief 推流成功。
     */
    /**
     * @locale en
     * @brief Everything is OK.
     */
    kSingleStreamTaskErrorCodeOK = 0,
    /**
     * @locale zh
     * @hidden currently not available
     * @brief 预留错误码，未启用
     */
    /**
     * @locale en
     * @hidden currently not available
     * @brief Reserved error code, not accessible.
     */
    kSingleStreamTaskErrorCodeBase= 1090,
    /**
     * @locale zh
     * @brief 服务端未知错误。
     */
    /**
     * @locale en
     * @brief Unknown error from server.
     */
    kSingleStreamTaskErrorCodeUnKnownByServer = 1091,
    /**
     * @locale zh
     * @brief 任务处理超时，请检查网络状态并重试。
     */
    /**
     * @locale en
     * @brief Request timed out. Please check network status and retry.
     */
    kSingleStreamTaskErrorCodeSignalRequestTimeout = 1092,
    /**
     * @locale zh
     * @brief 服务端检测任务参数不合法
     */
    /**
     * @locale en 
     * @brief server detects that the task parameters are invalid
     */
    kSingleStreamTaskErrorCodeInvalidParamByServer = 1093,
    /**
     * @locale zh
     * @brief 转推任务在目标房间的用户ID被踢出目标房间
     */
    /**
     * @locale en
     * @brief In the retweet task, the user ID in the target room has been kicked out of the target room.
     */
    kSingleStreamTaskErrorRemoteKicked = 1094,
    /**
     * @locale zh
     * @brief 转推任务加入目标房间失败
     */
    /**
     * @locale en
     * @brief he retweet task failed to join the target room.
     */
    kSingleStreamTaskErrorCodeJoinDestRoomFailed = 1095,
    /**
     * @locale zh
     * @brief 转推任务在源房间拉流超时
     */
    /**
     * @locale en
     * @brief The stream pulling for the retweet task in the source room timed out.
     */
    kSingleStreamTaskErrorCodeReceiveSrcStreamTimeout = 1096,
    /**
     * @locale zh
     * @brief 音视频编码转推任务不支持
     */
    /**
     * @locale en
     * @brief Audio and video encoding retweet tasks are not supported
     */
    kSingleStreamTaskErrorCodeNotSurportCodec = 1097,
};

/**
 * @locale zh
 * @hidden for internal use only
 * @type keytype
 * @brief 合流类型。
 */
/**
 * @locale en
 * @hidden for internal use only
 * @type keytype
 * @brief Stream mixing type.
 */
enum MixedStreamType {
    /**
     * @locale zh
     * @brief 服务端合流
     */
    /**
     * @locale en
     * @brief Mix media streams on the server side.
     */
    kMixedStreamTypeByServer = 0,
    /**
     * @locale zh
     * @brief 端云一体合流。SDK 智能决策在客户端或服务端完成合流。 <br>
     *        使用前，请联系技术支持同学开通，否则不生效。
     */
    /**
     * @locale en
     * @brief Intelligent stream mixing. The SDK will intelligently decide that a stream mixing task would be done on the client or the server. <br>
     *        Before using the option, please contact the technical support. Otherwise, the option does not take effect.
     */
    kMixedStreamTypeByClient = 1,
};

/**
 * @locale zh
 * @type keytype
 * @brief 任务类型
 */
/**
 * @locale en
 * @type keytype
 * @brief Task type
 */
enum MixedStreamPushTargetType {
    /**
     * @locale zh
     * @brief 推到 CDN
     */
    /**
     * @locale en
     * @brief Push to CDN.
     */
    kMixedStreamPushTargetTypeToCDN = 0,
    /**
     * @locale zh
     * @brief WTN 流
     */
    /**
     * @locale en
     * @brief WTN stream
     */
    kMixedStreamPushTargetTypeToWTN = 1,
};

/**
 * @locale zh
 * @type keytype
 * @brief AAC 编码规格。
 */
/**
 * @locale en
 * @type keytype
 * @brief Advanced Audio Coding (AAC) profile.
 */
enum MixedStreamAudioProfile {
    /**
     * @locale zh
     * @brief AAC-LC 规格，默认值。
     */
    /**
     * @locale en
     * @brief (Default) AAC Low-Complexity profile (AAC-LC).
     */
    kMixedStreamAudioProfileLC = 0,
    /**
     * @locale zh
     * @brief HE-AAC v1 规格。
     */
    /**
     * @locale en
     * @brief HE-AAC v1 profile (AAC LC with SBR).
     */
    kMixedStreamAudioProfileHEv1 = 1,
    /**
     * @locale zh
     * @brief HE-AAC v2 规格。
     */
    /**
     * @locale en
     * @brief HE-AAC v2 profile (AAC LC with SBR and Parametric Stereo).
     */
    kMixedStreamAudioProfileHEv2 = 2,
};

/**
 * @locale zh
 * @hidden
 * @brief 遗漏的音符
 */
 /**
 * @locale en
 * @hidden
 * @brief notes missing
 */
enum MixedStreamSyncStrategy {
    /**
     * @locale zh
     * @brief 不使用同步策略
     */
    /**
     * @locale en
     * @brief Does not use synchronization strategy
     */
    kMixedStreamSyncStrategyNoSync = 0,
    /**
     * @locale zh
     * @brief 使用音频精准同步策略
     */
     /**
     * @locale en
     * @brief Uses audio precision synchronization strategy
     */
    kMixedStreamSyncStrategyAudioPreciseSync = 1,
    /**
     * @locale zh
     * @brief 使用单通模式同步策略
     */
     /**
     * @locale en
     * @brief Uses a unison mode synchronization strategy
     */
    kMixedStreamSyncStrategySimplexModeSync = 2,
};

/**
 * @locale zh
 * @type keytype
 * @brief 音频编码格式。
 */
/**
 * @locale en
 * @type keytype
 * @brief The audio codec.
 */
enum MixedStreamAudioCodecType {
    /**
     * @locale zh
     * @brief AAC 格式。
     */
    /**
     * @locale en
     * @brief AAC format.
     */
    kMixedStreamAudioCodecTypeAAC = 0,
};

/**
 * @locale zh
 * @type keytype
 * @brief 服务端合流转推 SEI 内容。
 */
/**
 * @locale en
 * @type keytype
 * @brief Content of SEI sent while pushing mixed stream to CDN.
 */
enum MixedStreamSEIContentMode {
    /**
     * @locale zh
     * @brief 视频流中包含全部的 SEI 信息。默认设置。
     */
    /**
     * @locale en
     * @brief The video stream contains all the SEI information. Default value.
     */
    kMixedStreamSEIContentModeDefault = 0,
    /**
     * @locale zh
     * @brief 随非关键帧传输的 SEI 数据中仅包含音量信息。 <br>
     *        当设置 `MixedStreamControlConfig.enable_volume_indication` 为 True 时，此参数设置生效。
     */
    /**
     * @locale en
     * @brief The SEI data transmitted with non-key frames contains volume information only. <br>
     *        This parameter only takes effect after you set `MixedStreamControlConfig.enable_volume_indication` to "True".
     */
    kMixedStreamSEIContentModeEnableVolumeIndication = 1,
};

/**
 * @locale zh
 * @type keytype
 * @brief 视频编码格式。
 */
/**
 * @locale en
 * @type keytype
 * @brief The video codec.
 */
enum MixedStreamVideoCodecType {
    /**
     * @locale zh
     * @brief H.264 格式，默认值。
     */
    /**
     * @locale en
     * @brief (Default) H.264 format.
     */
    kMixedStreamVideoCodecTypeH264 = 0,
    /**
     * @locale zh
     * @brief ByteVC1 格式。
     */
    /**
     * @locale en
     * @brief ByteVC1 format.
     */
    kMixedStreamVideoCodecTypeByteVC1 = 1,
};

/**
 * @locale zh
 * @type keytype
 * @brief 图片或视频流的缩放模式。
 */
/**
 * @locale en
 * @type keytype
 * @brief The render mode.
 */
enum MixedStreamRenderMode {
    /**
     * @locale zh
     * @brief 视窗填满优先，默认值。 <br>
     *        视频尺寸等比缩放，直至视窗被填满。当视频尺寸与显示窗口尺寸不一致时，多出的视频将被截掉。
     */
    /**
     * @locale en
     * @brief (Default) Fill and Crop. <br>
     *        The video frame is scaled with fixed aspect ratio, until it completely fills the canvas. The region of the video exceeding the canvas will be cropped.
     */
    kMixedStreamRenderModeHidden = 1,
    /**
     * @locale zh
     * @brief 视频帧内容全部显示优先。 <br>
     *        视频尺寸等比缩放，优先保证视频内容全部显示。当视频尺寸与显示窗口尺寸不一致时，会把窗口未被填满的区域填充成背景颜色。
     */
    /**
     * @locale en
     * @brief Fit. <br>
     *        The video frame is scaled with fixed aspect ratio, until it fits just within the canvas. Other part of the canvas is filled with the background color.
     */
    kMixedStreamRenderModeFit = 2,
    /**
     * @locale zh
     * @brief 视频帧自适应画布。 <br>
     *        视频尺寸非等比例缩放，把窗口充满。在此过程中，视频帧的长宽比例可能会发生变化。
     */
    /**
     * @locale en
     * @brief Fill the canvas. <br>
     *        The video frame is scaled to fill the canvas. During the process, the aspect ratio may change.
     */
    kMixedStreamRenderModeAdaptive = 3,
};

/**
 * @locale zh
 * @type keytype
 * @brief 合流输出媒体类型。
 */
/**
 * @locale en
 * @type keytype
 * @brief The media type of the mixing stream.
 */
enum MixedStreamMediaType {
    /**
     * @locale zh
     * @brief 包含音频和视频
     */
    /**
     * @locale en
     * @brief Audio and video
     */
    kMixedStreamMediaTypeAudioAndVideo = 0,
    /**
     * @locale zh
     * @brief 只包含音频
     */
    /**
     * @locale en
     * @brief Audio only
     */
    kMixedStreamMediaTypeAudioOnly = 1,
    /**
     * @locale zh
     * @hidden currently not available
     * @brief 只包含视频
     */
    /**
     * @locale en
     * @hidden currently not available
     * @brief Video only
     */
    kMixedStreamMediaTypeVideoOnly = 2,
};

/**
 * @locale zh
 * @type keytype
 * @brief 合流布局区域类型，视频区或水印图片区。
 */
/**
 * @locale en
 * @type keytype
 * @brief The contents of the layout section in the mixing stream, including video section and water mark section.
 */
enum MixedStreamLayoutRegionType {
    /**
     * @locale zh
     * @brief 视频。
     */
    /**
     * @locale en
     * @brief Video.
     */
    kMixedStreamLayoutRegionTypeVideoStream = 0,
    /**
     * @locale zh
     * @brief 水印图片。
     */
    /**
     * @locale en
     * @brief Watermark image.
     */
    kMixedStreamLayoutRegionTypeImage = 1,
};

/**
 * @locale zh
 * @type keytype
 * @hidden for internal use only on Windows
 * @brief 客户端合流回调视频格式。 <br>
 *        设置为系统不支持的格式时，自动回调系统默认格式。
 */
/**
 * @locale en
 * @type keytype
 * @hidden for internal use only on Windows
 * @brief The video format for client stream mixing callback. <br>
 *        If the format you set is not adaptable to the system, the format will be set as the default value.
 */
enum MixedStreamClientMixVideoFormat {
    /**
     * @locale zh
     * @brief YUV I420。Android、Windows 默认回调格式。支持系统：Android、Windows。
     */
    /**
     * @locale en
     * @brief YUV I420 format. The default format for Android and Windows. Supported system: Android, Windows.
     */
    kMixedStreamClientMixVideoFormatI420 = 0,
    /**
     * @locale zh
     * @brief OpenGL GL_TEXTURE_2D 格式纹理。支持系统：安卓。
     */
    /**
     * @locale en
     * @brief OpenGL GL_TEXTURE_2D format. Supported system: Android.
     */
    kMixedStreamClientMixVideoFormatTexture2D = 1,
    /**
     * @locale zh
     * @brief CVPixelBuffer BGRA。iOS 默认回调格式。支持系统: iOS。
     */
    /**
     * @locale en
     * @brief CVPixelBuffer BGRA format. The default format for iOS. support system: iOS.
     */
    kMixedStreamClientMixVideoFormatCVPixelBufferBGRA = 2,
    /**
     * @locale zh
     * @brief YUV NV12。macOS 默认回调格式。支持系统: macOS。
     */
    /**
     * @locale en
     * @brief YUV NV12 format. The default format for macOS. Supported system: macOS.
     */
    kMixedStreamClientMixVideoFormatNV12 = 3,
    /**
     * @locale zh
     * @brief D3D11Texture2D, BGRA。支持系统: windows。
     */
    /**
     * @locale en
     * @brief D3D11Texture2D, BGRA. Supported system: windows.
     */
    kMixedStreamClientMixVideoFormatD3D11Texture2D = 4,
};
/**
 * @locale zh
 * @type keytype
 * @brief 服务端合流转推发起模式。
 */
/**
 * @locale en
 * @type keytype
 * @brief The initiation mode for pushing the stream mixed on the server side to CDN.
 */
enum MixedStreamPushMode {
    /**
     * @locale zh
     * @brief 无用户发布媒体流时，发起合流任务无效。默认设置。 <br>
     *        当有用户发布媒体流时，才能发起合流任务。
     */
    /**
     * @locale en
     * @brief When no user publishes a media stream, a mixing-stream task can't be initiated. Default setting. <br>
     *        When a user publishes at least one media stream, mixing-stream task be initiated.
     */
    kMixedStreamPushModeOnStream = 0,
    /**
     * @locale zh
     * @brief 无用户发布媒体流时，可以使用占位图发起合流任务。 <br>
     *        占位图设置参看 alternate_image_url{@link #MixedStreamLayoutRegionConfig-alternate_image_url} 和 alternate_image_fill_mode{@link #MixedStreamLayoutRegionConfig-alternate_image_fill_mode}
     */
    /**
     * @locale en
     * @brief Even if no user is actively publishing media streams, it is still possible to initiate a stream mixing task using a placeholder image. <br>
     *        See alternate_image_url{@link #MixedStreamLayoutRegionConfig-alternate_image_url} and alternate_image_fill_mode{@link #MixedStreamLayoutRegionConfig-alternate_image_fill_mode} for setting a placeholder image.
     */
    kMixedStreamPushModeOnStartRequest = 1,
};
/**
 * @locale zh
 * @type keytype
 * @brief 服务端合流占位图填充模式。
 */
/**
 * @locale en
 * @type keytype
 * @brief The fill mode of the placeholder image.
 */
enum MixedStreamAlternateImageFillMode {
    /**
     * @locale zh
     * @brief 占位图跟随用户原始视频帧相同的比例缩放。默认设置。
     */
    /**
     * @locale en
     * @brief The placeholder image is scaled with the same aspect ratio as the user's original video frame. Default setting.
     */
    kMixedStreamAlternateImageFillModeFit = 0,
    /**
     * @locale zh
     * @brief 占位图不跟随用户原始视频帧相同的比例缩放，保持图片原有比例。
     */
    /**
     * @locale en
     * @brief The placeholder image is not scaled with the same aspect ratio as the user's original video frame. It maintains the original aspect ratio.
     */
    kMixedStreamAlternateImageFillModeFill = 1,
};
/**
 * @locale zh
 * @type keytype
 * @brief Region 中流的类型属性
 */
/**
 * @locale en
 * @type keytype
 * @brief Stream type in the region
 */
enum MixedStreamVideoType {
    /**
     * @locale zh
     * @brief 主流。包括： <br>
     *        - 由摄像头/麦克风通过内部采集机制，采集到的流
     *        - 通过自定义采集，采集到的流。
     */
    /**
     * @locale en
     * @brief Mainstream, including: <br>
     *       - Video/audio captured by the the camera/microphone using internal capturing;
     *       - Video/audio captured by custom method.
     */
    kMixedStreamVideoTypeMain = 0,
    /**
     * @locale zh
     * @brief 屏幕流。
     */
    /**
     * @locale en
     * @brief Screen-sharing stream.
     */
    kMixedStreamVideoTypeScreen = 1,
};

/**
 * @locale zh
 * @type keytype
 * @brief WTN 流任务详情 <br>
 *
 */
/**
 * @locale en
 * @type keytype
 * @brief Detailed information of the stream mixing task. <br>
 */
typedef struct MixedStreamTaskInfo {
    /**
     * @locale zh
     * @brief 任务 ID <br>
     * 对于 WTN 流任务，该值代表 WTN 流 ID。你可以通过该 ID，指定需要订阅的 WTN 流。
     */
    /**
     * @locale en
     * @brief Task ID <br>
     *        For WTN stream task, this value represents the WTN stream ID. You can subscribe a WTN stream by specifying the ID.
     */
    const char* task_id;
    /**
     * @locale zh
     * @brief 任务类型，合流转推 CDN 还是 WTN 流。
     */
    /**
     * @locale en
     * @brief Task type: Pushing a mixing stream to CDN or starting a WTN stream.
     */
    MixedStreamPushTargetType push_target_type;
} MixedStreamInfo;

/**
 * @locale zh
 * @type keytype
 * @brief 音频合流参数。 <br>
 *         值不合法或未设置时，自动使用默认值。
 */
/**
 * @locale en
 * @type keytype
 * @brief Audio mix stream configurations. <br>
 *         With invalid or empty input, the configurations will be set as the default values.
 */
typedef struct MixedStreamAudioConfig {
    /**
     * @locale zh
     * @brief 音频采样率，单位 Hz。可取 32000 Hz、44100 Hz、48000 Hz，默认值为 48000 Hz。建议设置。
     */
    /**
     * @locale en
     * @brief The sample rate (Hz). Supported sample rates: 32,00 Hz, 44,100 Hz, 48,000 Hz. Defaults to 48,000 Hz. It's recommended to be set.
     */
    int32_t sample_rate = 48000;
    /**
     * @locale zh
     * @brief 音频声道数。可取 1（单声道）、2（双声道），默认值为 2。建议设置。
     */
    /**
     * @locale en
     * @brief The number of channels. Supported channels: 1 (single channel), 2 (dual channel).  Defaults to 2. It's recommended to be set.
     */
    int32_t channels = 2;
    /**
     * @locale zh
     * @brief 音频码率，单位 Kbps。可取范围 [32, 192]，默认值为 64 Kbps。建议设置。
     */
    /**
     * @locale en
     * @brief The bitrate (Kbps) in range of [32, 192]. Defaults to 64 Kbps. It's recommended to be set.
     */
    int32_t bitrate = 64;
    /**
     * @locale zh
     * @brief AAC 编码规格，参看 MixedStreamAudioProfile{@link #MixedStreamAudioProfile}。默认值为 `0` 代表 AAC-LC。
     */
    /**
     * @locale en
     * @brief AAC profile. See MixedStreamAudioProfile{@link #MixedStreamAudioProfile}. It defaults to `0`, indicating AAC-LC.
     */
    MixedStreamAudioProfile audio_profile = MixedStreamAudioProfile::kMixedStreamAudioProfileLC;
    /**
     * @locale zh
     * @brief 音频编码格式，参看 MixedStreamAudioCodecType{@link #MixedStreamAudioCodecType}。当前只有一个值为 `0`，代表 AAC。 <br>
     *        WTN 流任务不支持设置本参数。
     */
    /**
     * @locale en
     * @brief AAC profile. See MixedStreamAudioCodecType{@link #MixedStreamAudioCodecType}. Currently, the only value `0` indicates AAC. <br>
     *        This parameter is not supported for WTN stream tasks.
     */
    MixedStreamAudioCodecType audio_codec = MixedStreamAudioCodecType::kMixedStreamAudioCodecTypeAAC;
} MixedStreamAudioConfig;

#define MIXED_STREAM_VIDEO_DEFAULT_WIDTH 360
#define MIXED_STREAM_DEFAULT_VIDEO_HEIGHT 640
/**
 * @locale zh
 * @type keytype
 * @brief 视频合流配置参数。 <br>
 *        值不合法或未设置时，自动使用默认值。
 */
/**
 * @locale en
 * @type keytype
 * @brief Video mix stream configurations. <br>
 *        With invalid or empty input, the configurations will be set as the default values.
 */
typedef struct MixedStreamVideoConfig {
    /**
     * @locale zh
     * @brief 合流视频宽度。单位为 px，范围为 [2, 1920]，必须是偶数。默认值为 360 px。建议设置。 <br>
     *        设置值为非偶数时，自动向上取偶数。
     */
    /**
     * @locale en
     * @brief The width (pixels) to be set. The range is [2, 1920], and must be an even number. The default value is 360 pixels. It's recommended to be set. <br>
     *        If an odd number is set, the width will be adjusted to the next larger even number.
     */
    int32_t width = MIXED_STREAM_VIDEO_DEFAULT_WIDTH;
    /**
     * @locale zh
     * @brief 合流视频高度。单位为 px，范围为 [2, 1920]，必须是偶数。默认值为 640 px。建议设置。 <br>
     *        设置值为非偶数时，自动向上取偶数。
     */
    /**
     * @locale en
     * @brief The height (pixels) to be set. The range is [2, 1920], and must be an even number. The default value is 640 pixels. It's recommended to be set. <br>
     *        If an odd number is set, the height will be adjusted to the next larger even number.
     */
    int32_t height = MIXED_STREAM_DEFAULT_VIDEO_HEIGHT;
    /**
     * @locale zh
     * @brief 合流视频帧率。单位为 FPS，取值范围为 [1,60]，默认值为 15 FPS。建议设置。
     */
    /**
     * @locale en
     * @brief The frame rate (FPS) in range of [1, 60]. The default value is 15 FPS. It's recommended to be set.
     */
    int32_t fps = 15;
    /**
     * @locale zh
     * @brief 视频 I 帧时间间隔。单位为秒，取值范围为 [1, 5]，默认值为 2 秒。建议设置。 <br>
     *        WTN 流任务不支持设置本参数。 <br>
     *        本参数不支持过程中更新。
     */
    /**
     * @locale en
     * @brief The time interval between I-frames (second) in range of [1, 5]. The default value is 2 seconds. It's recommended to be set. <br>
     *        This parameter is not supported for WTN stream tasks. <br>
     *        This parameter cannot be updated during the task.
     */
    int32_t gop = 2;
    /**
     * @locale zh
     * @brief 合流视频码率。单位为 Kbps，取值范围为 [1,10000]，默认值为 `500`。建议设置。
     */
    /**
     * @locale en
     * @brief The bitrate (Kbps) in range of [1, 10000]. The default value is `500`. It's recommended to be set.
     */
    int32_t bitrate = 500;
    /**
     * @locale zh
     * @brief 视频编码格式，参看 MixedStreamVideoCodecType{@link #MixedStreamVideoCodecType}。默认值为 `0`。建议设置。 <br>
     *        WTN 流的视频编码格式只能为设置为 `0`，H264。 <br>
     *        本参数不支持过程中更新。
     */
    /**
     * @locale en
     * @brief The video codec. See MixedStreamVideoCodecType{@link #MixedStreamVideoCodecType}. The default value is `0`. It's recommended to be set. <br>
     *        The video codec of WTN stream can only be set as `0`, indicating H264. <br>
     *        This parameter cannot be updated during the task.
     */
    MixedStreamVideoCodecType video_codec = MixedStreamVideoCodecType::kMixedStreamVideoCodecTypeH264;
    /**
      * @locale zh
      * @brief 是否在服务端合流转推中开启 B 帧。 <br>
      *        - true: 是
      *        - false: 否
      */
     /**
      * @locale en
      * @brief Whether to use B frames when pushing the mixing streams to CDN. <br>
      *         - True: Yes
      *         - False: No
      */
    bool enable_bframe = false;
} MixedStreamVideoConfig;

/**
 * @locale zh
 * @hidden for internal use only on Windows
 * @type keytype
 * @brief 客户端合流参数。
 */
/**
 * @locale en
 * @hidden for internal use only on Windows
 * @type keytype
 * @brief Client mixing parameters.
 */
typedef struct ClientMixedStreamConfig {
    /**
     * @locale zh
     * @brief 客户端合流是否使用混音，默认为 true。
     */
    /**
     * @locale en
     * @brief Whether to use audio mixing. Default is true.
     */
    bool use_audio_mixer = true;
    /**
     * @locale zh
     * @brief 客户端合流回调视频格式，参看 MixedStreamClientMixVideoFormat{@link #MixedStreamClientMixVideoFormat}。
     */
    /**
     * @locale en
     * @brief The video format to be set. See MixedStreamClientMixVideoFormat{@link #MixedStreamClientMixVideoFormat}.
     */
    MixedStreamClientMixVideoFormat video_format;
} ClientMixedStreamConfig;

/**
 * @locale zh
 * @hidden for internal use only
 * @type keytype
 * @brief 转推直播单通同步参数。
 */
/**
 * @locale en
 * @hidden for internal use only
 * @type keytype
 * @brief Parameters of simplex mode and synchronization when pushing to CDN.
 */
typedef struct MixedStreamSyncControlConfig {
    /**
     * @locale zh
     * @brief 在进行同步处理时，缓存音视频流的最大长度。单位为毫秒。默认值为 2000。
     * @note 参与转推直播的这些媒体流延迟越高，应该将此值设置的越大。但此值越大，因缓存媒体流造成的内存占用也会更大。推荐值为 `2000`。
     */
    /**
     * @locale en
     * @brief The max length of the cached stream in milliseconds. 2000 by default.
     * @note Set the value based on the stall of the media streams. Higher the value, bigger the memory usage. The recommended value is `2000`.
     */
    int32_t max_cache_time_ms = 2000;
    /**
     * @locale zh
     * @brief 是否在转推直播时，启用单通模式。默认为 false，不启用。 <br>
     *        启用单通模式时，RTC SDK 会对指定的多个用户的媒体流进行同步处理，再合流后推送到 CDN，但基准流所属用户不会播放来自其他用户的媒体流。你需要设定以下参数。 <br>
     *        非单通模式时，RTC SDK 不会对媒体流进行同步处理，只是简单合流后推送到 CDN。以下参数设定无效。
     */
    /**
     * @locale en
     * @brief Whether to enable simplex mode during pushing to CDN. False(duplex mode) by default. <br>
     *        When simplex mode is enabled, RTC SDK synchronizes and transcodes the media streams from the local user and several remote users, and pushing the mixed stream to CDN. `syncBaseUser` does not play the streams from the remote users. You must set the following parameters. <br>
     *        When duplex mode is enabled, RTC SDK transcodes the media streams from the local user and several remote users without synchronization, and pushing the mixed stream to CDN. The following parameters are not effective.
     */
    MixedStreamSyncStrategy sync_strategy = MixedStreamSyncStrategy::kMixedStreamSyncStrategyNoSync;
    /**
     * @locale zh
     * @brief 在进行同步处理时，基准流所属用户的 ID。默认为空。
     */
    /**
     * @locale en
     * @brief User ID of the base stream during syncing. Null by default.
     */
    const char* base_user_id = nullptr;
    /**
     * @locale zh
     * @brief 是否通过 RTC SDK 进行转推直播。默认为 True。 <br>
     *        如果选择 `False`，你会通过 onCacheSyncVideoFrames{@link #IClientMixedStreamObserver#onCacheSyncVideoFrames} 收到同步的帧，你可以使用此视频帧，自行实现合流转推。
     */
    /**
     * @locale en
     * @brief Whether to use RTC SDK to push to CDN. True by default. <br>
     *        If `False`, you can get the media frames by onCacheSyncVideoFrames{@link #IClientMixedStreamObserver#onCacheSyncVideoFrames} and manually push them to CDN.
     */
    bool video_need_sdk_mix = true;
} MixedStreamSyncControlConfig;

/**
 * @locale zh
 * @type keytype
 * @brief 图片合流水印图片分辨率。
 */
/**
 * @locale en
 * @type keytype
 * @brief Resolution of the watermark image.
 */
typedef struct MixedStreamLayoutRegionImageWaterMarkConfig {
    /**
     * @locale zh
     * @brief 原始图片的宽度，单位为 px。
     */
    /**
     * @locale en
     * @brief Width of the original image in px.
     */
    int image_width = 0;
    /**
     * @locale zh
     * @brief 原始图片的高度，单位为 px。
     */
    /**
     * @locale en
     * @brief Height of the original image in px.
     */
    int image_height = 0;
} MixedStreamLayoutRegionImageWaterMarkConfig;

/**
 * @locale zh
 * @type keytype
 * @brief 推流 CDN 的空间音频参数。
 */
/**
 * @locale en
 * @type keytype
 * @brief Spatial audio config when pushing to CDN.
 */
typedef struct MixedStreamSpatialAudioConfig {
    /**
     * @locale zh
     * @brief 是否开启推流 CDN 时的空间音频效果。
     * @note 当你启用此效果时，你需要设定推流中各个 MixedStreamLayoutRegionConfig{@link #MixedStreamLayoutRegionConfig} 的 `spatial_position` 值，实现空间音频效果。
     */
    /**
     * @locale en
     * @brief Whether to enable the spatial audio effect when pushing to CDN.
     * @note when you enable the feature, set the `spatial_position` of each MixedStreamLayoutRegionConfig{@link #MixedStreamLayoutRegionConfig} for spatial audio effect.
     */
    bool enable_spatial_render;
    /**
     * @locale zh
     * @brief 听众的空间位置。参看 Position{@link #Position}。 <br>
     *        听众指收听来自 CDN 的音频流的用户。 <br>
     *        WTN 流任务不支持设置本参数。
     */
    /**
     * @locale en
     * @brief The spatial position of the audience. See Position{@link #Position}. <br>
     *        The audience is the users who receive the audio stream from CDN. <br>
     *        This parameter is not supported for WTN stream tasks.
     */
    Position audience_spatial_position;
    /**
     * @locale zh
     * @brief 听众的空间朝向。参看 HumanOrientation{@link #HumanOrientation}。 <br>
     *        听众指收听来自 CDN 的音频流的用户。
     */
    /**
     * @locale en
     * @brief The orientation of the audience. See HumanOrientation{@link #HumanOrientation}. <br>
     *        The audience is the users who receive the audio stream from CDN.
     */
    HumanOrientation audience_spatial_orientation;
} MixedStreamSpatialAudioConfig;

/**
 * @locale zh
 * @type keytype
 * @brief 服务端合流控制参数
 */
/**
 * @locale en
 * @type keytype
 * @brief Configurations to be set while mixing streams on the server side.
 */
typedef struct MixedStreamControlConfig {
    /**
     * @locale zh
     * @valid since 3.56
     * @brief 是否开启单独发送声音提示 SEI 的功能： <br>
     *        - true：开启；
     *        - false：（默认）关闭。
     *        开启后，你可以通过 `MixedStreamControlConfig.sei_content_mode` 控制 SEI 的内容是否只携带声音信息。
     */
    /**
     * @locale en
     * @valid since 3.56
     * @brief Sets whether to enable the function of separately sending sound indication SEI: <br>
     *        - true: Enable.
     *        - false: (Default) Disable.
     *        After setting this parameter to "true", you can choose whether to send sound indication SEI only through `MixedStreamControlConfig.sei_content_mode`.
     */
    bool enable_volume_indication = false;
    /**
     * @locale zh
     * @valid since 3.56
     * @brief 声音提示间隔，单位为秒，取值范围为 [0.3,+∞)，默认值为 2。 <br>
     *        此值仅取整百毫秒。若传入两位及以上小数，则四舍五入取第一位小数的值。例如，若传入 0.36，则取 0.4。
     */
    /**
     * @locale en
     * @valid since 3.56
     * @brief Sound indication interval in s. The range is [0.3,+∞). The default value is 2. <br>
     *        If a decimal with two or more decimal places is passed in, it will be rounded to the value of the first decimal place. For example, if you pass in 0.36, it will be automatically adjusted to 0.4.
     */
    float volume_indication_interval = 2.0f;
    /**
     * @locale zh
     * @valid since 3.56
     * @brief 有效音量大小，取值范围为 [0, 255]，默认值为 0。 <br>
     *        超出取值范围则自动调整为默认值，即 0。
     */
    /**
     * @locale en
     * @valid since 3.56
     * @brief Effective volume within the range of [0, 255]. The default value is 0. <br>
     *        If the value is set beyond the range, it will be automatically adjusted to the default value 0.
     */
    int talk_volume = 0;
    /**
     * @locale zh
     * @valid since 3.56
     * @brief 声音信息 SEI 是否包含音量值： <br>
     *        - true：是。
     *        - false：（默认）否。
     */
    /**
     * @locale en
     * @valid since 3.56
     * @brief Sets whether volume value is contained in the sound indication SEI: <br>
     *        - true: Yes.
     *        - false: (Default) No.
     */
    bool is_add_volume_value = false;
    /**
     * @locale zh
     * @valid since 3.56
     * @brief 设置 SEI 内容。参看 MixedStreamSEIContentMode{@link #MixedStreamSEIContentMode}。 <br>
     * 默认值为 `0`，视频流中包含全部的 SEI 信息
     */
    /**
     * @locale en
     * @valid since 3.56
     * @brief Sets SEI content. See MixedStreamSEIContentMode{@link #MixedStreamSEIContentMode}. <br>
     * The default value is `0`, indicating that the SEI in the video stream is included.
     */
    MixedStreamSEIContentMode sei_content_mode = kMixedStreamSEIContentModeDefault;
    /**
     * @locale zh
     * @valid since 3.56
     * @brief SEI 信息的 payload type。 <br>
     *        默认值为 `100`，只支持设置 `5` 和 `100`。 <br>
     *        在转推直播的过程中，该参数不支持变更。
     */
    /**
     * @locale en
     * @valid since 3.56
     * @brief SEI payload type. <br>
     *        The default value is `100`, and the value supported is `5` and `100`. <br>
     *        During the process of pushing streams to CDN, you cannot change the parameter.
     */
    int sei_payload_type = 100;
    /**
     * @locale zh
     * @valid since 3.56
     * @brief SEI 信息的 payload UUID。
     * @note PayloadType 为 `5` 时，必须填写 PayloadUUID，否则会收到错误回调，错误码为 1091。 <br>
     *         PayloadType 不是 `5` 时，不需要填写 PayloadUUID，如果填写会被后端忽略。 <br>
     *         该参数长度需为 32 位，否则会收到错误码为 1091 的回调。 <br>
     *         该参数每个字符的范围需为 [0, 9] [a, f] [A, F] <br>
     *         该参数不应带有`-`字符，如系统自动生成的 UUID 中带有`-`，则应删去。 <br>
     *         在转推直播的过程中，该参数不支持变更。
     */
     /**
     * @locale en
     * @valid since 3.56
     * @brief SEI payload UUID.
     * @note When PayloadType is `5`, you must set PayloadUUID, or you will receive a callback indicating parameter error. The error code is 1091. <br>
     *        When PayloadType is not `5`, it is not required to set PayloadUUID. If filled, it will be ignored by the backend. <br>
     *        The length of PayloadUUID should be 32 bits, or you will receive an error code of 1091. <br>
     *        Each character of the parameter should be within the range of [0, 9] [a, f] [A, F]. <br>
     *        The PayloadUUID should not contain `-`. If the automatically generated UUID contains `-`, you should delete it. <br>
     *        During the process of pushing streams to CDN, you cannot change the parameter.
     */
     const char* sei_payload_uuid = nullptr;
     /**
      * @locale zh
      * @valid since 3.57
      * @brief 设置合流推到 CDN 时输出的媒体流类型。参看 MixedStreamMediaType{@link #MixedStreamMediaType}。 <br>
      *        默认输出音视频流。支持输出纯音频流，但暂不支持输出纯视频流。
      */
     /**
      * @locale en
      * @brief Sets the type of media stream pushed to CDN after being mixed. MixedStreamMediaType{@link #MixedStreamMediaType}. <br>
      *        The default value is 0，which means pushing both audio and video. <br>
      */
     MixedStreamMediaType media_type = MixedStreamMediaType::kMixedStreamMediaTypeAudioAndVideo;
     /**
      * @locale zh
      * @valid since 3.57
      * @brief 设置是否在没有用户发布流的情况下发起转推直播，默认不允许。具体参看 MixedStreamPushMode{@link #MixedStreamPushMode}。 <br>
      *        该参数在发起合流任务后的转推直播过程中不支持动态变更。
      */
     /**
      * @locale en
      * @brief Sets whether to initiate a stream mixing task in the absence of any users publishing streams. It defaults to be dependent on a user actively publishing media streams. See MixedStreamPushMode{@link #MixedStreamPushMode}. <br>
      *        Once the stream mixing task is initiated, this parameter can not be updated any more.
      */
     MixedStreamPushMode push_stream_mode = MixedStreamPushMode::kMixedStreamPushModeOnStream;
} MixedStreamControlConfig;

/**
 * @locale zh
 * @type keytype
 * @brief WTN 流的布局模式
 */
/**
 * @locale en
 * @type keytype
 * @brief Layout mode of the WTN stream
 */
enum StreamLayoutMode {
    /**
     * @locale zh
     * @brief 自动布局
     */
    /**
     * @locale en
     * @brief Auto
     */
    kStreamLayoutModeAuto = 0,
    /**
     * @locale zh
     * @brief 自定义
     */
    /**
     * @locale en
     * @brief Manual
     */
    kStreamLayoutModeCustom = 2
};
/**
 * @locale zh
 * @type keytype
 * @brief WTN 流的补帧模式
 */
/**
 * @locale en
 * @type keytype
 * @brief Interpolation mode of the WTN stream
 */
enum InterpolationMode {
    /**
     * @locale zh
     * @type keytype
     * @brief 补最后一帧
     */
    /**
     * @locale en
     * @type keytype
     * @brief Fill with the last frame.
     */
    kInterpolationModeLastFrameFill = 0,
    /**
     * @locale zh
     * @type keytype
     * @brief 补背景图片
     */
    /**
     * @locale en
     * @type keytype
     * @brief Fill with the background image.
     */
    kInterpolationModeBackgroundImageFill = 1
};
/**
 * @locale zh
 * @type keytype
 * @brief WTN 流视频裁剪配置
 */
/**
 * @locale en
 * @type keytype
 * @brief Configurations on how to crop the WTN stream
 */
typedef struct SourceCrop {
    /**
     * @locale zh
     * @brief 裁剪后得到的视频帧左上角横坐标相对于裁剪前整体画面的归一化比例，取值范围[0.0, 1.0)。 <br>
     * 默认值为 0.0
     */
    /**
     * @locale en
     * @brief The normalized horizontal coordinate value of the top left vertex of the cropped image to width of the original image, ranging within [0.0, 1.0). <br>
     * Default value is 0.0.
     */
    float location_x = 0.0;
    /**
     * @locale zh
     * @brief 裁剪后得到的视频帧左上角纵坐标相对于裁剪前整体画面的归一化比例，取值范围[0.0, 1.0) <br>
     * 默认值为 0.0
     */
    /**
     * @locale en
     * @brief The normalized vertical coordinate value of the top left vertex of the cropped image to height of the original image, ranging within [0.0, 1.0). <br>
     * Default value is 0.0.
     */
    float location_y = 0.0;
    /**
     * @locale zh
     * @brief 裁剪后得到的视频帧宽度相对于裁剪前整体画面的归一化比例，取值范围(0.0, 1.0] <br>
     * 默认值为 1.0
     */
    /**
     * @locale en
     * @brief The normalized ratio of the width of the cropped image to that of the original image, ranging within [0.0, 1.0). <br>
     * Default value is 1.0.
     */
    float width_proportion = 1.0;
    /**
     * @locale zh
     * @brief 裁剪后得到的视频帧高度相对于裁剪前整体画面的归一化比例，取值范围(0.0, 1.0] <br>
     * 默认值为 1.0
     */
    /**
     * @locale en
     * @brief The normalized ratio of the height of the cropped image to that of the original image, ranging within [0.0, 1.0). <br>
     * Default value is 1.0.
     */
    float height_proportion = 1.0;
} SourceCrop;
/**
 * @locale zh
 * @type keytype
 * @brief 单个图片或视频流在合流中的布局信息。 <br>
 *        开启合流功能后，在多路图片或视频流合流时，你可以设置其中一路流在合流中的预设布局信息。
 */
/**
 * @locale en
 * @type keytype
 * @brief Layout information for one of the video streams to be mixed. <br>
 *        After starting to push streams to CDN and mixing multiple video streams, you can set the layout information for each of them.
 */
typedef struct MixedStreamLayoutRegionConfig {
    /**
     * @locale zh
     * @brief 合流用户的 ID。建议设置。
     */
    /**
     * @locale en
     * @brief The ID of the user who publishes the video stream. It's recommended to be set.
     */
    const char* user_id = nullptr;
   /**
     * @locale zh
     * @brief 图片或视频流所在房间的房间 ID。建议设置。 <br>
     *        如果此图片或视频流是通过 startForwardStreamToRooms{@link #IRTCRoom#startForwardStreamToRooms} 转发到用户所在房间的媒体流时，你应将房间 ID 设置为用户所在的房间 ID。
     */
    /**
     * @locale en
     * @brief The room ID of the media stream. It's recommended to be set. <br>
     *        If the media stream is the stream forwarded by startForwardStreamToRooms{@link #IRTCRoom#startForwardStreamToRooms}, you must set the roomID to the room ID of the target room.
     */
    const char* room_id = nullptr;
    /**
     * @locale zh
     * @brief 单个用户画面左上角在整个画布坐标系中的 X 坐标（pixel），即以画布左上角为原点，用户画面左上角相对于原点的横向位移。 <br>
     *        取值范围为 [0, 整体画布宽度)。默认值为 0。
     */
    /**
     * @locale en
     * @brief The X-coordinate in pixels of the upper-left corner of the user's frame in the entire canvas coordinate system, where the origin is at the upper-left corner of the canvas. It represents the horizontal displacement of the upper-left corner of the user's frame relative to the origin. <br>
     *        The value range is [0, the width of the canvas). The default value is 0.
     */
    int location_x = 0;
    /**
     * @locale zh
     * @brief 单个用户画面左上角在整个画布坐标系中的 Y 坐标（pixel），即以画布左上角为原点，用户画面左上角相对于原点的纵向位移。 <br>
     *        取值范围为 [0, 整体画布高度)。默认值为 0。
     */
    /**
     * @locale en
     * @brief The Y-coordinate in pixels of the upper-left corner of the user's frame in the entire canvas coordinate system, where the origin is at the upper-left corner of the canvas. It represents the vertical displacement of the upper-left corner of the user's frame relative to the origin. <br>
     *        The value range is [0, the height of the canvas). The default value is 0.
     */
    int location_y = 0;
    /**
     * @locale zh
     * @brief 单个用户画面的宽度。取值范围为 [0, 整体画布宽度]，默认值为 360。
     */
    /**
     * @locale en
     * @brief The width of the user's frame in pixels. The value range is [0, the width of the canvas]. The default value is 360.
     */
    int width = MIXED_STREAM_VIDEO_DEFAULT_WIDTH;
    /**
     * @locale zh
     * @brief 单个用户画面的高度。取值范围为 [0, 整体画布高度]，默认值为 640。
     */
    /**
     * @locale en
     * @brief The heigh of the user's frame in pixels. The value range is [0, the height of the canvas]. The default value is 640.
     */
    int height = MIXED_STREAM_DEFAULT_VIDEO_HEIGHT;
    /**
     * @locale zh
     * @brief 透明度，可选范围为 (0.0, 1.0]，0.0 为全透明。默认值为 1.0。
     */
    /**
     * @locale en
     * @brief The opacity in range of (0.0, 1.0]. The lower value, the more transparent. The default value is 1.0.
     */
    float alpha = 1.0f;
    /**
     * @locale zh
     * @brief 圆角半径相对画布宽度的比例。默认值为 `0.0`。 <br>
     *        做范围判定时，首先根据画布的宽高，将 `width`，`height`，和 `corner_radius` 分别转换为像素值：`width_px`，`height_px`，和 `corner_radius_px`。然后判定是否满足 `corner_radius_px < min(width_px/2, height_px/2)`：若满足，则设置成功；若不满足，则将 `corner_radius_px` 设定为 `min(width_px/2, height_px/2)`，然后将 `corner_radius` 设定为 `corner_radius_px` 相对画布宽度的比例值。 <br>
     *        WTN 流任务不支持设置本参数。
     */
    /**
     * @locale en
     * @brief The proportion of the radius to the width of the canvas. `0.0` by default. <br>
     *        After you set the value, `width_px`, `height_px`, and `corner_radius_px` are calculated based on `width`, `height`, `corner_radius`, and the width of the canvas. If `corner_radius_px < min(width_px/2, height_px/2)` is met, the value of `corner_radius` is set valid; if not, `corner_radius_px` is set to `min(width_px/2, height_px/2)`, and `corner_radius` is set to the proportion of `corner_radius_px` to the width of the canvas. <br>
     *        This parameter is not supported for WTN stream tasks.
     */
    float corner_radius = 0;

    /**
     * @locale zh
     * @brief 用户视频布局在画布中的层级。取值范围为 [0 - 100]，0 为底层，值越大越上层。默认值为 0。
     */
    /**
     * @locale en
     * @brief The layer on which the video is rendered. The range is [0, 100]. 0 for the bottom layer, and 100 for the top layer. The default value is 0.
     */
    int32_t z_order = 0;
    /**
     * @locale zh
     * @brief 是否为本地用户： <br>
     *        - true：是
     *        - false:（默认值）否
     *        WTN 流任务不支持设置本参数。
     */
    /**
     * @locale en
     * @brief Whether the source user of the stream is a local user: <br>
     *         - true: Yes
     *         - false: (Default) No
     *        This parameter is not supported for WTN stream tasks.
     */
    bool is_local_user = false;
    /**
     * @locale zh
     * @brief 是否为屏幕流。默认值为 `kMixedStreamVideoTypeMain`，主流。参看 MixedStreamVideoType{@link #MixedStreamVideoType}。
     */
    /**
     * @locale en
     * @brief Whether the stream comes from screen sharing. It defaults to  `kMixedStreamVideoTypeMain` indicating the main stream. See MixedStreamVideoType{@link #MixedStreamVideoType}.
     */
    MixedStreamVideoType stream_type = MixedStreamVideoType::kMixedStreamVideoTypeMain;
    /**
     * @locale zh
     * @brief 合流内容控制。默认值为 `kMediaTypeAudioAndVideo`，参看 MixedStreamMediaType{@link #MixedStreamMediaType}。
     */
    /**
     * @locale en
     * @brief The stream mixing content type. The default value is `kMediaTypeAudioAndVideo`. See MixedStreamMediaType{@link #MixedStreamMediaType}.
     */
    MixedStreamMediaType media_type = MixedStreamMediaType::kMixedStreamMediaTypeAudioAndVideo;
    /**
     * @locale zh
     * @brief 图片或视频流的缩放模式，参看 MixedStreamRenderMode{@link #MixedStreamRenderMode}。默认值为 `1`，视窗填满优先。建议设置。
     */
    /**
     * @locale en
     * @brief The render mode. See MixedStreamRenderMode{@link #MixedStreamRenderMode}. The default value is `1` indicating the video frame is scaled with fixed aspect ratio. It's recommended to be set.
     */
    MixedStreamRenderMode render_mode = MixedStreamRenderMode::kMixedStreamRenderModeHidden;
    /**
     * @locale zh
     * @brief 合流布局区域类型，视频区或水印图片区。参看 MixedStreamLayoutRegionType{@link #MixedStreamLayoutRegionType}。默认值为 `0`，视频。 <br>
     *        WTN 流任务不支持设置本参数。
     */
    /**
     * @locale en
     * @brief  Section type, including video section and water mark section. See MixedStreamLayoutRegionType{@link #MixedStreamLayoutRegionType}. The default value is `0` indicating videos. <br>
     *        This parameter is not supported for WTN stream tasks.
     */
    MixedStreamLayoutRegionType region_content_type = MixedStreamLayoutRegionType::  kMixedStreamLayoutRegionTypeVideoStream;
    /**
     * @locale zh
     * @brief 水印图 RGBA 数据。当 `region_content_type` 为图片类型时需要设置。 <br>
     *        - `kMixedStreamLayoutRegionTypeImage = 1` 时，传入图片 RGBA 数据。
     *        - `kMixedStreamLayoutRegionTypeVideoStream = 0` 时传入空。
     *        WTN 流任务不支持设置本参数。
     */
    /**
     * @locale en
     * @brief The RGBA data of the watermark. It is required when the region_content_type is set to `kMixedStreamLayoutRegionTypeImage`. <br>
     * Put in null when you choose `kMixedStreamLayoutRegionTypeVideoStream` as `region_content_type`. <br>
     *        This parameter is not supported for WTN stream tasks.
     */
    uint8_t* image_water_mark = nullptr;
    /**
     * @locale zh
     * @type keytype
     * @brief 水印图参数。当 `region_content_type` 为图片类型时需要设置。 <br>
     *        - `kMixedStreamLayoutRegionTypeImage = 1` 时，传入图片参数，参看 MixedStreamLayoutRegionImageWaterMarkConfig{@link #MixedStreamLayoutRegionImageWaterMarkConfig}。
     *        - `kMixedStreamLayoutRegionTypeVideoStream = 0` 时传入空。
     *        WTN 流任务不支持设置本参数。
     */
    /**
     * @locale en
     * @type keytype
     * @brief The resolution of the watermark. It is required when the region_content_type is set to `kMixedStreamLayoutRegionTypeImage`. See MixedStreamLayoutRegionImageWaterMarkConfig{@link #MixedStreamLayoutRegionImageWaterMarkConfig}. <br>
     *        Put in null when you choose `kMixedStreamLayoutRegionTypeVideoStream` as `region_content_type`. <br>
     *        This parameter is not supported for WTN stream tasks.
     */
    MixedStreamLayoutRegionImageWaterMarkConfig image_water_mark_param;
    /**
     * @locale zh
     * @valid since 3.57
     * @brief 设置占位图的填充模式。 <br>
     *        该参数用来控制当用户停止发布视频流，画面恢复为占位图后，此时占位图的填充模式。默认值为 `kMixedStreamAlternateImageFillModeFit`，占位图跟随用户原始视频帧相同的比例缩放。 <br>
     *        参看 MixedStreamAlternateImageFillMode{@link #MixedStreamAlternateImageFillMode}。 <br>
     *        WTN 流任务不支持设置本参数。
     */
    /**
     * @locale en
     * @brief Sets the fill mode of the placeholder image. <br>
     *        This parameter is used to control the fill mode of the placeholder image after the user stops publishing video streams and the screen reverts to the placeholder image.  It defaults to `kMixedStreamAlternateImageFillModeFit` indicating the image is scaled with the video's aspect ratio. <br>
     *        See MixedStreamAlternateImageFillMode{@link #MixedStreamAlternateImageFillMode}. <br>
     *        This parameter is not supported for WTN stream tasks.
     */
    MixedStreamAlternateImageFillMode alternate_image_fill_mode = kMixedStreamAlternateImageFillModeFit;
    /**
     * @locale zh
     * @valid since 3.57
     * @brief 设置占位图的 URL，长度小于 1024 字符.
     */
    /**
     * @locale en
     * @brief Sets the URL of the placeholder image, limited to a maximum of 1024 characters.
     */
    const char* alternate_image_url = nullptr;
    /**
     * @locale zh
     * @type keytype
     * @brief 空间位置。参看 Position{@link #Position}。 <br>
     *        WTN 流任务不支持设置本参数。
     */
    /**
     * @locale en
     * @type keytype
     * @brief spatial position. See Position{@link #Position}. <br>
     *        This parameter is not supported for WTN stream tasks.
     */
    Position spatial_position;
    /**
     * @locale zh
     * @type keytype
     * @brief 是否开启空间音频效果。默认值为 true。 <br>
     *        WTN 流任务不支持设置本参数。
     */
    /**
     * @locale en
     * @type keytype
     * @brief Whether to apply spatial audio effect. The default value is true. <br>
     *        This parameter is not supported for WTN stream tasks.
     */
    bool apply_spatial_audio = true;
    /**
     * @locale zh
     * @brief 支持对每一路参与WTN 流的视频进行裁剪，参看 SourceCrop{@link #SourceCrop} <br>
     * 合流转推任务不支持。
     */
    /**
     * @locale en
     * @brief Configurations on how to crop the WTN stream. Refer to SourceCrop{@link #SourceCrop} for more details. <br>
     *  This parameter is not supported for mixing-stream tasks.
     */
    SourceCrop source_crop;
} MixedStreamLayoutRegionConfig;
/**
 * @locale zh
 * @type keytype
 * @brief 数据帧类型
 */
/**
 * @locale en
 * @type keytype
 * @brief  Data frame type
 */
enum DataFrameType {
    /**
     * @locale zh
     * @brief SEI 视频帧
     */
    /**
     * @locale en
     * @brief SEI  video frame
     */
    kDataFrameTypeSei = 0,
};

/**
 * @locale zh
 * @hidden for internal use only
 * @type keytype
 * @brief 合流房间状态
 */
/**
 * @locale en
 * @hidden for internal use only
 * @type keytype
 * @brief Confluence room status
 */
enum TranscoderRoomStatus {
    /**
     * @brief join room status
     */
    kRoomStatusJoinRoom,
    /**
     * @brief leave room status
     */
    kRoomStatusLeaveRoom,
    /**
     * @brief room status was offline
     */
    kRoomStatusOffline,
    /**
     * @brief room status was online
     */
    kRoomStatusOnline,
};

/**
 * @hidden for internal use only
 */
typedef size_t status_t;
/**
 * @locale zh
 * @hidden for internal use only
 */
 /**
 * @locale en
 * @hidden for internal use only
 */
class IVideoFrame;
/**
 * @locale zh
 * @hidden for internal use only
 */
 /**
 * @locale en
 * @hidden for internal use only
 */
class IAudioFrame;

/**
 * @locale zh
 * @hidden for internal use only
 * @type keytype
 * @brief 数据帧
 */
/**
 * @locale en
 * @hidden for internal use only
 * @type keytype
 * @brief  Data frame
 */
typedef struct IDataFrame {
    /**
     * @locale zh
     * @brief 数据帧类型，参看 DataFrameType{@link #DataFrameType}
     */
    /**
     * @locale en
     * @brief Data frame type. See DataFrameType{@link #DataFrameType}
     */
    DataFrameType frame_type = kDataFrameTypeSei;
    /**
     * @locale zh
     * @brief 数据帧数据
     */
    /**
     * @locale en
     * @brief Data frame data
     */
    uint8_t* u8_data = nullptr;
    /**
     * @locale zh
     * @brief 数据帧数据大小
     */
    /**
     * @locale en
     * @brief Data frame data size
     */
    uint32_t u32_data_size = 0;
    /**
     * @locale zh
     * @brief 数据帧时间戳，单位：微秒
     */
    /**
     * @locale en
     * @brief Data frame timestamp in microseconds
     */
    uint64_t u64_ts_us = 0;
} IDataFrame;

/**
 * @locale zh
 * @type keytype
 * @brief 合流/WTN 流的转推目标参数。
 * 如无特别说明，参数可适用于 WTN 流和合流转推任务。
 * 如无特别说明，参数可用于启动和更新任务。
 */
/**
 * @locale en
 * @type keytype
 * @brief Configurations for the mixing streams.
 * If not specified, the parameter can be used for both WTN stream and mixing-stream tasks.
 * If not specified, the parameter can be passed when starting a task or during a task.
 */
typedef struct MixedStreamPushTargetConfig {
    /**
     * @locale zh
     * @type keytype
     * @brief  WTN流 ID。
     *        合流任务不支持设置本参数。
     */
    /**
     * @locale en
     * @type keytype
     * @brief The WTN stream ID.
     *        This parameter is not supported for mixing-stream stream tasks.
     */
    const char* push_wtn_stream_id = nullptr;
    /**
     * @locale zh
     * @type api
     * @brief 推流 CDN 地址。仅支持 RTMP 协议，Url 必须满足正则 `/^rtmps?:\/\//`。建议设置。 <br>
     *        本参数不支持过程中更新。
     *        WTN 流任务不支持设置本参数。
     */
    /**
     * @locale en
     * @type api
     * @brief The URL for the mixed stream to be pushed to. Only supports live transcoding via RTMP. The URL should match the regular expression `/^rtmps?:\/\//`. It's recommended to be set. <br>
     *        This parameter cannot be updated while pushing stream to the CDN.
     *        This parameter is not supported for WTN stream tasks.
     * @param pushURL URL for the mixed stream to be pushed to.
     */
    const char* push_cdn_url = nullptr;
    /**
     * @locale zh
     * @type keytype
     * @brief 推流任务类型。
     */
    /**
     * @locale en
     * @type keytype
     * @brief The task type.
     */
    MixedStreamPushTargetType push_target_type = kMixedStreamPushTargetTypeToCDN;
} MixedStreamPushTargetConfig;

/**
 * @locale zh
 * @type keytype
 * @brief 合流转推配置参数。
 * 如无特别说明，参数可适用于WTN 流和合流转推任务。
 * 如无特别说明，参数可用于启动和更新任务。
 */
/**
 * @locale en
 * @type keytype
 * @brief Stream-mixing task configurations. <br>
 * If not specified, the parameter can be used for both WTN stream and mixing-stream tasks. <br>
 * If not specified, the parameter can be passed when starting a task or during a task.
 */
class IMixedStreamConfig {
public:
    /**
     * @locale zh
     * @type api
     * @brief 构建合流任务参数
     * @return 合流任务参数
     */
    /**
     * @locale en
     * @type api
     * @brief Create a mixed stream task configuration.
     * @return A mixed stream task configuration.
     */
    static BYTERTC_STATIC_API IMixedStreamConfig* createMixedStreamConfig();
    /**
     * @locale zh
     * @type api
     * @brief 获取发起合流任务用户所在的房间 ID。
     * @return 合流房间 ID
     */
    /**
     * @locale en
     * @type api
     * @brief Get the ID of the room where the streams to be mixed sources from.
     * @return The room ID
     */
    virtual const char* getRoomID() = 0;
    /**
     * @locale zh
     * @type api
     * @brief 设置合流房间 ID
     * @param room_id 发起合流的用户所在的房间 ID
     * @note
     *        本参数不支持过程中更新。
     */
    /**
     * @locale en
     * @type api
     * @brief Set the room ID to specify where the streams to be mixed sources from.
     * @param room_id The room ID
     * @note
     *        This parameters cannot be updated during.
     */
    virtual void setRoomID(const char* room_id) = 0;
    /**
     * @locale zh
     * @type api
     * @brief 获取发起合流任务的用户 ID。
     * @return 用户 ID。
     */
    /**
     * @locale en
     * @type api
     * @brief Gets the ID of the user who started the mixing-stream task.
     * @return The user ID
     */
    virtual const char* getUserID() = 0;
    /**
     * @locale zh
     * @type api
     * @brief 设置发起合流任务的用户 ID。`room_id` 和 `user_id` 长度相加不得超过 126 字节。建议设置。 <br>
     *        本参数不支持过程中更新。
     * @param user_id 推流用户 ID。
     */
    /**
     * @locale en
     * @type api
     * @brief Sets the user ID to specify who started the mixing-stream task. The sum length of `room_id` and `user_id` should not exceed 126 bytes. It's recommended to be set. <br>
     *        This parameter cannot be updated during the task.
     * @param user_id The user ID.
     */
    virtual void setUserID(const char* user_id) = 0;
    /**
     * @locale zh
     * @type api
     * @brief 获取合流音频参数。
     * @return 合流音频参数内容，参看 MixedStreamAudioConfig{@link #MixedStreamAudioConfig}。
     */
    /**
     * @locale en
     * @type api
     * @brief Gets the audio configurations.
     * @return Audio configurations. See MixedStreamAudioConfig{@link #MixedStreamAudioConfig}.
     */
    virtual MixedStreamAudioConfig getAudioConfig() = 0;
    /**
     * @locale zh
     * @type api
     * @brief 设置音频参数。建议设置。
     * @param MixedStreamAudioConfig 音频参数，参看 MixedStreamAudioConfig{@link #MixedStreamAudioConfig}。
     * @note
     *      - 本参数不支持过程中更新。
     *      - WTN 流任务不支持设置本参数。
     */
    /**
     * @locale en
     * @type api
     * @brief Sets audio configurations. It's recommended to be set.
     * @param MixedStreamAudioConfig Audio configurations. See MixedStreamAudioConfig{@link #MixedStreamAudioConfig}.
     * @note
     *      - This parameters cannot be updated during the task.
     *      - This parameter is not supported for WTN stream tasks.
     */
    virtual void setAudioConfig(const MixedStreamAudioConfig&) = 0;
    /**
     * @locale zh
     * @type api
     * @brief 获取视频转码参数。
     * @return 视频转码参数内容，参看 MixedStreamVideoConfig{@link #MixedStreamVideoConfig}。
     */
    /**
     * @locale en
     * @type api
     * @brief Gets the video configurations.
     * @return Video configurations. See MixedStreamVideoConfig{@link #MixedStreamVideoConfig}.
     */
    virtual MixedStreamVideoConfig getVideoConfig() = 0;
    /**
     * @locale zh
     * @type api
     * @brief 设置视频参数。建议设置。
     * @param MixedStreamVideoConfig 视频参数，参看 MixedStreamVideoConfig{@link #MixedStreamVideoConfig}。
     */
    /**
     * @locale en
     * @type api
     * @brief Sets video configurations. It's recommended to be set.
     * @param MixedStreamVideoConfig Video configurations. See MixedStreamVideoConfig{@link #MixedStreamVideoConfig}.
     */
    virtual void setVideoConfig(const MixedStreamVideoConfig&) = 0;
    /**
     * @locale zh
     * @hidden(Windows,macOS,Linux)
     * @type api
     * @brief 获取转推 CDN 时的空间音频参数。
     * @return 参看 MixedStreamSpatialAudioConfig{@link #MixedStreamSpatialAudioConfig}。
     */
    /**
     * @locale en
     * @hidden(Windows,macOS,Linux)
     * @type api
     * @brief Get the spatial audio configurations of pushing to CDN.
     * @return See MixedStreamSpatialAudioConfig{@link #MixedStreamSpatialAudioConfig}.
     */
    virtual MixedStreamSpatialAudioConfig getSpatialAudioConfig() = 0;
    /**
     * @locale zh
     * @hidden(Windows,macOS,Linux)
     * @type api
     * @brief 设定转推 CDN 时的空间音频效果。参看 MixedStreamSpatialAudioConfig{@link #MixedStreamSpatialAudioConfig}。
     */
    /**
     * @locale en
     * @hidden(Windows,macOS,Linux)
     * @type api
     * @brief Sets spatial audio configurations. See MixedStreamSpatialAudioConfig{@link #MixedStreamSpatialAudioConfig}.
     */
    virtual void setSpatialAudioConfig(const MixedStreamSpatialAudioConfig&) = 0;
    /**
     * @locale zh
     * @hidden for internal use only
     * @type api
     * @brief 获取动态扩展自定义参数
     * @return 动态扩展自定义参数
     */
    /**
     * @locale en
     * @hidden for internal use only
     * @type api
     * @brief Get dynamically extend customizable parameters
     * @return dynamically extend customizable parameters
     */
    virtual const char* getAdvancedConfig() = 0;
    /**
     * @locale zh
     * @hidden for internal use only
     * @type api
     * @brief 设置动态扩展自定义参数
     * @param advanced_config 动态扩展自定义参数
     */
    /**
     * @locale en
     * @hidden for internal use only
     * @type api
     * @brief Sets dynamically extend customizable parameters
     * @param advanced_config  dynamically extend customizable parameters
     */
    virtual void setAdvancedConfig(const char* advanced_config) = 0;
    /**
     * @locale zh
     * @hidden for internal use only
     * @type api
     * @brief 获取业务透传鉴权信息
     * @return 业务透传鉴权信息
     */
    /**
     * @locale en
     * @hidden for internal use only
     * @type api
     * @brief Get Business Transparent Authentication Information
     * @return Business Transparent Authentication Information
     */
    virtual const char* getAuthInfo() = 0;
    /**
     * @locale zh
     * @hidden for internal use only
     * @type api
     * @brief 设置业务透传鉴权信息
     * @param auth_info 业务透传鉴权信息
     */
    /**
     * @locale en
     * @hidden for internal use only
     * @type api
     * @brief Sets Business Transparent Authentication Information
     * @param auth_info  Business Transparent Authentication Information
     */
    virtual void setAuthInfo(const char* auth_info) = 0;
    /**
     * @locale zh
     * @hidden for internal use only
     * @type api
     * @brief 获取转推直播同步参数。
     * @return 转推直播同步参数，参看 MixedStreamSyncControlConfig{@link #MixedStreamSyncControlConfig}。
     */
    /**
     * @locale en
     * @hidden for internal use only
     * @type api
     * @brief Get transcoding sync control parameters.
     * @return Transcoding sync control parameters. See MixedStreamSyncControlConfig{@link #MixedStreamSyncControlConfig}.
     */
    virtual MixedStreamSyncControlConfig getSyncControlConfig() = 0;
    /**
     * @locale zh
     * @hidden for internal use only
     * @type api
     * @brief 设置转推直播同步参数。参看 MixedStreamSyncControlConfig{@link #MixedStreamSyncControlConfig}。
     */
    /**
     * @locale en
     * @hidden for internal use only
     * @type api
     * @brief Sets transcoding sync control parameters. See MixedStreamSyncControlConfig{@link #MixedStreamSyncControlConfig}.
     */
    virtual void setSyncControlConfig(MixedStreamSyncControlConfig&) = 0;
    /**
     * @locale zh
     * @type api
     * @brief 获取服务端合流控制参数。参看 MixedStreamControlConfig{@link #MixedStreamControlConfig}。
     */
    /**
     * @locale en
     * @type api
     * @brief Gets the configurations while mixing streams on the server side. See MixedStreamControlConfig{@link #MixedStreamControlConfig}.
     */
    virtual MixedStreamControlConfig getControlConfig() = 0;
    /**
     * @locale zh
     * @type api
     * @brief 设置服务端合流控制参数。参看 MixedStreamControlConfig{@link #MixedStreamControlConfig}。
     * @param config 参看 MixedStreamControlConfig{@link #MixedStreamControlConfig}.
     */
    /**
     * @locale en
     * @type api
     * @brief Sets the configurations while mixing streams on the server side. See MixedStreamControlConfig{@link #MixedStreamControlConfig}.
     * @param config see MixedStreamControlConfig{@link #MixedStreamControlConfig}.
     */
    virtual void setControlConfig(MixedStreamControlConfig& config) = 0;
    /**
     * @locale zh
     * @type api
     * @brief 获取用户配置的额外信息。
     * @return 用户配置的额外信息。
     */
    /**
     * @locale en
     * @type api
     * @brief Get user config extra information.
     * @return ExtraI Information.
     */
    virtual const char* getUserConfigExtraInfo() = 0;
    /**
     * @locale zh
     * @type api
     * @brief 设置用户配置的额外信息。
     * @param user_ext_info 用户配置的额外信息。
     * @note
     *      WTN 流任务不支持设置本参数。
     */
    /**
     * @locale en
     * @type api
     * @brief set user config extra information.
     * @param user_ext_info Extra information.
     * @note
     *      This parameter is not supported for WTN stream tasks.
     */
    virtual void setUserConfigExtraInfo(const char* user_ext_info) = 0;
    /**
     * @locale zh
     * @type api
     * @brief 获取 WTN 流的布局模式。
     * @return StreamLayoutMode{@link #StreamLayoutMode} 信息。
     */
    /**
     * @locale en
     * @type api
     * @brief Get the layout mode of the WTN stream.
     * @return StreamLayoutMode{@link #StreamLayoutMode} Information.
     */
    virtual StreamLayoutMode getStreamLayoutMode() = 0;
    /**
     * @locale zh
     * @type api
     * @brief 设置 WTN 流的布局模式。
     * @param mode 布局模式。参看 StreamLayoutMode{@link #StreamLayoutMode}。
     */
    /**
     * @locale en
     * @type api
     * @brief Sets the layout mode of the WTN stream.
     * @param mode Layout mode. See StreamLayoutMode{@link #StreamLayoutMode}.
     */
    virtual void setStreamLayoutMode(StreamLayoutMode mode) = 0;
    /**
     * @locale zh
     * @type api
     * @brief 获取 WTN 流的补帧模式。
     * @return InterpolationMode{@link #InterpolationMode} 信息。
     */
    /**
     * @locale en
     * @type api
     * @brief Get the interpolation mode of the WTN stream.
     * @return InterpolationMode{@link #InterpolationMode} Information.
     */
    virtual InterpolationMode getInterpolationMode() = 0;
    /**
     * @locale zh
     * @type api
     * @brief 设置 WTN 流的补帧模式。
     * @param mode 补帧模式。参看 InterpolationMode{@link #InterpolationMode}。
     */
    /**
     * @locale en
     * @type api
     * @brief Sets the interpolationMode mode of the WTN stream.
     * @param mode Interpolation mode. See InterpolationMode{@link #InterpolationMode}.
     */
    virtual void setInterpolationMode(InterpolationMode mode) = 0;

    /**
     * @locale zh
     * @type api
     * @brief 获取 WTN 流 ID。
     * @return WTN 流 ID。
     */
    /**
     * @locale en
     * @type api
     * @brief Gets the WTN stream ID.
     * @return The WTN stream ID.
     */
    virtual const char* getPushWTNStreamID() = 0;
    /**
     * @locale zh
     * @type api
     * @brief 设置 WTN 流 ID。
     * @param stream_id WTN 流 ID。
     */
    /**
     * @locale en
     * @type api
     * @brief Specify the WTN stream.
     * @param stream_id The WTN stream ID.
     */
    virtual void setPushWTNStreamID(const char* stream_id) = 0;
    /**
     * @locale zh
     * @type api
     * @brief 获取任务类型。
     * @return MixedStreamPushTargetType{@link #MixedStreamPushTargetType}
     */
    /**
     * @locale en
     * @type api
     * @brief Gets the task type.
     * @return MixedStreamPushTargetType{@link #MixedStreamPushTargetType}
     */
    virtual MixedStreamPushTargetType getPushTargetType() = 0;
    /**
     * @locale zh
     * @type api
     * @brief 设置推流目标。
     * @param type 参看 MixedStreamPushTargetType{@link #MixedStreamPushTargetType}。
     */
    /**
     * @locale en
     * @type api
     * @brief Sets push task type.
     * @param type See MixedStreamPushTargetType{@link #MixedStreamPushTargetType}.
     */
    virtual void setPushTargetType(MixedStreamPushTargetType type) = 0;

    /**
     * @locale zh
     * @type api
     * @brief 获取背景颜色。
     * @return 背景颜色，用十六进制颜色码（HEX）表示。例如，#FFFFFF 表示纯白，#000000 表示纯黑。默认值为 #000000。建议设置。
     */
    /**
     * @locale en
     * @type api
     * @brief Gets the background color.
     * @return Background color. Use HEX color code to represent. For example, #FFFFFF for white, and #000000 for black.
     */
    virtual const char* getBackgroundColor() = 0;
    /**
     * @locale zh
     * @type api
     * @brief 设置背景颜色。
     * @param background_color 背景颜色，用十六进制颜色码（HEX）表示。例如，#FFFFFF 表示纯白，#000000 表示纯黑。默认值为 #000000。建议设置。
     */
    /**
     * @locale en
     * @type api
     * @brief set user config extra information.
     * @param background_color Background color. Use HEX color code to represent. For example, #FFFFFF for white, and #000000 for black. The default value is #000000. It's recommended to be set.
     */
    virtual void setBackgroundColor(const char* background_color) = 0;
    /**
     * @locale zh
     * @type api
     * @brief 获取背景图片的地址。
     * @return 背景图片的地址。
     */
    /**
     * @locale en
     * @type api
     * @brief Gets the background image URL.
     * @return Background image URL.
     */
    virtual const char* getBackgroundImageURL() = 0;
    /**
     * @locale zh
     * @type api
     * @brief 设置合流后整体画布的背景图片。
     * @param background_image_url 图片的 URL，长度最大为 1024 bytes。 <br>
     *        支持的图片格式包括：JPG, JPEG, PNG。如果背景图片的宽高和整体屏幕的宽高不一致，背景图片会缩放到铺满屏幕。
     */
    /**
     * @locale en
     * @type api
     * @brief Sets the background image for the canvas that renders the mixed stream.
     * @param background_image_url The URL of the image, with a maximum length of 1024 bytes. <br>
     *        Supported image formats include: JPG, JPEG, PNG. If the width and height of the background image do not match the width and height of the overall screen, the background image will be scaled to fill the screen.
     */
    virtual void setBackgroundImageURL(const char* background_image_url) = 0;
    /**
     * @locale zh
     * @type api
     * @brief 获取合流视窗布局信息。
     * @param index 视窗对应下标。
     * @return 合流视窗布局信息，参看 MixedStreamLayoutRegionConfig{@link #MixedStreamLayoutRegionConfig}。
     */
    /**
     * @locale en
     * @valid since 3.57
     * @type api
     * @brief Gets the video layout information of the mixed stream.
     * @param index The number of the view of which you want to get information.
     * @return Layout information. See MixedStreamLayoutRegionConfig{@link #MixedStreamLayoutRegionConfig}.
     */
    virtual MixedStreamLayoutRegionConfig getLayoutByIndex(int32_t index) = 0;
    /**
     * @locale zh
     * @type api
     * @brief 获取合流参数中视窗的数量
     * @return 合流参数中视窗的数量
     */
    /**
     * @locale en
     * @type api
     * @brief Get the number of windows in the confluence parameter
     * @return The number of windows in the confluence parameter
     */
    virtual int32_t getLayoutRegionsSize() = 0;
    /**
     * @locale zh
     * @type api
     * @brief 设置视频流合流整体布局信息。
     * @param regions 用户布局信息列表。为 MixedStreamLayoutRegionConfig{@link #MixedStreamLayoutRegionConfig} 数据类型的数组。每一个该类型对象为一路单独的视频流的布局信息。 <br>
     *                       值不合法或未设置时，自动使用默认值。建议设置。
     * @param regions_size 合流视窗数量。建议设置。
     */
    /**
     * @locale en
     * @type api
     * @brief Sets the layout configurations.
     * @param regions User layout information list. It's a list of MixedStreamLayoutRegionConfig{@link #MixedStreamLayoutRegionConfig} that you construct for each stream. <br>
     *                       With invalid or empty input, the configurations will be set as the default values. It's recommended to be set.
     * @param regions_size Amount of views. It's recommended to be set.
     */
    virtual void setLayoutConfig(MixedStreamLayoutRegionConfig regions[], int32_t regions_size) = 0;
    virtual ~IMixedStreamConfig() = default;
};

/**
 * @locale zh
 * @type keytype
 * @brief 单流转推配置目标房间信息
 */
/**
 * @locale en
 * @type keytype
 * @brief Configurations for pushing a single media stream dest room info
 */
typedef struct DestInfo {
    /**
     * @locale zh
     * @brief 跨房间转发媒体流过程中目标房间 ID
     */
    /**
     * @locale en
     * @brief ID of the room where the media stream aims to forward to.
     */
    const char* room_id;
    /**
     * @locale zh
     * @brief 转发目标房间中自定义用户ID 
     */
    /**
     * @locale en
     * @brief ID of the user where the media stream aims to forward to in aim room.
     */
    const char* user_id;
} DestInfo;

/**
 * @locale zh
 * @type keytype
 * @brief 单流转推类型
 */
/**
 * @locale en
 * @type keytype
 * @brief forward stream push type
 */
enum SingleStreamPushType {
    /**
     * @locale zh
     * @brief 单流转推到CDN
     */
    /**
     * @locale en
     * @brief single stream retweeting to CDN.
     */
    kPushToCDN = 1,
    /**
     * @locale zh
     * @brief 单流转推到RTC房间
     */
    /**
     * @locale en
     * @brief single stream retweeting to RTC room.
     */
    kPushToRTC = 2
};

/**
 * @locale zh
 * @type keytype
 * @brief 单流转推直播配置参数。
 */
/**
 * @locale en
 * @type keytype
 * @brief Configurations for pushing a single media stream to CDN.
 */
typedef struct PushSingleStreamParam {
    /**
     * @locale zh
     * @brief 媒体流所在的房间 ID
     */
    /**
     * @locale en
     * @brief The room ID of the media stream
     */
    const char* room_id;
    /**
     * @locale zh
     * @brief 媒体流所属的用户 ID
     */
    /**
     * @locale en
     * @brief The user ID of the media stream
     */
    const char* user_id;
    /**
     * @locale zh
     * @brief 推流 CDN 地址。仅支持 RTMP 协议，Url 必须满足正则 `/^rtmps?:\/\//`。 <br>
     *        本参数不支持过程中更新。
     */
    /**
     * @locale en
     * @brief The URL for live transcoding. Only supports live transcoding via RTMP. The URL should match the regular expression `/^rtmps?:\/\//`. <br>
     *        This parameter cannot be updated during the task.
     */
    const char* uri;
    /**
     * @locale zh
     * @brief 媒体流是否为屏幕流。
     */
    /**
     * @locale en
     * @brief Whether the media stream is a screen-sharing stream.
     */
    bool is_screen_stream;
    /**
     * @locale zh
     * @brief 转推目标房间数组，默认值为nullptr
     */
    /**
     * @locale en 
     * @brief forward dest room info，nullptr by default
     */
    DestInfo* dest_info = nullptr;
    /**
     * @locale zh
     * @brief 转推目标房间数量,`dest_info` 的数组长度， <br>
     *        默认值为0,取值范围[0, 8]
     */
    /**
     * @locale en 
     * @brief forward dest room counts, The length of `dest_info`.
     *        0 by default, The range is `[0, 8]`
     */
    int32_t dest_info_size = 0;
    /**
     * @locale zh
     * @brief 转推类型，默认值转推CDN
     */
    /**
     * @locale en
     * @brief forward stream type, kPushToCDN by default.
     */
    SingleStreamPushType push_type = kPushToCDN;
}PushSingleStreamParam;

/**
* @locale zh
* @hidden internal use only
* @type keytype
* @brief 缓存同步模式。
*/
/**
* @locale en
* @hidden internal use only
* @type keytype
*/
enum ChorusCacheSyncMode {
    /**
     * @locale zh
     * @brief 合唱场景下，主唱应采用此模式，以发送带时间戳信息的媒体数据。
     */
    /**
     * @locale en
     * @brief Under chorus scenarios, the lead singer uses the mode to send the media data with timestamp attached.
     */
    kChorusCacheSyncModeProducer= 0,
    /**
     * @locale zh
     * @brief 合唱场景下，副唱应采用此模式。 <br>
     *        此模式下，副唱收到来自主唱的带时间戳的媒体数据。副唱发送的媒体数据中带有来自主唱的时间戳。
     */
    /**
     * @locale en
     * @brief Under chorus scenarios, the sub singer uses the mode. <br>
     *        In this mode, the sub singer receives the media data from the lead singer with timestamp attached. The sub singer sends the media data with the lead singer's timestamp attached.
     */
    kChorusCacheSyncModeRetransmitter= 1,
    /**
     * @locale zh
     * @brief 合唱场景下，听众应采用此模式。 <br>
     *        此模式下，听众收到来自主唱的时间戳，并据此对齐来自主唱和副唱的媒体数据，以获得良好的合唱播放效果。
     */
    /**
     * @locale en
     * @brief Under chorus scenarios, the audience uses the mode. <br>
     *        In this mode, the audience receives the lead singer's timestamp. The audience aligns and renders the media data from lead and sub singers, and gets good chorus experience.
     */
    kChorusCacheSyncModeConsumer = 2
};
/**
* @locale zh
* @hidden internal use only
* @type keytype
* @brief 缓存同步事件。
*/
/**
* @locale en
* @hidden internal use only
* @type keytype
*/
enum ChorusCacheSyncEvent {
     /**
     * @locale zh
     * @brief 成功
     */
    /**
     * @locale en
     * @brief Success
     */
    kChorusCacheSyncEventStartSuccess = 0,
     /**
     * @locale zh
     * @brief 失败。
     */
    /**
     * @locale en
     * @brief Failed
     */
    kChorusCacheSyncEventStartFailed = 1,
};
/**
* @locale zh
* @hidden internal use only
* @type errorcode
* @brief 缓存同步错误码。
*/
/**
* @locale en
* @hidden internal use only
* @type errorcode
*/
enum ChorusCacheSyncError {
    /**
     * @locale zh
     * @brief 成功。
     */
    /**
     * @locale en
     * @brief OK
     */
    kChorusCacheSyncErrorOK = 0,
   /**
    * @locale zh
    * @brief 失败。推送至 CDN 时，应进行以下设置： <br>
    *        - `IMixedStreamConfig.MixedStreamSyncControlConfig.enable_sync = true`；
    *        - `IMixedStreamConfig.MixedStreamSyncControlConfig.base_user_id = {uid of producer}`。
    */
    /**
    * @locale en
    * @type Failure. Check if the config of pushing mixed streams to CDN is as follows: <br>
    *        - `IMixedStreamConfig.MixedStreamSyncControlConfig.enable_sync = true`；
    *        - `IMixedStreamConfig.MixedStreamSyncControlConfig.base_user_id = {uid of producer}`。
    */   
    kChorusCacheSyncErrorWrongState = 1,
    /**
    * @locale zh
    * @brief 缓存同步功能已启动，不需要重复开启。
    */
    /**
    * @locale en
    * @type Already running. Do not enable the feature repeatedly.
    */
    kChorusCacheSyncErrorAlreadyRunning = 2
};
/**
* @locale zh
* @hidden internal use only
* @type keytype
* @brief 缓存同步配置。
*/
/**
* @locale en
* @hidden internal use only
* @type keytype
*/
struct ChorusCacheSyncConfig {
     /**
      * @locale zh
      * @brief 最大媒体缓存时长（ms）。 <br>
      *        取值范围是 `[500, 2500]`，默认值是 `2000`。 <br>
      *        值越大，同步效果越好，但会造成占用内存较大。如果参与缓存同步的各路媒体流之间的时间差超过此值，会造成丢帧。
      */
     /**
      * @locale en
      * @brief Maximum duration of media data cached in ms. <br>
      *        `2000` by default. The range is `[500, 2500]`. <br>
      *        Use higher value for better syncing effect. If the time difference between media streams in cache synchronization exceeds this value, frame loss will occur.
      */
    int32_t max_cache_time_ms = 2000;
    /**
      * @locale zh
      * @brief 收到 onSyncedVideoFrames{@link #IChorusCacheSyncObserver#onSyncedVideoFrames} 的频率。 <br>
      *        默认值是 `15`。此值通常应小于等于原始视频帧率；如果大于原始视频帧率，可能会收到重复帧。
      */
     /**
      * @locale en
      * @brief The frequency of receiving onSyncedVideoFrames{@link #IChorusCacheSyncObserver#onSyncedVideoFrames}. <br>
      *        `15` by default. The value is smaller than that of the original video fps usually; if the value is bagger than that, you may receive duplicated video frame.
      */
    int32_t video_fps = 15;
    /**
      * @locale zh
      * @brief 模式。参看 ChorusCacheSyncMode{@link #ChorusCacheSyncMode}. 默认值是 `retransmitter`。
      */
     /**
      * @locale en
      * @brief Mode. See ChorusCacheSyncMode{@link #ChorusCacheSyncMode}. `retransmitter` by default.
      */
    ChorusCacheSyncMode mode = kChorusCacheSyncModeRetransmitter;
};

/**
 * @locale zh
 * @type callback
 * @hidden for internal use only on Windows
 * @brief 合流推流 Observer <br>
 * 注意：回调函数是在 SDK 内部线程（非 UI 线程）同步抛出来的，请不要做耗时操作或直接操作 UI，否则可能导致 app 崩溃。
 */
/**
 * @locale en
 * @type callback
 * @hidden for internal use only on Windows
 * @region Push to CDN
 * @brief Register this observer to receive stream mixing related callbacks from the SDK. <br>
 * Note: Callback functions are thrown synchronously in a non-UI thread within the SDK. Therefore, you must not perform any time-consuming operations or direct UI operations within the callback function, as this may cause the app to crash.
 */
class IClientMixedStreamObserver {
public:
    /**
     * @locale zh
     * @hidden for internal use only on Windows
     * @type callback
     * @brief 转推直播状态回调
     * @param info 参看 MixedStreamTaskInfo{@link #MixedStreamTaskInfo}
     * @param type 转推直播类型，参看 MixedStreamType{@link #MixedStreamType}
     * @param event 转推直播任务状态，参看 MixedStreamTaskEvent{@link #MixedStreamTaskEvent}
     * @param error_code 转推直播错误码，参看 MixedStreamTaskErrorCode{@link #MixedStreamTaskErrorCode}。
     */
    /**
     * @locale en
     * @hidden for internal use only on Windows
     * @type callback
     * @brief Used for reporting events during pushing streams to CDN
     * @param event Type Stream mixing status, see MixedStreamTaskEvent{@link #MixedStreamTaskEvent}
     * @param task_id Task ID
     * @param error_code Errors occurring during the pushing process. See MixedStreamTaskErrorCode{@link #MixedStreamTaskErrorCode}
     * @param mix_type Stream mixing and pushing type. See MixedStreamType{@link #MixedStreamType}
     */
    virtual void onClientMixedStreamEvent(MixedStreamTaskInfo info, MixedStreamType type, MixedStreamTaskEvent event, MixedStreamTaskErrorCode error_code) = 0;
    /**
     * @locale zh
     * @hidden for internal use only on Windows
     * @type callback
     * @region CDN 推流
     * @brief 客户端合流音频首帧回调
     * @param task_id 任务 ID
     */
    /**
     * @locale en
     * @hidden for internal use only on Windows
     * @type callback
     * @region Push to CDN
     * @brief
     * @param task_id Task ID
     */
    virtual void onMixedFirstAudioFrame(const char* task_id) = 0;
    /**
     * @locale zh
     * @hidden for internal use only on Windows
     * @type callback
     * @region CDN 推流
     * @brief 客户端合流视频首帧回调
     * @param task_id 任务 ID
     */
    /**
     * @locale en
     * @hidden(Linux) not available
     * @type callback
     * @region Push to CDN
     * @brief
     * @param task_id Task ID
     */
    virtual void onMixedFirstVideoFrame(const char* task_id) = 0;
    /**
     * @locale zh
     * @type callback
     * @hidden for internal use only on Windows
     * @region CDN 推流
     * @brief 合流视频回调，运行在视频回调线程
     * @param task_id 合流任务 ID
     * @param video_frame 视频帧，参看 IVideoFrame{@link #IVideoFrame}。
     * @note 收到该回调的周期与视频的帧间隔一致。
     */
    /**
     * @locale en
     * @type callback
     * @hidden for internal use only on Windows
     * @region Push to CDN
     * @brief Callback with the video data after stream mixing, running on the video callback thread
     * @param task_id Task ID
     * @param video_frame Video Frame, see IVideoFrame{@link #IVideoFrame}.
     * @note The interval between callbacks is the same with that between video frames.
     */
    virtual void onMixedVideoFrame(const char* task_id, IVideoFrame* video_frame) = 0;

    /**
     * @locale zh
     * @type callback
     * @hidden for internal use only on Windows
     * @region CDN 推流
     * @brief 合流音频回调，运行在音频回调线程
     * @param task_id 转推直播任务 ID
     * @param audio_frame 音频帧，参看 IAudioFrame{@link #IAudioFrame}。
     * @note 收到该回调的周期为每 10 毫秒一次，并且每次的音频数据量为 10 毫秒数据量。
     */
    /**
     * @locale en
     * @type callback
     * @hidden for internal use only on Windows
     * @region Push to CDN
     * @brief Callback with the audio data after stream mixing, running on the audio callback thread
     * @param task_id Task ID.
     * @param audio_frame Audio Frame, see IAudioFrame{@link #IAudioFrame}.
     * @note You will receive the callback every 10 milliseconds. Each callback carries data collected in the last 10 <br>
     * milliseconds.
     */
    virtual void onMixedAudioFrame(const char* task_id, IAudioFrame* audio_frame) = 0;

    /**
     * @locale zh
     * @type callback
     * @hidden for internal use only on Windows
     * @region CDN 推流
     * @brief 视频 SEI 帧回调，运行在视频回调线程
     * @param task_id 转推直播任务 ID
     * @param data_frame SEI 数据
     */
    /**
     * @locale en
     * @type callback
     * @hidden for internal use only on Windows
     * @region Push to CDN
     * @brief This callback carries SEI data, running on the video callback thread
     * @param task_id Task ID
     * @param data_frame SEI data
     */
    virtual void onMixedDataFrame(const char* task_id, IDataFrame* data_frame) = 0;

    /**
     * @locale zh
     * @hidden for internal use only
     * @type callback
     * @region CDN 推流
     * @brief 同步视频帧回调。
     * @param task_id 转推直播任务 ID。
     * @param uids 同步视频帧对应的 uid 数组。
     * @param video_frames 同步视频帧数组，与 uids 对应。
     * @param data_frame SEI 数据。
     * @param count 数组的长度。
     */
    /**
     * @locale en
     * @hidden for internal use only
     * @type callback
     * @region Push to CDN
     * @brief Synchronized video frame callback.
     * @param task_id Task ID.
     * @param uids Array of synchronized video frames corresponding to uids.
     * @param video_frames Array of synchronized video frames corresponding to uids.
     * @param data_frame SEI data.
     * @param count The length of the array.
     */
    virtual void onCacheSyncVideoFrames(const char* task_id, const char* uids[], IVideoFrame* video_frames[],
            IDataFrame* data_frame[], int count) = 0;

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
    virtual ~IClientMixedStreamObserver() = default;
};

/**
 * @locale zh
 * @hidden internal use only
 * @type callback
 * @brief 缓存同步 Observer
 */
/**
 * @locale en
 * @hidden internal use only
 * @type callback
 * @brief Observer
 */
class IChorusCacheSyncObserver {
public:
    /**
     * @hidden constructor/destructor
    */
    virtual ~IChorusCacheSyncObserver() {
    }
    /**
     * @locale zh
     * @type callback
     * @brief 调用 startChorusCacheSync{@link #IRTCEngine#startChorusCacheSync}，并设置为 `consumer` 的用户会通过此回调获取经缓存同步后的视频帧。获取频率通过启动同步时的 `fps` 进行设置。
     * @param count `uids` 和 `videoFrames` 的数组长度
     * @param uids 参与合唱缓存同步的 `producer` 和 `retransmitter` 的列表，不包括参与但未发送媒体数据的用户。
     * @param video_frames 对应 `uids` 的视频帧。参看 IVideoFrame{@link #IVideoFrame}。
     */
    /**
     * @locale en
     * @type callback
     * @brief The user who calls startChorusCacheSync{@link #IRTCEngine#startChorusCacheSync} as `consumer` receives the callback with synced video frames. The interval of receiving the callback is set by `fps`.
     * @param count length of `uids`
     * @param uids The list of participants as `producer` and `retransmitter`. The participants not sending media data are excluded.
     * @param video_frames Video data frame corresponding to `uids`. See IVideoFrame{@link #IVideoFrame}.
     */    
    virtual void onSyncedVideoFrames(int count, const char* uids[], bytertc::IVideoFrame* video_frames[]) = 0;
    /**
     * @locale zh
     * @type callback
     * @brief 参与合唱缓存同步的 `producer` 和 `retransmitter` 发生变化时，收到此回调。
     * @param count 当前的 `uids` 的长度
     * @param uids 当前的参与者列表
     * @note 有以下情况可能造成参与者发生变化： <br>
     *        - 用户主动调用 startChorusCacheSync{@link #IRTCEngine#startChorusCacheSync} 或 stopChorusCacheSync{@link #IRTCEngine#stopChorusCacheSync};
     *        - 原本参与缓存同步的用户发生异常退出。
     */
    /**
     * @locale en
     * @type callback
     * @brief Receives the callback when the `producer` or `retransmitter` changes.
     * @param count Current length of `uids`
     * @param uids Current list of the participants
     * @note You may receive the callback is the following cases: <br>
     *        - The user calls startChorusCacheSync{@link #IRTCEngine#startChorusCacheSync} or stopChorusCacheSync{@link #IRTCEngine#stopChorusCacheSync};
     *        - The cache syncing participant quits abnormally.
     */
    virtual void onSyncedUsersChanged(int count, const char* uids[]) = 0;
    /**
     * @locale zh
     * @type callback
     * @brief 缓存同步事件回调
     * @param event 事件，参看 ChorusCacheSyncEvent{@link #ChorusCacheSyncEvent}。
     * @param error 错误码，参看 ChorusCacheSyncError{@link #ChorusCacheSyncError}。
     */
    /**
     * @locale en
     * @type callback
     * @brief Chorus cache sync event callback
     * @param event See ChorusCacheSyncEvent{@link #ChorusCacheSyncEvent}.
     * @param error See ChorusCacheSyncError{@link #ChorusCacheSyncError}.
     */
    virtual void onSyncEvent(ChorusCacheSyncEvent event, ChorusCacheSyncError error) = 0;
};
}  // namespace bytertc
