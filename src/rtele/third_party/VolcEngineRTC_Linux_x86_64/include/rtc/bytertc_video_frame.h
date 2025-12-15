/*
 * Copyright (c) 2020 The VolcEngineRTC project authors. All Rights Reserved.
 * @brief VolcEngineRTC Video Frame
 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include <memory>
#include <cstring>
#include "bytertc_media_defines.h"
#ifdef BYTERTC_ANDROID
#include <jni.h>
#endif

namespace bytertc {

/**
 * @locale zh
 * @type keytype
 * @brief 编码帧类型
 */
/**
 * @locale en
 * @type keytype
 * @brief Video compression picture type
 */
enum VideoPictureType {
    /**
     * @locale zh
     * @brief 未知类型
     */
    /**
     * @locale en
     * @brief Unknown type
     */
    kVideoPictureTypeUnknown = 0,
    /**
     * @locale zh
     * @brief I 帧，关键帧，编解码不需要参考其他视频帧
     */
    /**
     * @locale en
     * @brief I-frames, key frames which are coded without reference to any other frame.
     */
    kVideoPictureTypeI,
    /**
     * @locale zh
     * @brief P 帧，向前参考帧，编解码需要参考前一帧视频帧
     */
    /**
     * @locale en
     * @brief P-frames, forward-predicted frames which are coded by a forward predictive coding method.
     */
    kVideoPictureTypeP,
    /**
     * @locale zh
     * @brief B 帧，前后参考帧，编解码需要参考前后两帧视频帧
     */
    /**
     * @locale en
     * @brief  B-frames, bi-directionally predicted frames which are coded by both forward and backward predictive coding method.
     */
    kVideoPictureTypeB
};

/**
 * @locale zh
 * @type keytype
 * @brief 视频帧旋转信息
 */
/**
 * @locale en
 * @type keytype
 * @brief Video frame rotation
 */
enum VideoRotation {
    /**
     * @locale zh
     * @brief 不旋转
    */
   /**
     * @locale en
     * @brief Not to rotate.
    */
    kVideoRotation0 = 0,
    /**
     * @locale zh
     * @brief 顺时针旋转 90 度
    */
   /**
     * @locale en
     * @brief Rotate 90 degrees clockwise.
    */
    kVideoRotation90 = 90,
    /**
     * @locale zh
     * @brief 顺时针旋转 180 度
    */
   /**
     * @locale en
     * @brief Rotate 180 degrees clockwise.
    */
    kVideoRotation180 = 180,
    /**
     * @locale zh
     * @brief 顺时针旋转 270 度
    */
   /**
     * @locale en
     * @brief Rotate 270 degrees clockwise.
    */
    kVideoRotation270 = 270
};

/**
 * @locale zh
 * @type keytype
 * @brief 视频帧缩放模式，默认值为 FitWithCropping。
 */
/**
 * @locale en
 * @type keytype
 * @brief  Video frame scale mode
 */
enum VideoStreamScaleMode {
    /**
     * @locale zh
     * @brief 自动模式
     */
    /**
     * @locale en
     * @brief Auto mode, default to FitWithCropping.
     */
    kVideoStreamScaleModeAuto = 0,
    /**
     * @locale zh
     * @brief 对视频帧进行缩放，直至充满和视窗分辨率一致为止。这一过程不保证等比缩放。
     */
    /**
     * @locale en
     * @brief Stretch the video frame until the video frame and the window have the same resolution. The video frame's aspect ratio can be changed as it is automatically stretched to fill the window, but the whole image is visible.
     */
    kVideoStreamScaleModeStretch = 1,
    /**
     * @locale zh
     * @brief 视窗填满优先。 <br>
     *        视频帧等比缩放，直至视窗被视频填满。如果视频帧长宽比例与视窗不同，视频帧的多出部分将无法显示。 <br>
     *        缩放完成后，视频帧的一边长和视窗的对应边长一致，另一边长大于等于视窗对应边长。
     */
    /**
     * @locale en
     * @brief  Fit the window with cropping <br>
     *         Scale the video frame uniformly until the window is filled. If the video frame's aspect ratio is different from that of the window, the extra part of the video frame will be cropped. <br>
     *         After the scaling process is completed, the width or height of the video frame will be consistent with that of the window, and the other dimension will be greater than or equal to that of the window.
     */
    kVideoStreamScaleModeFitWithCropping = 2,
    /**
     * @locale zh
     * @brief 视频帧内容全部显示优先。 <br>
     *        视频帧等比缩放，直至视频帧能够在视窗上全部显示。如果视频帧长宽比例与视窗不同，视窗上未被视频帧填满区域将被涂黑。 <br>
     *        缩放完成后，视频帧的一边长和视窗的对应边长一致，另一边长小于等于视窗对应边长。
     */
    /**
     * @locale en
     * @brief  Fit the window with filling <br>
     *         Scale the video frame uniformly until its width or height reaches the boundary of the window. If the video frame's aspect ratio is different from that of the window, the area that is not filled will be black. <br>
     *         After the scaling process is completed, the width or height of the video frame will be consistent with that of the window, and the other dimension will be less than or equal to that of the window.
     */
    kVideoStreamScaleModeFitWithFilling = 3,
};


/**
 * @locale zh
 * @type keytype
 * @brief 视频编码模式
 */
/**
 * @locale en
 * @type keytype
 * @brief  Video encoding mode
 */
enum VideoCodecMode {
    /**
     * @locale zh
     * @brief 自动选择
     */
    /**
     * @locale en
     * @brief Automatic selection
     */
    kVideoCodecModeAuto = 0,
    /**
     * @locale zh
     * @brief 硬编码
     */
    /**
     * @locale en
     * @brief Hardcoding
     */
    kVideoCodecModeHardware,
    /**
     * @locale zh
     * @brief 软编码
     */
    /**
     * @locale en
     * @brief Softcoding
     */
    kVideoCodecModeSoftware,
};

/**
 * @locale zh
 * @type keytype
 * @brief 编码策略偏好。
 */
/**
 * @locale en
 * @type keytype
 * @brief  Encoder preference.
 */
enum VideoEncodePreference {
    /**
     * @locale zh
     * @brief 无偏好。不降低帧率和分辨率。
     */
    /**
     * @locale en
     * @brief No preference. The frame rate and the resolution will not be adjusted.
     */
    kVideoEncodePreferenceDisabled = 0,
    /**
     * @locale zh
     * @brief 优先保障帧率。适用于动态画面。
     */
    /**
     * @locale en
     * @brief The high frame rate mode. Ensure the highest framerate possible. This mode is designed for videos with high-motion contents.
     */
    kVideoEncodePreferenceFramerate,
    /**
     * @locale zh
     * @brief 清晰模式，优先保障分辨率。适用于静态画面。
     */
    /**
     * @locale en
     * @brief The high-resolution mode. Ensure the highest resolution possible. This mode is designed for videos with static contents.
     */
    kVideoEncodePreferenceQuality,
    /**
     * @locale zh
     * @brief 平衡帧率与分辨率。
     * 对于屏幕流来说是开启自动模式智能模式，将根据屏幕内容智能决策选择流畅模式或清晰模式。
     */
    /**
     * @locale en
     * @brief Balancing resolution and frame rate.
     * For the screen-recordings, the mode is dynamically determined by RTC based on the content.
     */
    kVideoEncodePreferenceAuto,
};


/**
 * @locale zh
 * @type keytype
 * @brief 摄像头。
 */
/**
 * @locale en
 * @type keytype
 * @brief  camera.
 */
enum CameraID {
    /**
     * @locale zh
     * @brief 移动端前置摄像头，PC 端内置摄像头
     */
    /**
     * @locale en
     * @brief Front-facing camera for mobile, build-in camera for PC
     */
    kCameraIDFront = 0,
    /**
     * @locale zh
     * @brief 移动端后置摄像头，PC 端无定义
     */
    /**
     * @locale en
     * @brief Postconditioning camera for mobile, PC is undefined for camera 1
     */
    kCameraIDBack = 1,
    /**
     * @locale zh
     * @hidden currently not available
     * @brief 外接摄像头
     */
    /**
     * @locale en
     * @hidden currently not available
     * @brief External camera
     */
    kCameraIDExternal = 2,
    /**
     * @locale zh
     * @brief 无效值
     */
    /**
     * @locale en
     * @brief Invalid value
     */
    kCameraIDInvalid = 3
};


#define SEND_KBPS_AUTO_CALCULATE -1
#define SEND_KBPS_DISABLE_VIDEO_SEND 0

/**
 * @locale zh
 * @deprecated since 3.36 along with setVideoEncoderConfig(StreamIndex index, const VideoSolution* solutions, int solution_num) = 0;
 * @type keytype
 * @brief 视频流参数
 */
/**
 * @locale en
 * @deprecated since 3.45 along with setVideoEncoderConfig(StreamIndex index, const VideoSolution* solutions, int solution_num) = 0;
 * @type keytype
 * @brief Video stream parameters
 */
struct VideoSolution {
    /**
     * @locale zh
     * @brief 视频宽度，单位：像素
     */
    /**
     * @locale en
     * @brief Width (pixels)
     */
    int width;
    /**
     * @locale zh
     * @brief 视频高度，单位：像素
     */
    /**
     * @locale en
     * @brief Height (pixels)
     */
    int height;
    /**
     * @locale zh
     * @brief 视频帧率
     */
    /**
     * @locale en
     * @brief Video frame rate
     */
    int fps;
    /**
     * @locale zh
     * @brief 最大发送编码码率（kbps），建议使用默认的自动码率。<li>-1: 自动码率</li><li>0: 不开启上限</li><li>>0: 填写预估码率<li>
     */
    /**
     * @locale en
     * @brief Set maximum bitrate in kbps <br>
     *        We recommend the default setting: `SEND_KBPS_AUTO_CALCULATE(-1)` <br>
     *        `SEND_KBPS_DISABLE_VIDEO_SEND(0)` for no limitation for the sending bitrate
     */
    int max_send_kbps = SEND_KBPS_AUTO_CALCULATE;
    int min_send_kbps = 0;
    /**
     * @locale zh
     * @brief 视频编码质量策略，参看 VideoEncodePreference{@link #VideoEncodePreference}
     */
    /**
     * @locale en
     * @brief ncoder preference. See VideoEncodePreference{@link #VideoEncodePreference}.
     */
    VideoEncodePreference encode_preference = VideoEncodePreference::kVideoEncodePreferenceFramerate;
};

/**
 * @locale zh
 * @hidden for internal use only on Windows and Android
 * @type keytype
 * @brief 视野范围（Fov）内的视频帧信息 <br>
 *        Tile 是 全景视频的基本单位。 <br>
 *        视野范围内的视频又分为高清视野和低清背景，均包含了多个 Tile。 <br>
 *        视频帧信息为发送端使用 `setVideoEncoderConfig(const VideoEncoderConfig& encoderConfig, const char* parameters)` 接口进行设置。
 */
/**
 * @locale en
 * @hidden for internal use only on Windows and Android
 * @type keytype
 * @brief Information of video frames within the FoV (Field of View). <br>
 *        Tile is the unit of a video within Fov. <br>
 *        A video within Fov includes HD view and LD background each of which consists of multiple Tiles. <br>
 *        The information of the video frames within the Fov is set by calling `setVideoEncoderConfig(const VideoEncoderConfig& encoderConfig, const char* parameters)` on the sender side.
 */
struct FovVideoTileInfo {
    /**
     * @locale zh
     * @brief 高清视野宽度
     */
    /**
     * @locale en
     * @brief Width of the HD view.
     */
    uint32_t hd_width = 0;
    /**
     * @locale zh
     * @brief 高清视野高度
     */
    /**
     * @locale en
     * @brief Height of the HD view
     */
    uint32_t hd_height = 0;
    /**
     * @locale zh
     * @brief 低清背景宽度
     */
    /**
     * @locale en
     * @brief Width of the LD background
     */
    uint32_t ld_width = 0;
    /**
     * @locale zh
     * @brief 低清背景高度
     */
    /**
     * @locale en
     * @brief Height of the LD background
     */
    uint32_t ld_height = 0;
    /**
     * @locale zh
     * @brief Tile 宽度
     */
    /**
     * @locale en
     * @brief Width of a Tile
     */
    uint32_t tile_width = 0;
    /**
     * @locale zh
     * @brief Tile 高度
     */
    /**
     * @locale en
     * @brief Height of a Tile
     */
    uint32_t tile_height = 0;
    /**
     * @locale zh
     * @brief 高清视野中的 Tile 行数
     */
    /**
     * @locale en
     * @brief Number of Tile rows in the HD view
     */
    uint32_t hd_row = 0;
    /**
     * @locale zh
     * @brief 高清视野中的 Tile 列数
     */
    /**
     * @locale en
     * @brief Number of Tile columns in the HD view
     */
    uint32_t hd_column = 0;
    /**
     * @locale zh
     * @brief 低清背景中的 Tile 行数
     */
    /**
     * @locale en
     * @brief Number of Tile rows in the LD background
     */
    uint32_t ld_row = 0;
    /**
     * @locale zh
     * @brief 低清背景中的 Tile 列数
     */
    /**
     * @locale en
     * @brief Number of Tile columns in the LD background
     */
    uint32_t ld_column = 0;
    /**
     * @locale zh
     * @brief 视野范围中的 Tile 行数
     */
    /**
     * @locale en
     * @brief Number of tile rows within the FoV
     */
    uint32_t dest_row = 0;
    /**
     * @locale zh
     * @brief 视野范围中的 Tile 列数
     */
    /**
     * @locale en
     * @brief Number of tile columns within the FoV
     */
    uint32_t dest_column = 0;
    /**
     * @locale zh
     * @brief Tile 位置映射表
     */
    /**
     * @locale en
     * @brief Position map of the Tiles
     */
    uint8_t* tile_map = nullptr;
    /**
     * @locale zh
     * @brief Tile 数量
     */
    /**
     * @locale en
     * @brief Number of the Tiles
     */
    uint32_t tile_size = 0;
};
/**
 * @locale zh
 * @hidden constructor/destructor
 * @valid since 3.60.
 * @param width 视频宽度
 * @param height 视频高度
 * @type keytype
 * @brief 视频像素
 */
/**
 * @locale en
 * @hidden constructor/destructor
 * @valid since 3.60.
 * @param width Video width
 * @param height Video height
 * @type keytype
 * @brief Video pixel
 */
struct VideoDimensions {
    int width;
    int height;
};

/**
 * @locale zh
 * @hidden constructor/destructor
 */
/**
 * @locale en
 * @hidden constructor/destructor
 */
const static VideoDimensions VD_120x120 = {120, 120};
/**
 * @locale zh
 * @hidden constructor/destructor
 */
/**
 * @locale en
 * @hidden constructor/destructor
 */
const static VideoDimensions VD_160x120 = {160, 120};
/**
 * @locale zh
 * @hidden constructor/destructor
 */
/**
 * @locale en
 * @hidden constructor/destructor
 */
const static VideoDimensions VD_180x180 = {180, 180};
/**
 * @locale zh
 * @hidden constructor/destructor
 */
/**
 * @locale en
 * @hidden constructor/destructor
 */
const static VideoDimensions VD_240x180 = {240, 180};
/**
 * @locale zh
 * @hidden constructor/destructor
 */
/**
 * @locale en
 * @hidden constructor/destructor
 */
const static VideoDimensions VD_320x180 = {320, 180};
/**
 * @locale zh
 * @hidden constructor/destructor
 */
/**
 * @locale en
 * @hidden constructor/destructor
 */
const static VideoDimensions VD_240x240 = {240, 240};
/**
 * @locale zh
 * @hidden constructor/destructor
 */
/**
 * @locale en
 * @hidden constructor/destructor
 */
const static VideoDimensions VD_320x240 = {320, 240};
/**
 * @locale zh
 * @hidden constructor/destructor
 */
/**
 * @locale en
 * @hidden constructor/destructor
 */
const static VideoDimensions VD_424x240 = {424, 240};
/**
 * @locale zh
 * @hidden constructor/destructor
 */
/**
 * @locale en
 * @hidden constructor/destructor
 */
const static VideoDimensions VD_360x360 = {360, 360};
/**
 * @locale zh
 * @hidden constructor/destructor
 */
/**
 * @locale en
 * @hidden constructor/destructor
 */
const static VideoDimensions VD_480x360 = {480, 360};
/**
 * @locale zh
 * @hidden constructor/destructor
 */
/**
 * @locale en
 * @hidden constructor/destructor
 */
const static VideoDimensions VD_640x360 = {640, 360};
/**
 * @locale zh
 * @hidden constructor/destructor
 */
/**
 * @locale en
 * @hidden constructor/destructor
 */
const static VideoDimensions VD_480x480 = {480, 480};
/**
 * @locale zh
 * @hidden constructor/destructor
 */
/**
 * @locale en
 * @hidden constructor/destructor
 */
const static VideoDimensions VD_640x480 = {640, 480};
/**
 * @locale zh
 * @hidden constructor/destructor
 */
/**
 * @locale en
 * @hidden constructor/destructor
 */
const static VideoDimensions VD_840x480 = {840, 480};
/**
 * @locale zh
 * @hidden constructor/destructor
 */
/**
 * @locale en
 * @hidden constructor/destructor
 */
const static VideoDimensions VD_960x540 = {960, 540};
/**
 * @locale zh
 * @hidden constructor/destructor
 */
/**
 * @locale en
 * @hidden constructor/destructor
 */
const static VideoDimensions VD_960x720 = {960, 720};
/**
 * @locale zh
 * @hidden constructor/destructor
 */
/**
 * @locale en
 * @hidden constructor/destructor
 */
const static VideoDimensions VD_1280x720 = {1280, 720};
/**
 * @locale zh
 * @hidden constructor/destructor
 */
/**
 * @locale en
 * @hidden constructor/destructor
 */
const static VideoDimensions VD_1920x1080 = {1920, 1080};
/**
 * @locale zh
 * @hidden constructor/destructor
 */
/**
 * @locale en
 * @hidden constructor/destructor
 */
const static VideoDimensions VD_2540x1440 = {2540, 1440};
/**
 * @locale zh
 * @hidden constructor/destructor
 */
/**
 * @locale en
 * @hidden constructor/destructor
 */
const static VideoDimensions VD_3840x2160 = {3840, 2160};


/**
 * @locale zh
 * @type keytype
 * @brief 推荐的视频帧率
 * @hidden constructor/destructor
 */
/**
 * @locale en
 * @type keytype
 * @brief Recommended video frame rate
 * @hidden constructor/destructor
 */
enum FrameRateFps {
    kFrameRateFps1 = 1,
    kFrameRateFps7 = 7,
    kFrameRateFps10 = 10,
    kFrameRateFps15 = 15,
    kFrameRateFps24 = 24,
    kFrameRateFps30 = 30,
    kFrameRateFps60 = 60,
};
/**
 * @locale zh
 * @type keytype
 * @brief 视频编码配置。参考 [设置视频发布参数](https://www.volcengine.com/docs/6348/70122)
 */
/**
 * @locale en
 * @type keytype
 * @brief  Video encoding configuration. Refer to [Setting Video Encoder Configuration](https://docs.byteplus.com/byteplus-rtc/docs/70122) to learn more.
 */
struct VideoEncoderConfig {
    /**
     * @locale zh
     * @brief 视频宽度，单位：像素
     */
    /**
     * @locale en
     * @brief Width of the video frame in px
     */
    int width;
    /**
     * @locale zh
     * @brief 视频高度，单位：像素
     */
    /**
     * @locale en
     * @brief Height of the video frame in px
     */
    int height;
    /**
     * @locale zh
     * @brief 视频帧率，单位：fps
     */
    /**
     * @locale en
     * @brief Video frame rate in fps
     */
    int frame_rate;
    /**
     * @locale zh
     * @brief 最大编码码率，使用 SDK 内部采集时可选设置，自定义采集时必须设置，单位：kbps。 <br>
     *        内部采集模式下默认值为 -1，即适配码率模式，系统将根据输入的分辨率和帧率自动计算适用的码率。 <br>
     *        设为 0 则不对视频流进行编码发送。
     */
    /**
     * @locale en
     * @brief Maximum bit rate in kbps. Optional for internal capturing while mandatory for custom capturing. <br>
     *        The default value is -1 in internal capturing mode, SDK will automatically calculate the applicable bit rate based on the input resolution and frame rate. <br>
     *        No stream will be encoded and published if you set this parameter to 0.
     */
    int max_bitrate = SEND_KBPS_AUTO_CALCULATE;
    /**
     * @locale zh
     * @brief 视频最小编码码率, 单位 kbps。编码码率不会低于 `minBitrate`。 <br>
     *        默认值为 `0`。 <br>
     *        范围：[0, maxBitrate)，当 `maxBitrate` < `minBitrate` 时，为适配码率模式。 <br>
     *        以下情况，设置本参数无效： <br>
     *        - 当 `maxBitrate` 为 `0` 时，不对视频流进行编码发送。
     *        - 当 `maxBitrate` < `0` 时，适配码率模式。
     */
     /**
     * @locale en
     * @brief Minimum video encoding bitrate in kbps. The encoding bitrate will not be lower than the `minBitrate`. <br>
     *        It defaults to `0`. <br>
     *        It ranges within [0, maxBitrate). When `maxBitrate` < `minBitrate`, the bitrate is self-adpapted. <br>
     *         In the following circumstance, the assignment to this variable has no effect: <br>
     *        - When `maxBitrate` = `0`, the video encoding is disabled.
     *        - When `maxBitrate` < `0`, the bitrate is self-adapted.
     */
    int min_bitrate = 0;
    /**
     * @locale zh
     * @brief 编码策略偏好，默认为帧率优先。参看 VideoEncodePreference{@link #VideoEncodePreference}。
     */
    /**
     * @locale en
     * @brief Encoding preference. The default value is kVideoEncodePreferenceFramerate. See VideoEncodePreference{@link #VideoEncodePreference}.
     */
    VideoEncodePreference encoder_preference = VideoEncodePreference::kVideoEncodePreferenceFramerate;
};

/**
 * @locale zh
 * @hidden
 * @deprecated since 3.36 along with onUserUnPublishStream and onUserUnPublishScreen, and will be deleted in 3.51.
 * @type keytype
 * @brief 视频属性
 */
/**
 * @locale en
 * @hidden
 * @deprecated since 3.45 along with onUserUnPublishStream and onUserUnPublishScreen, and will be deleted in 3.51.
 * @type keytype
 * @brief Video attribute
 */
struct VideoSolutionDescription {
    /**
     * @locale zh
     * @brief 宽（像素） <br>
     *        默认值为 `1920` <br>
     *        为 `0` 时，保持源的宽。
     */
    /**
     * @locale en
     * @brief Width in pixels <br>
     *        The default value is `1920` <br>
     *        `0` for remaining the width of the source.
     */
    int width;
    /**
     * @locale zh
     * @brief 高（像素） <br>
     *        默认值为 `1080` <br>
     *        为 `0` 时，保持源的高。
     */
    /**
     * @locale en
     * @brief High in pixels <br>
     *        The default value is `1080` <br>
     *        `0` for remaining the height of the source.
     */
    int height;
    /**
     * @locale zh
     * @brief 视频帧率(fps)，默认为 15 fps
     */
    /**
     * @locale en
     * @brief Video frame rate in fps. The default value is 15 fps.
     */
    int fps;
    /**
     * @locale zh
     * @brief 最大发送速率（千比特每秒）。 <br>
     *        默认为 `-1`，适配码率模式，系统将根据输入的分辨率和帧率自动计算适用的码率
     */
    /**
     * @locale en
     * @brief Maximum transmission rate in Kbps <br>
     *        The default value is `-1`, Adaptive mode in which RTC will set the bitrate to a value which is calculated according to the target resolution and the frame rate.
     */
    int max_send_kbps;
    int min_send_kbps;
    /**
     * @locale zh
     * @brief 缩放模式。参看 VideoStreamScaleMode{@link #VideoStreamScaleMode}
     */
    /**
     * @locale en
     * @brief Zoom mode. See VideoStreamScaleMode{@link #VideoStreamScaleMode}
     */
    VideoStreamScaleMode scale_mode = VideoStreamScaleMode::kVideoStreamScaleModeAuto;
    /**
     * @locale zh
     * @brief 视频的编码类型。参看 VideoCodecType{@link #VideoCodecType}
     */
    /**
     * @locale en
     * @brief The encoding type of the video. See VideoCodecType{@link #VideoCodecType}
     */
    VideoCodecType codec_name = VideoCodecType::kVideoCodecTypeUnknown;
    /**
     * @locale zh
     * @brief 视频的编码模式。参看 VideoCodecMode{@link #VideoCodecMode}
     */
    /**
     * @locale en
     * @brief The encoding mode of the video. See VideoCodecMode{@link #VideoCodecMode}
     */
    VideoCodecMode codec_mode = VideoCodecMode::kVideoCodecModeAuto;
    /**
     * @locale zh
     * @brief 视频编码质量偏好策略。参看 VideoEncodePreference{@link #VideoEncodePreference}
     */
    /**
     * @locale en
     * @brief Video coding quality preference strategy. See VideoEncodePreference{@link #VideoEncodePreference}
     */
    VideoEncodePreference encode_preference = VideoEncodePreference::kVideoEncodePreferenceFramerate;
    /**
     * @locale zh
     * @hidden constructor/destructor
     */
    /**
     * @locale en
     * @hidden constructor/destructor
     */
    bool operator==(const VideoSolutionDescription& config) const {
        bool result = width == config.width && height == config.height && fps == config.fps
                             && max_send_kbps == config.max_send_kbps && min_send_kbps == config.min_send_kbps
                             && scale_mode == config.scale_mode && codec_name == config.codec_name
                             && codec_mode == config.codec_mode && encode_preference == config.encode_preference;
        return result;
    }
    /**
     * @locale zh
     * @hidden constructor/destructor
     */
    /**
     * @locale en
     * @hidden constructor/destructor
     */
    bool operator!=(const VideoSolutionDescription& config) const {
        bool result = (*this == config);
        return !result;
    }
};


/**
 * @locale zh
 * @deprecated since 3.45 and will be deleted in 3.51.
 * @type keytype
 * @region 视频管理
 */
/**
 * @locale en
 * @deprecated since 3.45 and will be deleted in 3.51.
 * @type keytype
 * @region video management
 */
typedef VideoSolutionDescription VideoProfile;

/**
 * @locale zh
 * @type keytype
 * @brief 视频帧像素格式
 */
/**
 * @locale en
 * @type keytype
 * @brief video frame pixel format
 */
enum VideoPixelFormat {
    /**
     * @locale zh
     * @brief 未知格式
     */
    /**
     * @locale en
     * @brief Unknown format
     */
    kVideoPixelFormatUnknown = 0,
    /**
     * @locale zh
     * @brief YUV I420 格式
     */
    /**
     * @locale en
     * @brief YUV I420
     */
    kVideoPixelFormatI420 = 1,
    /**
     * @locale zh
     * @brief YUV NV12 格式
     */
    /**
     * @locale en
     * @brief YUV NV12
     */
    kVideoPixelFormatNV12 = 2,
    /**
     * @locale zh
     * @brief YUV NV21 格式
     */
    /**
     * @locale en
     * @brief YUV NV21
     */
    kVideoPixelFormatNV21 = 3,
    /**
     * @locale zh
     * @brief RGB 24bit 格式，
     */
    /**
     * @locale en
     * @brief RGB 24bit
     */
    kVideoPixelFormatRGB24 = 4,
    /**
     * @locale zh
     * @brief RGBA 编码格式
     */
    /**
     * @locale en
     * @brief RGBA
     */
    kVideoPixelFormatRGBA = 5,
    /**
     * @locale zh
     * @brief ARGB 编码格式
     */
    /**
     * @locale en
     * @brief ARGB
     */
    kVideoPixelFormatARGB = 6,
    /**
     * @locale zh
     * @brief BGRA 编码格式
     */
    /**
     * @locale en
     * @brief BGRA
     */
    kVideoPixelFormatBGRA = 7,
    
    
    /**
     * @locale zh
     * @hidden for internal use only
     * @brief 像素格式结束标志。新加的格式数值应该小于kVideoPixelFormatEndMark。
     */
    /**
     * @locale en
     * @hidden for internal use only
     */
    kVideoPixelFormatEndMark = 0xFF,
    
    /**
     * @locale zh
     * @brief Texture2D 格式
     */
    /**
     * @locale en
     * @brief Texture2D
     */
    kVideoPixelFormatTexture2D = 0x0DE1,
    /**
     * @locale zh
     * @brief TextureOES 格式
     */
    /**
     * @locale en
     * @brief TextureOES
     */
    kVideoPixelFormatTextureOES = 0x8D65,
};

/**
 * @locale zh
 * @type keytype
 * @brief 视频帧内容类型
 */
/**
 * @locale en
 * @type keytype
 * @brief Video frame content type
 */
enum VideoContentType {
    /**
     * @locale zh
     * @brief 普通视频
     */
    /**
     * @locale en
     * @brief Normal video
     */
    kVideoContentTypeNormalFrame = 0,
    /**
     * @locale zh
     * @brief 黑帧
     */
    /**
     * @locale en
     * @brief Black frame
     */
    kVideoContentTypeBlackFrame = 1,
};


/**
 * @locale zh
 * @type keytype
 * @brief 视频缓冲区类型
 */
/**
 * @locale en
 * @type keytype
 * @brief Video buffer type
 */
enum VideoBufferType {
    /**
     * @locale zh
     * @brief 原始内存数据
     */
    /**
     * @locale en
     * @brief raw memory
     */
    kVideoBufferTypeRawMemory = 0,
    /**
     * @locale zh
     * @hidden(Windows, Android, Linux)
     * @brief CVPixelBufferRef 类型
     */
    /**
     * @locale en
     * @hidden(Windows, Android, Linux)
     * @brief CVPixelBufferRef
     */
    kVideoBufferTypeCVPixelBuffer = 1,
    /**
     * @locale zh
     * @brief OpenGL 纹理数据类型
     */
    /**
     * @locale en
     * @brief OpenGL texture
     */
    kVideoBufferTypeGLTexture = 2,
    /**
     * @locale zh
     * @brief cuda 数据类型
     */
    /**
     * @locale en
     * @brief Cuda data format
     */
    kVideoBufferTypeCuda = 3,
    /**
     * @locale zh
     * @brief direct3d11 纹理
     */
    /**
     * @locale en
     * @brief Direct3d11 texture
     */
    kVideoBufferTypeD3D11 = 4,
    /**
     * @locale zh
     * @brief vaapi 数据格式
     */
    /**
     * @locale en
     * @brief Vaapi data format
     */
    kVideoBufferTypeVAAPI = 5,
	/**
     * @locale zh
     * @hidden(Windows)
     * @brief nvidia jetson dma 数据格式
     */
    /**
     * @locale en
     * @hidden(Windows)
     * @brief nvidia jetson dma
     */
    kVideoBufferTypeNvidiaJetsonDma =6,
};

/**
 * @locale zh
 * @type keytype
 * @brief 自定义内存释放器
 */
/**
 * @locale en
 * @type keytype
 * @brief  Custom Memory Release
 */
struct ManagedMemory {
    /**
     * @locale zh
     * @brief 内存数据地址
     */
    /**
     * @locale en
     * @brief Memory data address
     */
    uint8_t* data = nullptr;
    /**
     * @locale zh
     * @brief 内存空间的大小
     */
    /**
     * @locale en
     * @brief Size of memory space
     */
    int size = 0;
    /**
     * @locale zh
     * @brief 用户自定义数据
     */
    /**
     * @locale en
     * @brief User-defined data
     */
    void* user_opaque = nullptr;
    /**
     * @locale zh
     * @brief 自定义内存释放方法指针，如果指针不为空，方法会被调用，用来释放内存空间。 <br>
     *        函数传入的参数分别是本结构体内 data、size、user_opaque 3 个字段。
     */
    /**
     * @locale en
     * @brief Custom memory free method pointer, if the pointer is not empty, the method will be called to free memory space. The parameters passed in by the <br>
     *         Function are data, size, user_opaque 3 fields in this structure.
     */
    int (*memory_deleter)(uint8_t* data, int size, void* user_opaque) = nullptr;
};

/**
 * @locale zh
 * @type keytype
 * @region 视频管理
 * @brief 视频帧数据
 */
/**
 * @locale en
 * @type keytype
 * @region Video Management
 * @brief Video frame data
 */
typedef struct VideoFrameData {
    /**
     * @locale zh
     * @brief 视频帧缓冲区类型，默认为原始内存，详见 VideoBufferType{@link #VideoBufferType}。必填。
     */
    /**
     * @locale en
     * @brief Video frame buffer type, the default value is raw memory. See VideoBufferType{@link #VideoBufferType}. Required.
     */
    VideoBufferType buffer_type = kVideoBufferTypeRawMemory;
    /**
     * @locale zh
     * @brief 视频帧像素格式，详见 VideoPixelFormat{@link #VideoPixelFormat}。当 `buffer_type` 为 `kVideoFrameTypeGLTexture` 必填。
     */
    /**
     * @locale en
     * @brief Video frame pixel format. See VideoPixelFormat{@link #VideoPixelFormat}. Required, when `buffer_type` is `kVideoBufferTypeRawMemory` or `kVideoFrameTypeGLTexture`.
     */
    VideoPixelFormat pixel_format = kVideoPixelFormatUnknown;

    /**
     * @locale zh
     * @brief 视频帧内容类型，参看 VideoContentType{@link #VideoContentType}
     */
    /**
     * @locale en
     * @brief Video frame content type. See VideoContentType{@link #VideoContentType}
     */
    VideoContentType content_type = kVideoContentTypeNormalFrame;
    /**
     * @locale zh
     * @brief 视频帧平面个数。当 `buffer_type` 为 `kVideoBufferTypeRawMemory` 时，必填。
     */
    /**
     * @locale en
     * @brief Number of video frame plane. Required, `buffer_type` is `kVideoBufferTypeRawMemory`.
     */
    int number_of_planes = 0;
    /**
     * @locale zh
     * @brief 视频帧平面数组。当 `buffer_type` 为 `kVideoBufferTypeRawMemory` 时，必填。
     */
    /**
     * @locale en
     * @brief Array of video frame plane. Required, when `buffer_type` is `kVideoBufferTypeRawMemory`.
     */
    uint8_t* plane_data[4] = { nullptr };
    /**
     * @locale zh
     * @brief stride 数组。stride 指视频帧平面相邻两行图像数据之间的内存长度（单位字节）。当 `buffer_type` 为 `kVideoBufferTypeRawMemory` 时，必填。
     */
    /**
     * @locale en
     * @brief Array of stride. Stride is the length of memory between two lines of image data in video frame plane in bytes. Required, when `buffer_type` is `kVideoBufferTypeRawMemory`.
     */
    int plane_stride[4] = { 0 };
    /**
     * @locale zh
     * @brief SEI 数据
     */
    /**
     * @locale en
     * @brief SEI data
     */
    uint8_t* sei_data = nullptr;
    /**
     * @locale zh
     * @brief SEI 数据大小
     */
    /**
     * @locale en
     * @brief SEI data size
     */
    int sei_data_size = 0;
    /**
     * @locale zh
     * @brief 视频帧感兴趣区域数据
     */
    /**
     * @locale en
     * @brief ROI(Region of Interest) Data in a video frame
     */
    uint8_t* roi_data = nullptr;
    /**
     * @locale zh
     * @brief 视频帧感兴趣区域数据数据长度
     */
    /**
     * @locale en
     * @brief Data data length of ROI(Region of Interest) Data in a video frame
     */
    int roi_data_size = 0;
    /**
     * @locale zh
     * @brief 视频帧宽度。必填。
     */
    /**
     * @locale en
     * @brief Video frame width. Required.
     */
    int width = 0;
    /**
     * @locale zh
     * @brief 视频帧高度。必填。
     */
    /**
     * @locale en
     * @brief Video frame height. Required.
     */
    int height = 0;
    /**
     * @locale zh
     * @brief 视频旋转角度，参看 VideoRotation{@link #VideoRotation}。 <br>
     */
    /**
     * @locale en
     * @brief Video rotation angle, see VideoRotation{@link #VideoRotation}. <br>
     */
    VideoRotation rotation = kVideoRotation0;
    /**
     * @locale zh
     * @brief 视频帧时间戳，单位：微秒。必填。
     */
    /**
     * @locale en
     * @brief Video frame timestamp in ms. Required.
     */
    int64_t timestamp_us = 0;
    /**
     * @locale zh
     * @brief 硬件加速缓冲区指针
     */
    /**
     * @locale en
     * @brief Hardware acceleration buffer pointer
     */
    void* hw_buffer = nullptr;
    /**
    * @locale zh
    * @brief 硬件加速上下文对象句柄
    */
    /**
     * @locale en
     * @brief Hardware acceleration context object handle
     */
    void* hw_context = nullptr;
    /**
     * @locale zh
     * @brief 纹理矩阵(仅针对纹理类型的 frame 生效)
     */
    /**
     * @locale en
     * @brief Texture matrix (only for texture type frame)
     */
    float texture_matrix[16] = { 0 };
    /**
     * @locale zh
     * @brief 纹理 ID(仅针对纹理类型的 frame 生效)。当 `buffer_type` 为 `kVideoBufferTypeRawMemory` 或 `kVideoFrameTypeGLTexture` 必填。
     */
    /**
     * @locale en
     * @brief Texture ID (only for texture type frame). Required, when `buffer_type` is `kVideoBufferTypeRawMemory` or `kVideoFrameTypeGLTexture`.
     */
    uint32_t texture_id = 0;
    
} VideoFrameData;

/**
 * @locale zh
 * @type keytype
 * @brief 视频帧接口
 */
/**
 * @locale en
 * @type keytype
 * @brief video frame interface
 */
class IVideoFrame {
public:
    /**
     * @locale zh
     * @brief 获取视频帧缓冲区类型，参看 VideoBufferType{@link #VideoBufferType}。
     */
    /**
     * @locale en
     * @brief Gets video frame buffer type, see VideoBufferType{@link #VideoBufferType}.
     */
    virtual VideoBufferType bufferType() = 0;
    /**
     * @locale zh
     * @brief 获取视频帧像素格式，参看 VideoPixelFormat{@link #VideoPixelFormat}。
     */
    /**
     * @locale en
     * @brief Gets video frame's pixel format, see VideoPixelFormat{@link #VideoPixelFormat}.
     */
    virtual VideoPixelFormat pixelFormat() = 0;
    /**
     * @locale zh
     * @brief 获取视频帧内容类型，参看 VideoContentType{@link #VideoContentType}。
     */
    /**
     * @locale en
     * @brief Gets video frame content type, see VideoContentType{@link #VideoContentType}.
     */
    virtual VideoContentType contentType() = 0;

    /**
     * @locale zh
     * @brief 获取视频帧时间戳，单位：微秒
     */
    /**
     * @locale en
     * @brief Gets video frame timestamp in microseconds
     */
    virtual int64_t timestampUs() = 0;
    /**
     * @locale zh
     * @brief 获取视频帧宽度
     */
    /**
     * @locale en
     * @brief Gets video frame width
     */
    virtual int width() = 0;
    /**
     * @locale zh
     * @brief 获取视频帧高度
     */
    /**
     * @locale en
     * @brief Gets video frame height
     */
    virtual int height() = 0;
    /**
     * @locale zh
     * @brief 获取视频帧旋转角度，参看 VideoRotation{@link #VideoRotation}
     */
    /**
     * @locale en
     * @brief Gets the video frame rotation angle, see VideoRotation{@link #VideoRotation}
     */
    virtual VideoRotation rotation() = 0;
    /**
     * @locale zh
     * @brief 获取视频帧平面数量
     */
    /**
     * @locale en
     * @brief Gets the number of video frame plane
     */
    virtual int numberOfPlanes() = 0;
    /**
     * @locale zh
     * @brief 获取视频帧平面
     * @param plane_index plane 索引
     */
    /**
     * @locale en
     * @brief Gets video frame plane
     * @param plane_index plane index
     */
    virtual uint8_t* planeData(int plane_index) = 0;
    /**
     * @locale zh
     * @brief 获取视频帧平面相邻两行图像数据之间的内存长度（单位字节）
     * @param plane_index plane 索引
     */
    /**
     * @locale en
     * @brief Get the length of memory between two lines of image data in video frame plane in bytes.
     * @param plane_index Plane index
     */
    virtual int planeStride(int plane_index) = 0;
    /**
     * @locale zh
     * @brief 获取 SEI 数据指针
     * @param size SEI 数据字节数
     */
    /**
     * @locale en
     * @brief Gets SEI data pointer
     * @param size Byte count of SEI data
     */
    virtual uint8_t* seiData(int& size)  = 0;  // NOLINT
    /**
     * @locale zh
     * @brief 获取硬件缓冲区指针
     */
    /**
     * @locale en
     * @brief Gets hardware buffer pointer
     */
    virtual void* hwBuffer() = 0;
    /**
     * @locale zh
     * @brief 获取硬件加速上下文对象句柄
     */
    /**
     * @locale en
     * @brief Gets hardware acceleration context object handle
     */
    virtual void* hwContext() = 0;
    /**
     * @locale zh
     * @brief 获取纹理矩阵(仅针对纹理类型的 frame 生效)
     * @param matrix 纹理矩阵
     */
    /**
     * @locale en
     * @brief Get Texture matrix (only for texture type frame)
     * @param matrix Texture matrix
     */
    virtual void textureMatrix(float matrix[16]) = 0;
    /**
     * @locale zh
     * @brief 获取纹理 ID(仅针对纹理类型的 frame 生效)
     */
    /**
     * @locale en
     * @brief Get Texture ID (only for texture type frame)
     */
    virtual uint32_t textureId() = 0;
    /**
     * @locale zh
     * @brief 视频帧引用计数加一
     * @note 视频帧消费者希望对视频帧进行异步处理时（例如切换线程进行渲染），需要调用此接口增加引用计数。异步处理结束则需要调用 `releaseRef` 使引用计数减1
     */
    /**
     * @locale en
     * @brief Increase the reference count of the video frame by 1.
     * @note If you wants to asynchronously process the video frame (for example, switch the thread for rendering), you need to call this interface to increase the reference count. After the asynchronous processing is completed, you need to call `releaseRef` to reduce the reference count by 1.
     */
    virtual void addRef() = 0;
    /**
     * @locale zh
     * @brief  视频帧引用计数减一
     * @note 视频帧引用计数减为 0 时，视频帧对象会被释放。视频帧对象释放后，不应该继续使用视频帧。
     */
    /**
     * @locale en
     * @brief Decrease the reference count of the video frame by 1.
     * @note When the reference count of a video frame is decreased to 0, the video frame object is released. After the video frame is released, it should not be used anymore.
     */
    virtual long releaseRef() = 0;
    /**
     * @locale zh
     * @brief 获取视频帧的摄像头位置信息，参看 CameraID{@link #CameraID}
     */
    /**
     * @locale en
     * @brief Gets camera position info of the video frame, see CameraID{@link #CameraID}
     */
    virtual CameraID cameraId()  = 0;
    /**
     * @locale zh
     * @hidden for internal use only on Windows and Android
     * @type api
     * @brief 获取全景视频的 Tile 信息
     * @return FoV（可视范围）随本端的头位姿实时更新获取到的视频帧，包括高清视野和低清背景。参见 FovVideoTileInfo{@link #FovVideoTileInfo}。
     * @list 
     */
    /**
     * @locale en
     * @hidden for internal use only on Windows and Android
     * @type api
     * @brief Get Tile information from the panoramic video frames to enable the FoV (Field of View).
     * @return Video frames in the FoV(filed of view) accroding to the head pose. Refer to FovVideoTileInfo{@link #FovVideoTileInfo} for more details.
     * @list 
     */
    virtual FovVideoTileInfo fovTileInfo() = 0;
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
    virtual ~IVideoFrame() = default;
};

// /**
//  * @locale zh
//  * @type api
//  * @region 视频管理
//  * @brief 创建 IVideoFrame
//  * @param builder 视频帧构建实例，参看 VideoFrameData{@link #VideoFrameData}
//  * @return IVideoFrame{@link #IVideoFrame} 实例
//  * @list 视频管理
//  * @order 10
//  */
// /**
//  * @locale en
//  * @type api
//  * @region  video management
//  * @brief  Create IVideoFrame
//  * @param builder Video frame build instance. See VideoFrameData{@link #VideoFrameData}
//  * @return IVideoFrame{@link #IVideoFrame} instance
//  */
// BYTERTC_API IVideoFrame* buildVideoFrame(const VideoFrameData& builder);

/**
 * @locale zh
 * @type keytype
 * @region 视频管理
 * @brief 视频帧参数
 */
/**
 * @locale en
 * @type keytype
 * @region  video management
 * @brief  Video frame parameters
 */
typedef struct EncodedVideoFrameBuilder {
    /**
     * @locale zh
     * @brief 视频帧编码格式，参看 VideoCodecType{@link #VideoCodecType}
     */
    /**
     * @locale en
     * @brief Video frame encoding format. See VideoCodecType{@link #VideoCodecType}
     */
    VideoCodecType codec_type = kVideoCodecTypeUnknown;
    /**
     * @locale zh
     * @brief 视频帧编码类型，参看 VideoPictureType{@link #VideoPictureType}
     */
    /**
     * @locale en
     * @brief Video frame encoding type. See VideoPictureType{@link #VideoPictureType}
     */
    VideoPictureType picture_type = kVideoPictureTypeUnknown;
    /**
     * @locale zh
     * @brief 视频帧旋转角度，参看 VideoRotation{@link #VideoRotation}
     */
    /**
     * @locale en
     * @brief Video frame rotation angle. See VideoRotation{@link #VideoRotation}
     */
    VideoRotation rotation = kVideoRotation0;
    /**
     * @locale zh
     * @brief 视频帧数据指针
     * @note IEncodedVideoFrame 会获取数据的所有权
     */
    /**
     * @locale en
     * @brief Video frame data pointer
     * @note IEncodedVideoFrame  takes ownership of the data
     */
    uint8_t* data = nullptr;
    /**
     * @locale zh
     * @brief 视频帧数据大小
     */
    /**
     * @locale en
     * @brief Video frame data size
     */
    int size = 0;
    /**
     * @locale zh
     * @brief 视频帧宽度，单位：px
     */
    /**
     * @locale en
     * @brief Video frame width in px
     */
    int width = 0;
    /**
     * @locale zh
     * @brief 视频帧高度，单位：px
     */
    /**
     * @locale en
     * @brief Video frame height in px
     */
    int height = 0;
    /**
     * @locale zh
     * @brief 视频采集时间戳，单位：微秒
     */
    /**
     * @locale en
     * @brief Video capture timestamp in microseconds
     */
    int64_t timestamp_us = 0;
    /**
     * @locale zh
     * @brief 视频编码时间戳，单位：微秒
     */
    /**
     * @locale en
     * @brief Video encoding timestamp in microseconds
     */
    int64_t timestamp_dts_us = 0;
    /**
     * @locale zh
     * @brief 用户定义的视频帧释放回调函数指针，如果指针不为空，方法会被调用用来释放内存空间
     */
    /**
     * @locale en
     * @brief The user-defined video frame releases the callback function pointer. If the pointer is not empty, the method will be called to free memory space
     */
    int (*memory_deleter)(uint8_t* data, int size, void* user_opaque) = nullptr;
} EncodedVideoFrameBuilder;
/**
 * @locale zh
 * @type keytype
 * @brief 视频帧信息
 */ 
/**
 * @locale en
 * @type keytype
 * @brief Information about video frames
 */
class IEncodedVideoFrame {
public:
    /**
     * @locale zh
     * @type api
     * @brief 获取视频编码类型
     * @return 视频编码类型，参看 VideoCodecType{@link #VideoCodecType}
     * @list Video Management
     */
    /**
     * @locale en
     * @type api
     * @brief Gets the video encoding type
     * @return Video encoding type. See VideoCodecType{@link #VideoCodecType}
     * @list Video Management
     */
    virtual VideoCodecType codecType() const = 0;
    /**
     * @locale zh
     * @type api
     * @brief 获取视频采集时间戳
     * @return 视频采集时间戳，单位：微秒
     * @list Video Management
     */
    /**
     * @locale en
     * @type api
     * @brief Gets video capture timestamp
     * @return Video capture timestamp in microseconds
     * @list Video Management
     */
    virtual int64_t timestampUs() const = 0;
    /**
     * @locale zh
     * @type api
     * @brief 获取视频编码时间戳
     * @return 视频编码时间戳，单位：微秒
     * @list Video Management
     */
    /**
     * @locale en
     * @type api
     * @brief Gets video encoding timestamp
     * @return Video encoding timestamp in microseconds
     * @list Video Management
     */
    virtual int64_t timestampDtsUs() const = 0;
    /**
     * @locale zh
     * @type api
     * @brief 获取视频帧宽度
     * @return 视频帧宽度，单位：px
     * @list Video Management
     */
    /**
     * @locale en
     * @type api
     * @brief Get videos frame width
     * @return Video frame width in px
     * @list Video Management
     */
    virtual int width() const = 0;
    /**
     * @locale zh
     * @type api
     * @brief 获取视频帧高度
     * @return 视频帧高度，单位：px
     * @list Video Management
     */
    /**
     * @locale en
     * @type api
     * @brief Gets video frame height
     * @return Video frame height in px
     * @list Video Management
     */
    virtual int height() const = 0;
    /**
     * @locale zh
     * @type api
     * @brief 获取视频编码帧类型
     * @return 视频编码帧类型，参看 VideoPictureType{@link #VideoPictureType}
     * @list Video Management
     */
    /**
     * @locale en
     * @type api
     * @brief Gets video compression picture type.
     * @return Video compression picture type.See VideoPictureType{@link #VideoPictureType}
     * @list Video Management
     */
    virtual VideoPictureType pictureType() const = 0;
    /**
     * @locale zh
     * @type api
     * @brief 获取视频帧旋转角度
     * @return 视频帧旋转角度，参看 VideoRotation{@link #VideoRotation}
     * @list Video Management
     */
    /**
     * @locale en
     * @type api
     * @brief Gets video frame rotation angle
     * @return Video frame rotation angle. See VideoRotation{@link #VideoRotation}
     * @list Video Management
     */
    virtual VideoRotation rotation() const = 0;
    /**
     * @locale zh
     * @type api
     * @brief 获取视频帧数据指针地址
     * @return 视频帧数据指针地址
     * @list Video Management
     */
    /**
     * @locale en
     * @type api
     * @brief Get the pointer to the video frame
     * @return The pointer to the video frame
     * @list Video Management
     */
    virtual uint8_t* data() const = 0;
    /**
     * @locale zh
     * @type api
     * @brief 获取视频帧数据大小
     * @return 视频帧数据大小
     * @list Video Management
     */
    /**
     * @locale en
     * @type api
     * @brief Gets video frame data size
     * @return Video frame data size
     * @list Video Management
     */
    virtual int dataSize() const = 0;
    /**
     * @locale zh
     * @type api
     * @brief 浅拷贝视频帧并返回指针
     * @list Video Management
     */
    /**
     * @locale en
     * @type api
     * @brief Makes shallow copies of video frame and return pointer
     * @list Video Management
     */
    virtual IEncodedVideoFrame* shallowCopy() const = 0;
    /**
     * @locale zh
     * @type api
     * @brief 释放视频帧
     * @list Video Management
     */
    /**
     * @locale en
     * @type api
     * @brief Releases video frame
     * @list Video Management
     */
    virtual void release() = 0;
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
    virtual ~IEncodedVideoFrame() = default;
};
/**
 * @locale zh
 * @type api
 * @brief 创建 IEncodedVideoFrame
 * @param builder 编码后的视频帧构建实例，参看 EncodedVideoFrameBuilder{@link #EncodedVideoFrameBuilder}
 * @return IEncodedVideoFrame{@link #IEncodedVideoFrame} 实例
 * @list 视频管理
 */
/**
 * @locale en
 * @type api
 * @brief  Create IEncodedVideoFrame
 * @param builder Encoded video frame build instance. See EncodedVideoFrameBuilder{@link #EncodedVideoFrameBuilder}
 * @return IEncodedVideoFrame{@link #IEncodedVideoFrame} instance
 * @list Video Management
 */
BYTERTC_API IEncodedVideoFrame* buildEncodedVideoFrame(const EncodedVideoFrameBuilder& builder);


/**
 * @locale zh
 * @hidden(Linux,Android,iOS)
 * @type keytype
 * @brief RTC 智能决策后得到的帧率和分辨率积（宽*高）。
 */
/**
 * @locale en
 * @hidden(Linux,Android,iOS)
 * @type keytype
 * @brief  The recommended pixel and framerate by RTC.
 */
struct FrameUpdateInfo {
    /**
     * @locale zh
     * @brief 分辨率积（宽*高）。
     */
    /**
     * @locale en
     * @brief Pixel (width * height).
     */
    int pixel;
    /**
     * @locale zh
     * @brief 帧率。
     */
    /**
     * @locale en
     * @brief The frame rate.
     */
    int framerate;
};

}  // namespace bytertc
