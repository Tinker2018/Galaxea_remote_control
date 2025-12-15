/*
 * Copyright (c) 2021 The VolcEngineRTC project authors. All Rights Reserved.
*/

#pragma once

#include <stdint.h>

namespace bytertc {
/**
 * @locale zh
 * @type keytype
 * @brief 相机补光灯状态
 */
/**
 * @locale en
 * @type keytype
 * @brief Camera fill light status
 */
enum TorchState{

    /**
     * @locale zh
     * @brief 相机补光灯关闭
     */
    /**
     * @locale en
     * @brief Camera fill light off
     */
    kTorchStateOff=0,
    /**
     * @locale zh
     * @brief 相机补光灯打开
     */
    /**
     * @locale en
     * @brief Camera fill light on
     */
    kTorchStateOn=1,
};
/**
 * @locale zh
 * @hidden(macOS,Windows,Linux)
 * @type api
 * @brief 相机控制接口
 * @note 必须已调用 StartVideoCapture{@link #IRTCEngine#StartVideoCapture} 使用 SDK 内部采集模块进行视频采集时，才能使用这个接口。
 */
/**
 * @locale en
 * @hidden(macOS,Windows,Linux)
 * @type api
 * @brief camera control interface
 * @note must have been called StartVideoCapture{@link #IRTCEngine#StartVideoCapture} to use this interface when using the SDK internal capture module for video capture.
 */
class ICameraControl {
public:
    virtual ~ICameraControl() = default;
    /**
     * @locale zh
     * @type api
     * @brief 设置当前使用的摄像头（前置/后置）的变焦倍数
     * @param zoom_val 变焦倍数。取值范围是 [1, <最大变焦倍数>]。 <br>
     *                 最大变焦倍数可以通过调用 getCameraZoomMaxRatio{@link #getCameraZoomMaxRatio} 获取。
     * @return
     *        - 0： 成功。
     *        - < 0： 失败。
     * @note
     *        - 必须已调用 StartVideoCapture{@link #IRTCEngine#StartVideoCapture} 使用 SDK 内部采集模块进行视频采集时，才能设置摄像头变焦倍数。
     *        - 设置结果在调用 StopVideoCapture{@link #IRTCEngine#StopVideoCapture} 关闭内部采集后失效。
     */
    /**
     * @locale en
     * @type api
     * @brief set the zoom factor of the currently used camera (front/postcondition)
     * @param zoom_val  zoom factor. The value range is [1, < Maximum Zoom Multiplier >]. <br>
     *                  The maximum zoom factor can be obtained by calling getCameraZoomMaxRatio{@link #getCameraZoomMaxRatio}.
     * @return
     *         - 0: Success.
     *         - < 0: Failure.
     * @note
     *         - You must have called StartVideoCapture {@link #IRTCEngine#StartVideoCapture} for video capture using the SDK internal capture module to set the camera zoom factor.
     *         - The setting result fails after calling StopVideoCapture{@link #IRTCEngine#StopVideoCapture} to turn off internal collection.
     */
    virtual int setCameraZoomRatio(float zoom_val) = 0;
    /**
     * @locale zh
     * @type api
     * @brief 获取当前使用的摄像头（前置/后置）的最大变焦倍数
     * @return 最大变焦倍数
     * @note 必须已调用 StartVideoCapture{@link #IRTCEngine#StartVideoCapture} 使用 SDK 内部采集模块进行视频采集时，才能检测摄像头最大变焦倍数。
     */
    /**
     * @locale en
     * @type api
     * @brief Get the maximum zoom multiple of the currently used camera (front/postcondition)
     * @return Maximum zoom multiple
     * @note You must have called StartVideoCapture {@link #IRTCEngine#StartVideoCapture} When using the SDK internal capture module for video capture, the camera maximum zoom multiple can be detected.
     */
    virtual float getCameraZoomMaxRatio() = 0;
    /**
     * @locale zh
     * @type api
     * @brief 检测当前使用的摄像头（前置/后置），是否支持变焦（数码/光学变焦）。
     * @return
     *        - true: 支持
     *        - false: 不支持
     * @note 必须已调用 StartVideoCapture{@link #IRTCEngine#StartVideoCapture} 使用 SDK 内部采集模块进行视频采集时，才能检测摄像头变焦能力。
     */
    /**
     * @locale en
     * @type api
     * @brief Detect whether the currently used camera (front/postcondition) supports zoom (digital/optical zoom).
     * @return
     *         - true: Support
     *         - false: Not supported
     * @note  You must have called StartVideoCapture{@link #IRTCEngine#StartVideoCapture} for video capture using the SDK internal capture module to detect camera zoom capability.
     */
    virtual bool isCameraZoomSupported() = 0;
    /**
     * @locale zh
     * @type api
     * @brief 检测当前使用的摄像头（前置/后置），是否支持闪光灯。
     * @return
     *        - true: 支持
     *        - false: 不支持
     * @note 必须已调用 StartVideoCapture{@link #IRTCEngine#StartVideoCapture} 使用 SDK 内部采集模块进行视频采集时，才能检测闪光能力。
     */
    /**
     * @locale en
     * @type api
     * @brief  detect the currently used camera (front/postcondition), whether flash is supported.
     * @return
     *         - true: Support
     *         - false: Not supported
     * @note  You must have called StartVideoCapture{@link #IRTCEngine#StartVideoCapture} for video capture using the SDK internal capture module to detect flash capability.
     */
    virtual bool isCameraTorchSupported() = 0;
    /**
     * @locale zh
     * @type api
     * @brief 打开/关闭当前使用的摄像头（前置/后置）的闪光灯
     * @param torch_state 打开/关闭。参看 TorchState{@link #TorchState}。
     * @return
     *        - 0： 成功。
     *        - < 0： 失败。
     * @note
     *        - 必须已调用 StartVideoCapture{@link #IRTCEngine#StartVideoCapture} 使用 SDK 内部采集模块进行视频采集时，才能设置闪光灯。
     *        - 设置结果在调用 StopVideoCapture{@link #IRTCEngine#StopVideoCapture} 关闭内部采集后失效。
     */
    /**
     * @locale en
     * @type api
     * @brief turn on/off the flash of the currently used camera (front/postcondition)
     * @param torch_state turn on/off. See TorchState{@link #TorchState}.
     * @return
     *         - 0: Success.
     *         - < 0: Failure.
     * @note
     *         - You must have called StartVideoCapture{@link #IRTCEngine#StartVideoCapture} for video capture using the SDK internal capture module to set the flash.
     *         - The setting result fails after calling StopVideoCapture{@link #IRTCEngine#StopVideoCapture} to turn off internal collection.
     */
    virtual int setCameraTorch(TorchState torch_state) = 0;
    /**
     * @locale zh
     * @type api
     * @brief 检查当前使用的摄像头是否支持对焦点设置
     * @return
     *        - true： 支持。
     *        - false： 不支持。
     * @note 必须已调用 StartVideoCapture{@link #IRTCEngine#StartVideoCapture} 使用 SDK 内部采集模块进行视频采集时，才能检查对焦点设置能力。
     */
    /**
     * @locale en
     * @type api
     * @brief Checks if the camera currently in use supports setting the focus point.
     * @return
     *        - true: Yes.
     *        - false: No.
     * @note Call StartVideoCapture{@link #IRTCEngine#StartVideoCapture} before calling this API.
     */
    virtual bool isCameraFocusPositionSupported() = 0;
    /**
     * @locale zh
     * @type api
     * @brief 设置当前使用的摄像头的对焦点
     * @param x 对焦点水平方向归一化坐标。以本地预览画布的左上为原点。取值范围为[0, 1]，0 为最左边，1 为最右边。
     * @param y 对焦点垂直方向归一化坐标。以本地预览画布的左上为原点。取值范围为[0, 1]，0 为最上边，1 为最下边。
     * @return
     *        - 0： 成功。
     *        - < 0： 失败。
     * @note
     *        - 必须已调用 StartVideoCapture{@link #IRTCEngine#StartVideoCapture} 使用 SDK 内部采集模块进行视频采集，并且使用 SDK 内部渲染时，才能设置对焦点。
     *        - 对焦点设置为画布中央(即[0.5， 0.5]) 时，恢复系统默认值。
     *        - 设置结果在调用 StopVideoCapture{@link #IRTCEngine#StopVideoCapture} 关闭内部采集后失效。
     */
    /**
     * @locale en
     * @type api
     * @brief Sets the focus point of the currently used camera.
     * @param x The x-coordinate of the focus point in range of [0, 1], taking the upper-left corner of the canvas as the origin.
     * @param y The y-coordinate of the focus point in range of [0, 1], taking the upper-left corner of the canvas as the origin.
     * @return
     *        - 0: Success.
     *        - < 0: Failure
     * @note
     *        - Call StartVideoCapture{@link #IRTCEngine#StartVideoCapture} before calling this API.
     *        - When the focus point is set to the center of the canvas (i.e., [0.5, 0.5]), the system default value is restored.
     *        - The setting expires after calling StopVideoCapture{@link #IRTCEngine#StopVideoCapture} to disable the internal capture.
     */
    virtual int setCameraFocusPosition(float x, float y) = 0;
    /**
     * @locale zh
     * @type api
     * @brief 检查当前使用的摄像头是否支持曝光点设置
     * @return
     *        - true： 支持。
     *        - false： 不支持。
     * @note 必须已调用 StartVideoCapture{@link #IRTCEngine#StartVideoCapture} 使用 SDK 内部采集模块进行视频采集时，才能检查曝光点设置能力。
     */
    /**
     * @locale en
     * @type api
     * @brief Checks if the camera currently in use supports setting the exposure point.
     * @return
     *        - true: Yes.
     *        - false: No.
     * @note Call StartVideoCapture{@link #IRTCEngine#StartVideoCapture} before calling this API.
     */
    virtual bool isCameraExposurePositionSupported() = 0;
    /**
     * @locale zh
     * @type api
     * @brief 设置当前使用的摄像头的曝光点
     * @param x 曝光点水平方向归一化坐标。以本地预览画布的左上为原点。取值范围为[0, 1]，0 为最左边，1 为最右边。
     * @param y 曝光点垂直方向归一化坐标。以本地预览画布的左上为原点。取值范围为[0, 1]，0 为最上边，1 为最下边。
     * @return
     *        - 0： 成功。
     *        - < 0： 失败。
     * @note
     *        - 必须已调用 StartVideoCapture{@link #IRTCEngine#StartVideoCapture} 使用 SDK 内部采集模块进行视频采集，并且使用 SDK 内部渲染时，才能设置曝光点。
     *        - 曝光点设置为画布中央(即[0.5， 0.5]) 时，恢复系统默认值。
     *        - 设置结果在调用 StopVideoCapture{@link #IRTCEngine#StopVideoCapture} 关闭内部采集后失效。
     */
    /**
     * @locale en
     * @type api
     * @brief Sets the exposure point of the currently used camera.
     * @param x The x-coordinate of the exposure point in range of [0, 1], taking the upper-left corner of the canvas as the origin.
     * @param y The y-coordinate of the exposure point in range of [0, 1], taking the upper-left corner of the canvas as the origin.
     * @return
     *        - 0: Success.
     *        - < 0: Failure.
     * @note
     *        - Call StartVideoCapture{@link #IRTCEngine#StartVideoCapture} before calling this API.
     *        - When the exposure point is set to the center of the canvas (i.e., [0.5, 0.5]), the system default value is restored.
     *        - The setting expires after calling StopVideoCapture{@link #IRTCEngine#StopVideoCapture}.
     */
    virtual int setCameraExposurePosition(float x, float y) = 0;
    /**
     * @locale zh
     * @type api
     * @brief Sets the exposure point of the currently used camera.
     * @param val 曝光补偿值，取值范围[-1, 1]，0 为系统默认值(没有曝光补偿)。
     * @return
     *        - 0： 成功。
     *        - < 0： 失败。
     * @note
     *        - 必须已调用 StartVideoCapture{@link #IRTCEngine#StartVideoCapture} 使用 SDK 内部采集模块进行视频采集时，才能设置曝光补偿。
     *        - 设置结果在调用 StopVideoCapture{@link #IRTCEngine#StopVideoCapture} 关闭内部采集后失效。
     */
    /**
     * @locale en
     * @type api
     * @brief Sets the exposure compensation for the currently used camera.
     * @param val Exposure compensation in range of [-1, 1]. Default to 0, which means no exposure compensation.
     * @return
     *        - 0: Success.
     *        - < 0: Failure.
     * @note
     *        - Call StartVideoCapture{@link #IRTCEngine#StartVideoCapture} before calling this API.
     *        - The setting expires after calling StopVideoCapture{@link #IRTCEngine#StopVideoCapture}.
     */
    virtual int setCameraExposureCompensation(float val) = 0;
    /**
     * @locale zh
     * @hidden(macOS, Windows, Linux)
     * @valid since 353
     * @type api
     * @brief 启用或禁用内部采集时人脸自动曝光模式。此模式会改善强逆光下，脸部过暗的问题；但也会导致 ROI 以外区域过亮/过暗的问题。
     * @param enable 是否启用。iOS默认开启，Android默认关闭。
     * @return
     *        - 0: 成功.
     *        - !0: 失败.
     * @note
     *        - 对 Android，你必须在调用 startVideoCapture{@link #IRTCEngine#startVideoCapture} 开启内部采集前，调用此接口方可生效。
     *        - 对 iOS，在采集前或采集中调用此接口均可生效。
     * @list Video Management
     */
    /**
     * @locale en
     * @hidden(macOS, Windows, Linux)
     * @valid since 353
     * @type api
     * @brief Enable or disable face auto exposure mode during internal video capture. This mode fixes the problem that the face is too dark under strong backlight; but it will also cause the problem of too bright/too dark in the area outside the ROI region.
     * @param enable. Whether to enable the mode. True by default for iOS, False by default for Android.
     * @return
     *        - 0: Success.
     *        - !0: Failure.
     * @note
     *        - For Android, you must call this API before calling startVideoCapture{@link #IRTCEngine#startVideoCapture} to enable internal capture to make the setting valid.
     *        - For iOS, calling this API takes effect immediately whether before or after internal video capturing.
     * @list Video Management
     */
    virtual int enableCameraAutoExposureFaceMode(bool enable) = 0;
    /**
     * @locale zh
     * @hidden(macOS, Windows, Linux)
     * @valid since 353
     * @type api
     * @brief 设置内部采集适用动态帧率时，帧率的最小值。
     * @param framerate 最小值。单位为 fps。默认值是 7。 <br>
     *                  动态帧率的最大帧率是通过 setVideoCaptureConfig{@link #IRTCEngine#setVideoCaptureConfig} 设置的帧率值。当传入参数大于最大帧率时，使用固定帧率模式，帧率为最大帧率；当传入参数小于最大帧率时，使用动态帧率。
     * @return
     *        - 0: 成功.
     *        - !0: 失败.
     * @note
     *        - 你必须在调用 startVideoCapture{@link #IRTCEngine#startVideoCapture} 开启内部采集前，调用此接口方可生效。
     *        - 如果由于性能降级、静态适配等原因导致采集最大帧率变化时，已设置的最小帧率值会与新的采集最大帧率值重新比较。比较结果变化可能导致固定/动态帧率模式切换。
     *        - 对 Android，默认开启动态帧率模式
     *        - 对 iOS，默认使用固定帧率模式
     * @list Video Management
     */
    /**
     * @locale en
     * @hidden(macOS, Windows, Linux)
     * @valid since 353
     * @type api
     * @brief Set the minimum frame rate of of the dynamic framerate mode during internal video capture.
     * @param framerate The minimum value in fps. The default value is 7. <br>
     *                  The maximum value of the dynamic framerate mode is set by calling setVideoCaptureConfig{@link #IRTCEngine#setVideoCaptureConfig}. When minimum value exceeds the maximum value, the frame rate is set to a fixed value as the maximum value; otherwise, dynamic framerate mode is enabled.
     * @return
     *        - 0: Success.
     *        - !0: Failure.
     * @note
     *        - You must call this API before calling startVideoCapture{@link #IRTCEngine#startVideoCapture} to enable internal capture to make the setting valid.
     *        - If the maximum frame rate changes due to performance degradation, static adaptation, etc., the set minimum frame rate value will be re-compared with the new maximum value. Changes in comparison results may cause switch between fixed and dynamic frame rate modes.
     *        - For Android, dynamic framerate mode is enabled.
     *        - For iOS, dynamic framerate mode is disabled.
     * @list Video Management
     */
    virtual int setCameraAdaptiveMinimumFrameRate(int framerate) = 0;
};

}  // namespace bytertc