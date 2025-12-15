/*
 * Copyright (c) 2020 The VolcEngineRTC project authors. All Rights Reserved.
 * @brief VolcEngineRTC Advance API
*/

#pragma once

#include "../bytertc_engine.h"

#include <stdint.h>
#ifdef __ANDROID__
#include "jni.h"
#endif


#ifdef __GNUC__
#    define attribute_deprecated __attribute__((deprecated))
#elif defined(_MSC_VER)
#    define attribute_deprecated __declspec(deprecated)
#else
#    define attribute_deprecated
#endif

namespace bytertc {
/**
 * @locale zh
 * @hidden for internal use only
 * @brief 设置应用的状态
 * @param engine <br>
 *       要通知的引擎
 * @param app_state 应用状态字符串
 */
/**
 * @locale en
 * @hidden for internal use only
 * @brief  Set the state of the application
 * @param engine <br>
 *        Engine to notify
 * @param app_state Application state string
 */
BYTERTC_API void setAppState(void* engine, const char* app_state);
/**
 * @locale zh
 * @hidden(iOS,macOS,Windows,Linux) for internal use only
 * @type api
 * @brief 用于设置 Android 的 ApplicationContext 给 Engine 使用， <br>
 * 如果 Android 使用纯 C++接口，则该方法必须在引擎创建前设置
 * @param j_egl_context <br>
 *      传入 Java 的 android.content.Context 类型的 EGLContext
 * @list 引擎管理
 */
/**
 * @locale en
 * @hidden(iOS,macOS,Windows,Linux) for internal use only
 * @type api
 * @brief Used to set the ApplicationContext for Android to use by Engine, <br>
 * If Android uses a pure C++ interface, this method must be set before the engine is created
 * @param j_egl_context <br>
 *      Passing in Java android.content. Context type EGLContext
 * @list Engine Management
 */
BYTERTC_API void setApplicationContext(void* j_application_context);
BYTERTC_API int setLogConfig(const LogConfig& log_config);

/**
 * @locale zh
 * @region 视频管理
 * @type keytype
 * @brief 硬件加速设备类型
 */
/**
 * @locale en
 * @region video management
 * @type keytype
 * @brief  Hardware acceleration device type
 */
enum HWDeviceType {
    /**
     * @locale zh
     * @brief 未知的设备类型
     */
    /**
     * @locale en
     * @brief Unknown device type
     */
    kHWDeviceTypeNone,

    /**
     * @locale zh
     * @brief direct3d 11 视频加速设备
     */
    /**
     * @locale en
     * @brief Direct3d 11 Video acceleration device
     */
    kHWDeviceTypeD3D11,

    /**
     * @locale zh
     * @brief cuda 硬件加速设备
     * @note cuda 是 nvidia 平台下硬件加速设备
     */
    /**
     * @locale en
     * @brief Cuda  hardware acceleration device
     * @note Cuda  is a hardware acceleration device under the nvidia platform
     */
    kHWDeviceTypeCuda,

    /**
     * @locale zh
     * @brief intel 平台下 qsv 加速设备
     */
    /**
     * @locale en
     * @brief Qsv acceleration device under intel  platform
     */
    kHWDeviceTypeQsv,

    /**
     * @locale zh
     * @brief windows 平台 dx 硬件加速设备
     */
    /**
     * @locale en
     * @brief Windows  platform dx hardware acceleration device
     */
    kHWDeviceTypeDxva2,

    /**
     * @locale zh
     * @brief Android 平台下硬件加速设备 mediacodec
     */
    /**
     * @locale en
     * @brief Hardware acceleration device mediacodec under Android  platform
     */
    kHWDeviceTypeMediaCodec,

    /**
     * @locale zh
     * @brief IOS、MACOS 平台下硬件加速设备 VideoToolbox
     */
    /**
     * @locale en
     * @brief IOS, MACOS  platform hardware acceleration device VideoToolbox
     */
    kHWDeviceTypeVideoToolbox,
     /**
     * @locale zh
     * @brief android、linux 平台下硬件加速设备 egl context
     */
    /**
     * @locale en
     * @brief Android, linux  platform hardware acceleration device egl context
     */
    kHWDeviceTypeEglContext,

    /**
     * @locale zh
     * @brief linux、windows 平台下硬件加速设备 va context
     */
    /**
     * @locale en
     * @brief Linux, windows  platform hardware acceleration device va context
     */
    kHWDeviceTypeVaapi,
};

/**
 * @locale zh
 * @hidden(macOS, Linux, iOS, Android)
 * @type keytype
 * @brief 硬编码设备 context
 */
/**
 * @locale en
 * @hidden(macOS, Linux, iOS, Android)
 * @type keytype
 * @brief  Hardcoding device context
 */
enum HWDeviceContextKey {
    /**
     * @locale zh
     * @brief cuda context
     */
    /**
     * @locale en
     * @brief cuda context
     */     
    kHWDeviceContextKeyCudaContext,
    /**
     * @locale zh
     * @brief derict3D11 设备
     */
    /**
     * @locale en
     * @brief Derict3D11  device
     */
    kHWDeviceContextKeyD3D11Device,
    /**
     * @locale zh
     * @brief derict3D11 设备 context
     */
    /**
     * @locale en
     * @brief Derict3D11  device context
     */
    kHWDeviceContextKeyD3D11DeviceContext,
    /**
     * @locale zh
     * @brief egl 设备 context
     */
    /**
     * @locale en
     * @brief Egl  device context
     */
    kHWDeviceContextKeyEglContext,
    /**
     * @locale zh
     * @brief vadisplay 设备 context
     */
    /**
     * @locale en
     * @brief Vadisplay  device context
     */
    kHWDeviceContextKeyVADisplay,
};

/**
 * @locale zh
 * @hidden(macOS, Linux, iOS, Android)
 * @type keytype
 * @brief 硬件加速设备 context
 */
/**
 * @locale en
 * @hidden(macOS, Linux, iOS, Android)
 * @type keytype
 * @brief Hardware acceleration device context
 */
class IHWDeviceContext {
public:
    virtual ~IHWDeviceContext() = default;
    /**
     * @locale zh
     * @brief 获取硬件加速设备 context 类型
     * @return 硬件加速设备类型，详见 HWDeviceType{@link #HWDeviceType}
     */
    /**
     * @locale en
     * @brief Get the hardware acceleration device context type
     * @return  Hardware acceleration device type. See HWDeviceType{@link #HWDeviceType}
     */
    virtual HWDeviceType deviceType() const = 0;
    /**
     * @locale zh
     * @brief 设置属性
     * @param key 硬件加速设备 context 类型，详见 HWDeviceContextKey{@link #HWDeviceContextKey}
     * @param value 指向硬件加速设备 context 地址的指针
     */
    /**
     * @locale en
     * @brief Set property
     * @param key Hardware acceleration device context type. See HWDeviceContextKey{@link #HWDeviceContextKey}
     * @param value Pointer to the hardware acceleration device context address
     */
    virtual void setProperty(HWDeviceContextKey key, void* value) = 0;
    /**
     * @locale zh
     * @brief 获取属性
     * @param key 硬件加速设备 context 类型，详见 HWDeviceContextKey{@link #HWDeviceContextKey}
     * @return 返回硬件加速设备 context 地址的指针
     */
    /**
     * @locale en
     * @brief Get the property
     * @param key Hardware acceleration device context type. See HWDeviceContextKey{@link #HWDeviceContextKey}
     * @return  Return a pointer to the hardware acceleration device context address
     */
    virtual void* getProperty(HWDeviceContextKey key) = 0;
    /**
     * @locale zh
     * @brief 内部 context 是否由该实例拥有
     * @return
     *        - true: 硬件设备 context 被实例对象拥有
     *        - false: 硬件设备上 context 文不被实例对象拥有
     */
    /**
     * @locale en
     * @brief Internal context is owned by the instance
     * @return
     *         - True: hardware device context is owned by the instance object
     *         - False: hardware device context is not owned by the instance object
     */
    virtual bool ownContext() const = 0;
    /**
     * @locale zh
     * @brief 设置 context 是否被实例对象拥有
     * @param own_context <br>
     *       - true: context 被实例对象拥有
     *       - false: context 不被实例对象拥有
     */
    /**
     * @locale en
     * @brief Sets whether context is owned by the instance object
     * @param own_context <br>
     *        - True: context is owned by the instance object
     *        - False: context is not owned by the instance object
     */
    virtual void setOwnContext(bool own_context) = 0;
    /**
     * @locale zh
     * @brief 转移硬件设备 context，
     * @return 指向硬件设备 context 的指针
     */
    /**
     * @locale en
     * @brief Transfer hardware device context,
     * @return  Pointer to hardware device context
     */
    virtual IHWDeviceContext* moveContext() = 0;
    /**
     * @locale zh
     * @brief 释放实例对象
     */
    /**
     * @locale en
     * @brief Release instance object
     */
    virtual void release() = 0;
};

/**
 * @locale zh
 * @hidden for internal use only
 * @brief 创建一个具有指定设备类型的硬件设备上下文
 */
 /**
 * @locale en
 * @hidden for internal use only
 * @brief create a hardware device context with specified deviceType
 */

BYTERTC_API IHWDeviceContext* createHwDeviceContext(HWDeviceType device_type);

/**
 * @locale zh
 * @hidden for internal use only
 * @brief 设置视频源的设备上下文信息
 * @param engine <br>
 *       要设置的引擎，详见 IRTCEngine{@link #IRTCEngine}
 * @param hw_context <br>
 *        设置视频帧，详见：IHWDeviceContext{@link #IHWDeviceContext}
 * @note 用于硬件编码传入 GPU 内存时使用
 */
/**
 * @locale en
 * @hidden for internal use only
 * @brief  Set the device context information of the video source
 * @param engine <br>
 *        The engine to be set, see: IRTCEngine{@link #IRTCEngine}
 * @param hw_context <br>
 *        Set the video frame, see: IHWDeviceContext{@link #IHWDeviceContext}
 * @note Used when hardware encoding is passed in GPU memory
 */
BYTERTC_API void setVideoSourceHWDeviceContext(void* engine, IHWDeviceContext* hw_context);

/**
 * @locale zh
 * @hidden for internal use only
 * @type callback
 * @brief 本地音频帧监测器
 * @list 自定义流处理
 */
/**
 * @locale en
 * @hidden for internal use only
 * @type callback
 * @brief Local audio frame monitor
 * @list Custom Stream Processing
 */
class ILocalEncodedAudioFrameObserver  {
public:
    virtual ~ILocalEncodedAudioFrameObserver() {
    }
    /**
     * @locale zh
     * @type callback
     * @brief 调用 RegisterLocalEncodedAudioFrameObserver 后，SDK 收到本地音频帧信息时，回调该事件
     * @param audio_source 本地音频源类型，详见 IAudioSource{@link #IAudioSource}
     * @param audio_stream 本地音频帧信息，参看 IEncodedAudioFrame
     * @list 自定义流处理
     */
    /**
     * @locale en
     * @type callback
     * @brief Call RegisterLocalEncodedAudioFrameObserver, when the SDK receives local audio frame information, callback the event
     * @param audio_source Local audio source type. See IAudioSource{@link #IAudioSource}
     * @param audio_stream Local audio frame information. See IEncodedAudioFrame
     * @list Custom Stream Processing
     */
    virtual void onLocalEncodedAudioFrame(IAudioSource* audio_source, const EncodedAudioFrameData& audio_stream) = 0;
};
}  // namespace bytertc

