/*
 *  Copyright (c) 2022 The VolcEngineRTC project authors. All Rights Reserved.
 *  @company ByteDance.Inc
 *  @brief panoramic video interface
 */

#pragma once

namespace bytertc {
/**
 * @locale zh
 * @hidden for internal use only on Windows and Android
 * @type keytype
 * @brief 四元数表示的头部位姿信息<br>
 * q = xi + yj + zk + w，i^2 = j^2 = k^2 = −1
 */
/**
 * @locale en
 * @hidden for internal use only on Windows and Android
 * @type keytype
 * @brief Head pose represented in quaternion.<br>
 * q = xi + yj + zk + w，i^2 = j^2 = k^2 = −1
 */
typedef struct Quaternionf {    
    /**
     * @locale zh
     * @brief x 坐标
     */
    /**
     * @locale en
     * @brief x-coordinate
     */
    float    x;
    /**
     * @locale zh
     * @brief y 坐标
     */
    /**
     * @locale en
     * @brief y-coordinate
     */
    float    y;
    /**
     * @locale zh
     * @brief z 坐标
     */
    /**
     * @locale en
     * @brief z-coordinate
     */
    float    z;
    /**
     * @locale zh
     * @brief 旋转角
     */
    /**
     * @locale en
     * @brief Angle of rotation
     */
    float    w;
} Quaternionf;

/**
 * @locale zh
 * @hidden (Windows, Android) internal use only
 * @type api
 * @brief 全景视频接口实例
 */
/**
 * @locale en
 * @hidden for internal use only on Windows and Android
 * @type api
 * @brief Panoramic video instance
 */
class IPanoramicVideo {
public:
    /** 
     * @locale zh
     * @type api
     * @brief 更新接收者的头位姿四元数。RTC 将根据头位姿下发相应的 Tile 视频。
     * @param info 参考 Quaternionf{@link #Quaternionf}。
     */
    /** 
     * @locale en
     * @type api
     * @brief Update the position of the head for the receiver. The receiver will receive the video tile according to the head position.
     * @param info Refer to Quaternionf{@link #Quaternionf} for details.
     */
    virtual int updateQuaternionf(const Quaternionf& info) = 0;

    virtual ~IPanoramicVideo() = default;
};
}
