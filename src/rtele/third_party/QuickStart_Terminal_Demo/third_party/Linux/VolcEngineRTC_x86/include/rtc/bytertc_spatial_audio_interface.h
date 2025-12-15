/*
 *  Copyright (c) 2022 The VolcEngineRTC project authors. All Rights Reserved.
 *  @company ByteDance.Inc
 *  @brief spatial audio interface
 */

#pragma once

#include "bytertc_audio_defines.h"

namespace bytertc {
/**
 * @locale zh
 * @type keytype
 * @brief 用户在空间音频坐标系里的位置信息。
 */
/**
 * @locale en
 * @type keytype
 * @brief Information on the user's position in the rectangular coordinate system for the spatial audio.
 */
struct PositionInfo {
    /**
     * @locale zh
     * @brief 用户在空间音频坐标系里的位置，需自行建立空间直角坐标系。参看 Position{@link #Position}
     */
    /**
     * @locale en
     * @brief 3D coordinate values of the user's position in the rectangular coordinate system for the spatial audio. You need to build your own rectangular coordinate system. Refer to Position{@link #Position} for details.
     */
    Position position;
    /**
     * @locale zh
     * @brief 用户在空间音频坐标系里的三维朝向信息。三个向量需要两两垂直。参看 HumanOrientation{@link #HumanOrientation}。
     */
    /**
     * @locale en
     * @brief Information on the three-dimensional orientation of the user in the rectangular coordinate system for the spatial audio. Any two of the 3D coordinate vectors of the user's position need to be perpendicular to each other. Refer to HumanOrientation{@link #HumanOrientation} for details.
     */
    HumanOrientation orientation;
};
/**
 * @locale zh
 * @hidden(Linux)
 * @type api
 * @brief 空间音频接口实例
 * @list 
 */
/**
 * @locale en
 * @hidden(Linux)
 * @type api
 * @brief Spatial audio instance
 * @list 
 */
class ISpatialAudio {
public:
    /**
     * @locale zh
     * @type api
     * @brief 开启/关闭空间音频功能。
     * @param enable 是否开启空间音频功能： <br>
     *        - true：开启
     *        - false：关闭（默认）
     * @note 该方法仅开启空间音频功能，你须调用 updatePosition{@link #IRangeAudio#updatePosition} 设置自身位置坐标后方可收听空间音频效果。空间音频相关 API 和调用时序详见[空间音频](https://www.volcengine.com/docs/6348/93903)。
     * @list 高级功能
     */
    /**
     * @locale en
     * @type api
     * @brief Enable/disable spatial audio function.
     * @param enable Whether to enable spatial audio function: <br>
     *        - true：Enable
     *        - false：Disable(Default setting)
     * @note You need to call updatePosition{@link #IRangeAudio#updatePosition} as well to really enjoy the spatial audio effect.
     * @list Advanced Features
     */
    virtual void enableSpatialAudio(bool enable) = 0;
    /**
     * @locale zh
     * @type api
     * @brief 关闭本地用户朝向对本地用户发声效果的影响。 <br>
     *        调用此接口后，房间内的其他用户收听本地发声时，声源都在收听者正面。
     * @note
     *        - 调用本接口关闭朝向功能后，在当前的空间音频实例的生命周期内无法再次开启。
     *        - 调用此接口不影响本地用户收听朝向的音频效果。要改变本地用户收听朝向，参看 updateSelfPosition{@link #ISpatialAudio#updateSelfPosition} 和 updateRemotePosition{@link #ISpatialAudio#updateRemotePosition} 。
     * @list 高级功能
     */
    /**
     * @locale en
     * @type api
     * @brief Turn off the effect of the orientation of the local user as the sound source. <br>
     *        After the effect is off, all the other users in the room listen to the local user as if the local user is in right front of each of them.
     * @note
     *        - After the orientation effect as the sound source is disabled, you cannot enable it during the lifetime of the `SpatialAudio` instance.
     *        - Calling this API does not affect the orientation effect of the local user as a listener. See updateSelfPosition{@link #ISpatialAudio#updateSelfPosition} and updateRemotePosition{@link #ISpatialAudio#updateRemotePosition} 
     */
    virtual void disableRemoteOrientation() = 0;
    /**
     * @locale zh
     * @hidden(Linux)
     * @valid since 3.52
     * @type api
     * @brief 设置本地用户在自建空间直角坐标系中的收听坐标和收听朝向，以实现本地用户预期的空间音频收听效果。
     * @param position_info 空间音频位置信息。参看 PositionInfo{@link #PositionInfo}。
     * @return
     *        - 0：成功。
     *        - <0：失败。
     *        - -2: 失败，原因是校验本地用户的三维朝向信息时，三个向量没有两两垂直。
     * @note
     *        - 该方法需在进房后调用。
     *        - 调用该接口更新坐标前，你需调用 enableSpatialAudio{@link #ISpatialAudio#enableSpatialAudio} 开启空间音频功能。空间音频相关 API 和调用时序详见[空间音频](https://www.volcengine.com/docs/6348/93903)。
     *        - 调用此接口在本地进行的设定对其他用户的空间音频收听效果不会产生任何影响。
     * @list 高级功能
     */
    /**
     * @locale en
     * @hidden(Linux)
     * @valid since 3.52
     * @type api
     * @brief Sets the coordinate and orientation of the local user as a listener in the rectangular coordinate system the local user built to achieve expected spatial audio effects.
     * @param position_info Information on the local user's position. Refer to  PositionInfo{@link #PositionInfo} for details.
     * @return
     *        - 0: Success.
     *        - <0: Failure.
     *        - -2: Failure. The reason is that any two of the 3D coordinate vectors of your position are not perpendicular to each other.
     * @note
     *        - You need to call this API after joining the room.
     *        - Before calling this API, you should call enableSpatialAudio{@link #ISpatialAudio#enableSpatialAudio} first to enable the spatial audio function.
     *        - The settings made locally will not influence other users' spatial audio experience.
     * @list Advanced Features
     */
    virtual int updateSelfPosition(const PositionInfo& position_info) = 0;
    /**
     * @locale zh
     * @hidden(Linux)
     * @valid since 3.52
     * @type api
     * @brief 设置房间内某一远端用户在本地用户自建的空间音频坐标系中的发声位置和发声朝向，以实现本地用户预期的空间音频收听效果。
     * @param uid 用户 ID
     * @param position_info 远端用户的空间音频位置信息。参看 PositionInfo{@link #PositionInfo}。
     * @return
     *        - 0：成功。
     *        - <0：失败。
     *        - -2: 失败，原因是校验远端用户的三维朝向信息时，三个向量没有两两垂直。
     * @note
     *        - 该方法需在创建房间后调用。
     *        - 调用此接口在本地进行的设定对其他用户的空间音频收听效果不会产生任何影响。
     * @list 高级功能
     */
    /**
     * @locale en
     * @hidden(Linux)
     * @valid since 3.52
     * @type api
     * @brief Sets the coordinate and orientation of the remote user as a speaker in the rectangular coordinate system of the local user. In this case, the local user hears from the remote user with the expected spatial audio effects.
     * @param uid User ID
     * @param position_info Information on the remote user's position. Refer to PositionInfo{@link #PositionInfo} for details.
     * @return
     *        - 0: Success.
     *        - <0: Failure.
     *        - -2: Failure. The reason is that any two of the 3D coordinate vectors of the position of the remote user are not perpendicular to each other.
     * @note
     *        - You must call this API after creating the room.
     *        - The settings made locally will not influence other users' spatial audio experience.
     * @list Advanced Features
     */
    virtual int updateRemotePosition(const char* uid, const PositionInfo& position_info) = 0;
    /**
     * @locale zh
     * @hidden(Linux)
     * @valid since 3.52
     * @type api
     * @brief 移除调用 updateRemotePosition{@link #ISpatialAudio#updateRemotePosition} 为某一远端用户设置的空间音频效果。
     * @param uid 远端用户 ID。
     * @return
     *        - 0：成功。
     *        - <0：失败。
     * @list 高级功能
     */
    /**
     * @locale en
     * @hidden(Linux)
     * @valid since 3.52
     * @type api
     * @brief Disables all spatial audio effects set by calling updateRemotePosition{@link #ISpatialAudio#updateRemotePosition} for a certain remote user.
     * @param uid User ID of the remote user.
     * @return
     *        - 0: Success.
     *        - <0: Failure.
     * @list Advanced Features
     */
    virtual int removeRemotePosition(const char* uid) = 0;
    /**
     * @locale zh
     * @hidden(Linux)
     * @valid since 3.52
     * @type api
     * @brief 移除调用 updateRemotePosition{@link #ISpatialAudio#updateRemotePosition} 为所有远端用户设置的空间音频效果。
     * @return
     *        - 0：成功。
     *        - <0：失败。
     * @list 高级功能
     */
    /**
     * @locale en
     * @hidden(Linux)
     * @valid since 3.52
     * @type api
     * @brief Disables all spatial audio effects set by calling updateRemotePosition{@link #ISpatialAudio#updateRemotePosition} for all remote users.
     * @return
     *        - 0: Success.
     *        - <0: Failure.
     * @list Advanced Features
     */
    virtual int removeAllRemotePosition() = 0;

    /**
     * @locale zh
     * @hidden constructor/destructor
     * @brief 析构函数
     */
    /**
     * @locale en
     * @hidden constructor/destructor
     * @brief  Destructor
     */
    virtual ~ISpatialAudio() = default;
};

}  // namespace bytertc