/*
 * Copyright (c) 2024 The VolcEngineRTC project authors. All Rights Reserved.
 * @brief VolcEngineRTC Room Interface
 */

#pragma once

#include "rtc/bytertc_video_effect_interface.h"  // NOLINT
#include "rtc/bytertc_defines.h"
#include "bytertc_room_event_handler.h"
#include "rtc/bytertc_range_audio_interface.h"
#include "rtc/bytertc_spatial_audio_interface.h"
#include "rtc/bytertc_panoramic_video_interface.h"
namespace bytertc {
/**
 * @locale zh
 * @type api
 * @brief 房间接口
 * @list 
 */
/**
 * @locale en
 * @type api
 * @brief  Room interface
 * @list 
 */
class IGameRoom {
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
    virtual ~IGameRoom() {
    }
    /**
     * @locale zh
     * @type api
     * @brief 退出并销毁调用 createGameRoom{@link #IRTCEngine#createGameRoom} 所创建的游戏房间实例。
     * @list 房间管理
     */
    /**
     * @locale en
     * @type api
     * @brief Leave and destroy the game room instance created by calling createGameRoom{@link #IRTCEngine#createGameRoom}.
     * @list Room Management
     */
    virtual void destroy() = 0;
    /**
     * @locale zh
     * @type api
     * @brief 加入游戏房间。 <br>
     *        调用 createGameRoom{@link #IRTCEngine#createGameRoom} 创建房间实例后，调用此方法加入房间，同房间内其他用户进行音频通话。
     * @param token 动态密钥，用于对登录用户进行鉴权验证。 <br>
     *        进入房间需要携带 Token。测试时可使用控制台生成临时 Token，正式上线需要使用密钥 SDK 在您的服务端生成并下发 Token。Token 有效期及生成方式参看[使用 Token 完成鉴权](70121)。 <br>
     *       - 使用不同 App ID 的 App 是不能互通的。
     *       - 请务必保证生成 Token 使用的 App ID 和创建引擎时使用的 App ID 相同，否则会导致加入房间失败。
     * @param user_info 用户信息，参看 UserInfo{@link #UserInfo}。
     * @return
     *        -  0: 成功。触发以下回调：
     *          - 本端收到房间状态通知 onRoomStateChanged{@link #IRTSRoomEventHandler#onRoomStateChanged} 回调。
     *          - 如果本端用户为可见用户，房间内其他用户收到 onUserJoined{@link #IRTCRoomEventHandler#onUserJoined} 回调通知。
     *        - -1：room_id/user_info.uid 包含了无效的参数。
     *        - -2：已经在房间内。接口调用成功后，只要收到返回值为 0 ，且未调用 leaveRoom:{@link #IGameRoom#leaveRoom} 成功，则再次调用进房接口时，无论填写的房间 ID 和用户 ID 是否重复，均触发此返回值。
     *        调用失败时，具体失败原因会通过 onRoomStateChanged{@link #IRTSRoomEventHandler#onRoomStateChanged} 回调告知。
     * @note
     *       - 同一个 App ID 的同一个房间内，每个用户的用户 ID 必须是唯一的。如果两个用户的用户 ID 相同，则后进房的用户会将先进房的用户踢出房间，并且先进房的用户会收到 onRoomStateChanged{@link #IRTSRoomEventHandler#onRoomStateChanged} 回调通知，错误类型详见 ErrorCode{@link #ErrorCode} 中的 kErrorCodeDuplicateLogin。
     *       - 用户加入房间成功后，在本地网络状况不佳的情况下，SDK 可能会与服务器失去连接，并触发 onConnectionStateChanged{@link #IRTCEngineEventHandler#onConnectionStateChanged} 回调。此时 SDK 会自动重试，直到成功重连。重连成功后，本地会收到 onRoomStateChanged{@link #IRTSRoomEventHandler#onRoomStateChanged} 回调通知。
     * @list 房间管理
     */
    /**
     * @locale en
     * @type api
     * @brief Join the game room. <br>
     *        After creating a room by calling createGameRoom{@link #IRTCEngine#createGameRoom}, call this API to join the room and make audio calls with other users in the room.
     * @param token Dynamic key for authenticating the logged-in user. <br>
     *         You need to bring Token to enter the room. When testing, you can use the console to generate temporary tokens. The official launch requires the use of the key SDK to generate and issue tokens at your server level. See [Authentication with Token](70121) for token validity and generation method. <br>
     *        - Apps with different App IDs cannot communicate with each other.
     *        - Make sure that the App ID used to generate the Token is the same as the App ID used to create the engine, otherwise it will cause the join room to fail.
     * @param user_info User information. See UserInfo{@link #UserInfo}.
     * @return
     *         - 0: Success. 
     *            - Local users receive notifications of the room state via onRoomStateChanged{@link #IRTSRoomEventHandler#onRoomStateChanged}. 
     *            - If the local user is also a visible user, the other participants receive onUserJoined{@link #IRTCRoomEventHandler#onUserJoined} callback.
     *         - -1: Room_id/user_info.uid contains invalid parameters.
     *         - -2: Already in the room. After the interface call is successful, as long as the return value of 0 is received and leaveRoom:{@link #IGameRoom#leaveRoom} is not called successfully, this return value is triggered when the room entry interface is called again, regardless of whether the filled room ID and user ID are duplicated.
     *         The reason for the failure will be communicated via the onRoomStateChanged{@link #IRTSRoomEventHandler#onRoomStateChanged} callback.
     * @note
     *        - In the same room with the same App ID, the user ID of each user must be unique. If two users have the same user ID, the user who entered the room later will kick the user who entered the room out of the room, and the user who entered the room will receive the onRoomStateChanged{@link #IRTSRoomEventHandler#onRoomStateChanged} callback notification. For the error type. See kErrorCodeDuplicateLogin in ErrorCode{@link #ErrorCode}.
     *        - After the user successfully joins the room, the SDK may lose connection to the server in case of poor local network conditions. At this point, onConnectionStateChanged{@link #IRTCEngineEventHandler#onConnectionStateChanged} callback will be triggered and the SDK will automatically retry until it successfully reconnects to the server. After a successful reconnection, you will receive a local onRoomStateChanged{@link #IRTSRoomEventHandler#onRoomStateChanged} callback notification.
     * @list Room Management
     */
    virtual int joinRoom(const char* token, const UserInfo& user_info) = 0;
    /**
     * @locale zh
     * @type api
     * @brief 离开游戏房间。 <br>
     *        调用此方法结束通话过程，并释放所有通话相关的资源。
     * @return
     *        - 0: 调用成功。如果用户是房间内可见用户，远端用户收到 onUserLeave{@link #IRTSRoomEventHandler#onUserLeave} 回调通知。
     *        - < 0: 调用失败，参看 ReturnStatus{@link #ReturnStatus} 获得更多错误说明。
     * @note
     *       - 加入房间后，必须调用此方法结束通话，否则无法开始下一次通话。
     *       - 此方法是异步操作，调用返回时并没有真正退出房间。真正退出房间后，本地会收到 onRoomStateChanged{@link #IRTSRoomEventHandler#onRoomStateChanged} 回调通知。你必须在收到 onRoomStateChanged{@link #IRTSRoomEventHandler#onRoomStateChanged} 回调后，再销毁房间或引擎，或调用 joinRoom{@link #IGameRoom#joinRoom} 再次加入房间。
     * @list 房间管理
     */
    /**
     * @locale en
     * @type api
     * @brief Leave the game room. <br>
     *        The user calls this method to leave the game room, end the call process, and release all call-related resources. <br>
     * @return
     *        - 0: Success. If the user is a host:
     *            - The other participants receive onUserLeave{@link #IRTSRoomEventHandler#onUserLeave} callback.
     *        - < 0 : Fail. See ReturnStatus{@link #ReturnStatus} for more details
     * @note
     *        - When visible users leave the room, others in the room will receive onUserLeave{@link #IRTSRoomEventHandler#onUserLeave}.
     *        It is an asynchronous operation, and the call returns without actually exiting the room. When actually exiting the room, you will receive onRoomStateChanged{@link #IRTSRoomEventHandler#onRoomStateChanged}. If the engine is destroyed immediately after this method is called, you will not receive onRoomStateChanged{@link #IRTSRoomEventHandler#onRoomStateChanged}.
     */
    virtual int leaveRoom() = 0;
    /**
     * @locale zh
     * @type api
     * @brief 更新游戏房间 Token。 <br>
     *        收到 onTokenWillExpire{@link #IRTCRoomEventHandler#onTokenWillExpire}，onPublishPrivilegeTokenWillExpire{@link #IRTCRoomEventHandler#onPublishPrivilegeTokenWillExpire}，或 onSubscribePrivilegeTokenWillExpire{@link #IRTCRoomEventHandler#onSubscribePrivilegeTokenWillExpire} 时，你必须重新获取 Token，并调用此方法更新 Token，以保证通话的正常进行。
     * @param token 重新获取的有效 Token。 <br>
     *        如果 Token 无效，你会收到 onRoomStateChanged{@link #IRTSRoomEventHandler#onRoomStateChanged}，错误码是 `-1010`。
     * @return 方法调用结果： <br>
     *        - 0：成功；
     *        - <0：失败。具体失败原因参看 ReturnStatus{@link #ReturnStatus}。
     * @note 请勿同时调用 updateToken{@link #IGameRoom#updateToken} 和 joinRoom{@link #IGameRoom#joinRoom} 方法更新 Token。若因 Token 过期或无效导致加入房间失败或已被移出房间，你应该在获取新的有效 Token 后调用 joinRoom{@link #IGameRoom#joinRoom} 重新加入房间。
     * @list 房间管理
     */
    /**
     * @locale en
     * @type api
     * @brief Update the game room Token. <br>
     *        You must call this API to update token to ensure the RTC call to continue when you receive onTokenWillExpire{@link #IRTCRoomEventHandler#onTokenWillExpire}, onPublishPrivilegeTokenWillExpire{@link #IRTCRoomEventHandler#onPublishPrivilegeTokenWillExpire}, or onSubscribePrivilegeTokenWillExpire{@link #IRTCRoomEventHandler#onSubscribePrivilegeTokenWillExpire}.
     * @param token  Valid token. <br>
     *        If the Token is invalid, you will receive onRoomStateChanged{@link #IRTSRoomEventHandler#onRoomStateChanged} with the error code of `-1010`.
     * @return API call result: <br>
     *        - 0: Success.
     *        - <0: Failure. See ReturnStatus{@link #ReturnStatus} for specific reasons.
     * @note Do not call both updateToken{@link #IGameRoom#updateToken} and joinRoom{@link #IGameRoom#joinRoom} to update the Token. If you fail to join the room or have been removed from the room due to an expired or invalid Token, call joinRoom{@link #IGameRoom#joinRoom} with a valid token to rejoin the room.
     * @list Room Management
     */
    virtual int updateToken(const char* token) = 0;
    /**
     * @locale zh
     * @type api
     * @brief 通过设置 IGameRoom{@link #IGameRoom} 对象的事件句柄，监听此对象对应的回调事件。
     * @param room_event_handler 参见 IRTCRoomEventHandler{@link #IRTCRoomEventHandler}
     * @return
     *        - 0: 调用成功。
     *        - < 0 : 调用失败。查看 ReturnStatus{@link #ReturnStatus} 获得更多错误说明
     * @list 房间管理
     */
    /**
     * @locale en
     * @type api
     * @brief Listens for event callbacks related to the IGameRoom{@link #IGameRoom} instance by setting the event handler of this instance.
     * @param room_event_handler Refer to IRTCRoomEventHandler{@link #IRTCRoomEventHandler}.
     * @return
     *        - 0: Success.
     *        - < 0 : Fail. See ReturnStatus{@link #ReturnStatus} for more details
     * @list Room Management
     */
    virtual int setRTCRoomEventHandler(IRTCRoomEventHandler* room_event_handler) = 0;
    /**
     * @locale zh
     * @type api
     * @brief 获取游戏房间范围语音接口实例。
     * @return 方法调用结果： <br>
     *        - IRangeAudio：成功，返回一个 IRangeAudio{@link #IRangeAudio} 实例。
     *        - nullptr：失败，当前 SDK 不支持范围语音功能。
     * @note 首次调用该方法须在创建房间后、加入房间前。范围语音相关 API 和调用时序详见[范围语音](https://www.volcengine.com/docs/6348/114727)。
     * @list 高级功能
     */
    /**
     * @locale en
     * @type api
     * @brief Gets the game room range audio instance.
     * @return API call result: <br>
     *        - IRangeAudio: Success. You will get an IRangeAudio{@link #IRangeAudio} returned from the SDK.
     *        - nullptr: Failure. The current SDK does not offer range audio function.
     * @note The first time this API is called must be between you create a room and you actually enter the room. Refer to [Range Voice](https://docs.byteplus.com/byteplus-rtc/docs/114727) for more information.
     * @list Advanced Features
     */
    virtual IRangeAudio* getRangeAudio() = 0;
    /**
     * @locale zh
     * @type api
     * @brief 开始讲话或者停止讲话时调用，调用该接口启动或停止音频采集。同房间其他用户会收到
     * 回调 OnAudioDeviceStateChanged{@link #IRTCEngineEventHandler#OnAudioDeviceStateChanged}
     * @param [in] enable
     *             true 表示开启采集
     *             false 表示关闭采集，默认设置
     * @return
     *        0:  表示参数检查通过，不代表打开麦克风会成功，比如房间不存在
     *        -2: 传入的room_id为空导致失败
     * @note 不可与 EnableAudioSend{@link #IGameRoom#EnableAudioSend} 同时调用。
     */
    /**
     * @locale en
     * @type api
     * @brief Called when starting to speak or stopping speaking. Calling this interface will start or stop audio collection. Other users in the same room will receive
     * callback OnAudioDeviceStateChanged{@link #IRTCEngineEventHandler#OnAudioDeviceStateChanged}
     * @param [in] enable
     *             true Indicates starting record
     *             false Indicates stopping record
     * @return
     *        0:  It means that the parameter check has passed, but it does not mean that opening the microphone will succeed. For example, the room does not exist.
     *        -2: The room_id passed in is empty, causing failure.
     * @note Cannot be called simultaneously with EnableAudioSend{@link #IGameRoom#EnableAudioSend}.
     */
    virtual int enableMicrophone(bool enable) = 0;
    /**
     * @locale zh
     * @type api
     * @brief 开启或关闭扬声器。
     * @param enable 是否开启声器：<br>
     *               - true：开启扬声器，订阅所有远端用户的音频流。
     *               - false：默认设置。关闭扬声器，取消订阅所有远端用户的音频流。
     * @return
     *        - 0：接口调用成功。
     *        - -3：接口调用失败。没有加入房间。
     */
    /**
     * @locale en
     * @type api
     * @brief Turns on/off the speaker.
     * @param enable Whether to turn on the speaker: <br>
     *              - true: Turns on the speaker to subscribe to audio streams of all remote users.
     *              - false: Default setting. Turns off the speaker to unsubscribes audio streams from all remote users.
     * @return
     *        - 0: Success.
     *        - -3: Failure. Not joined a room.
     */
    virtual int enableSpeakerphone(bool enable) = 0;
    /**
     * @locale zh
     * @type api
     * @brief 开始或停止发送音频流。调用此接口不影响音频采集。同房间其他用户会收到相应的回调。
     * @param enable 是否发送音频流：<br>
     *               - true：发送音频流。
     *               - false：默认设置。停止发送音频流（不会关闭麦克风）。
     * @return
     *        0:  表示参数检查通过，不代表打开麦克风会成功，比如房间不存在
     *        -2: 传入的room_id为空导致失败
     * @note 不可与 EnableMicrophone{@link #IGameRoom#EnableMicrophone} 同时调用。
     */
    /**
     * @locale en
     * @type api
     * @brief Start or stop sending audio streams. Calling this interface does not affect audio collection. Other users in the same room will receive the corresponding callback.
     * @param enable Whether to send the audio data: <br>
     *               - true: Starts sending audio data.
     *               - false: Default setting. Stops publishing audio data.
     * @return
     *        0:  It means that the parameter check has passed, but it does not mean that opening the microphone will succeed. For example, the room does not exist.
     *        -2: The room_id passed in is empty, causing failure.
     * @note Cannot be called at the same time as EnableMicrophone{@link #IGameRoom#EnableMicrophone}.
     */
    virtual int enableAudioSend(bool enable) = 0;
    /**
     * @locale zh
     * @type api
     * @brief 是否接收某个特定用户的音频流。关闭声音接收不会影响扬声器或其他音频输出设备的状态。
     * @param user_id 用户 ID，最大长度为128字节的非空字符串。支持的字符集范围为: <br>
     *            1. 26个大写字母 A ~ Z<br>
     *            2. 26个小写字母 a ~ z<br>
     *            3. 10个数字 0 ~ 9<br>
     *            4. 下划线"_", at符"@", 减号"-"
     * @param enable 是否接收指定用户的音频流：<br>
     *               - true：接收该用户的音频流。即允许该用户的音频数据被传递到本地设备并播放。
     *               - false：默认设置，不接收该用户的音频流，即不播放该用户的声音。但不会关闭扬声器，扬声器仍可用于其他音频输出。
     * @return
     *        - 0：接口调用成功。
     *        - -2：传入的用户 ID 为空字符串。
     */
    /**
     * @locale en
     * @type api
     * @brief Whether to receive the audio stream of a specific user. Call this insterface will not affect the state of the speaker or other audio output devices.
     * @param user_id User ID, a non-empty string with a maximum length of 128 bytes. The supported character set range is:<br>
     *            1. 26 capital letters A ~ Z<br>
     *            2. 26 lowercase letters a ~ z<br>
     *            3. 10 numbers 0 ~ 9<br>
     *            4. Underscore "_", at sign "@", minus sign "-"
     * @param enable
     *        - true：Receive the user's audio stream.
     *        - false：Default setting. Do not receive the user's audio stream. The speaker will not be muted and can still be used for other audio outputs.
     * @return
     *        - 0: Success.
     *        - -2: Failure. The provided `user_id` is an empty string.
     */
    virtual int enableAudioReceive(const char* user_id, bool enable) = 0;
    /**
     * @locale zh
     * @type api
     * @brief 调节某个房间内所有远端用户的音频播放音量。
     * @param volume 音频播放音量值和原始音量的比值，范围是 [0, 400]，单位为 %，自带溢出保护。为保证更好的通话质量，建议将 volume 值设为 [0,100]。 <br>
     *              - 0: 静音
     *              - 100: 原始音量，默认值
     *              - 400: 最大可为原始音量的 4 倍(自带溢出保护)
     * @return
     *        - 0: 调用成功。
     *        - < 0 : 调用失败。查看 ReturnStatus{@link #ReturnStatus} 获得更多错误说明
     * @note 假设某远端用户 A 始终在被调节的目标用户范围内， <br>
     *        - 该方法与 setRemoteAudioPlaybackVolume{@link #IRTCEngine#setRemoteAudioPlaybackVolume} 互斥，最新调用的任一方法设置的音量将覆盖此前已设置的音量，效果不叠加；
     *        - 当该方法与 setPlaybackVolume{@link #IRTCEngine#setPlaybackVolume} 方法共同使用时，本地收听用户 A 的音量将为两次设置的音量效果的叠加。
     * @list 音频管理
     */
    /**
     * @locale en
     * @type api
     * @brief Adjusts the audio playback volume from all the remote users in a room.
     * @param volume Ratio(%) of playback volume to original volume, in the range [0, 400], with overflow protection. <br>
     *               To ensure the audio quality, we recommend setting the volume to `100`. <br>
     *               - 0: mute
     *               - 100: original volume. Default value.
     *               - 400: Up to 4 times the original volume (with overflow protection)
     * @return
     *        - 0: Success.
     *        - < 0 : Fail. See ReturnStatus{@link #ReturnStatus} for more details
     * @note Suppose a remote user A is always within the range of the target user whose playback volume will be adjusted, <br>
     *        - If you use both this method and setRemoteAudioPlaybackVolume{@link #IRTCEngine#setRemoteAudioPlaybackVolume}, the volume that the local user hears from user A is the volume set by the method called later.
     *        - If you use both this method and setPlaybackVolume{@link #IRTCEngine#setPlaybackVolume}, the volume that the local user hears from user A is the overlay of both settings.
     * @list Audio Management
     */
    virtual int setRemoteRoomAudioPlaybackVolume(int volume) = 0;
};

} //bytertc