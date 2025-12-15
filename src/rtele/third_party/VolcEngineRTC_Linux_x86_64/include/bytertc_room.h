/*
 * Copyright (c) 2020 The VolcEngineRTC project authors. All Rights Reserved.
 * @brief VolcEngineRTC Room Interface
 */

#pragma once

#include "rtc/bytertc_video_effect_interface.h"  // NOLINT
#include "rtc/bytertc_defines.h"
#include "bytertc_room_event_handler.h"
#include "bytertc_rts_room.h"
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
class IRTCRoom : public IRTSRoom {
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
    virtual ~IRTCRoom() {
    }
    /**
     * @locale zh
     * @type api
     * @brief 退出并销毁调用 createRTCRoom{@link #IRTCEngine#createRTCRoom} 所创建的 RTC 房间实例。
     * @list 房间管理
     */
    /**
     * @locale en
     * @type api
     * @brief  Leave and destroy the RTC room instance created by calling createRTCRoom{@link #IRTCEngine#createRTCRoom}.
     * @list Room Management
     */
    virtual void destroy() = 0;
    /**
     * @locale zh
     * @type api
     * @brief 设置用户可见性。未调用该接口前，本地用户默认对他人可见。 <br>
     *        默认情况下，一个 RTC 房间最多同时容纳 50 名可见用户，最多 30 人可同时上麦。更多信息参看[用户和媒体流上限](https://www.volcengine.com/docs/6348/257549)。
     * @param enable 设置用户是否对房间内其他用户可见： <br>
     *        - true: 可见，用户可以在房间内发布音视频流，房间中的其他用户将收到用户的行为通知，例如进房、开启视频采集和退房。
     *        - false: 不可见，用户不可以在房间内发布音视频流，房间中的其他用户不会收到用户的行为通知，例如进房、开启视频采集和退房。
     * @return
     *        - 0: 调用成功。
     *        - < 0: 调用失败。参看 ReturnStatus{@link #ReturnStatus} 获得更多错误说明。
     *          设置用户可见性，会收到设置成功/失败回调 onUserVisibilityChanged{@link #IRTCRoomEventHandler#onUserVisibilityChanged}。（v3.54 新增）
     *        - 在加入房间前设置用户可见性，若设置的可见性与默认值不同，将在加入房间时触发本回调。
     *        - 在加入房间后设置用户可见性，若可见性前后不同，会触发本回调。
     *        - 断网重连后，若可见性发生改变，会触发本回调。
     * @note
     *       - 在加入房间前后，用户均可调用此方法设置用户可见性。
     *       - 在房间内，调用此方法成功切换用户可见性后，房间内其他用户会收到相应的回调。
     *   &#x0020;  • 从可见换至不可见时，房间内其他用户会收到 onUserLeave{@link #IRTSRoomEventHandler#onUserLeave}。 <br>
     *   &#x0020;  • 从不可见切换至可见时，房间内其他用户会收到 onUserJoined{@link #IRTSRoomEventHandler#onUserJoined}。 <br>
     *   &#x0020;  • 若调用该方法将可见性设为 `false`，此时尝试发布流会收到 `kWarningCodePublishStreamForbidden` 警告。
     * @list 房间管理
     */
    /**
     * @locale en
     * @type api
     * @brief Set the visibility of the user in the room. The local user is visible to others by default before calling this API. <br>
     *        An RTC room can accommodate a maximum of 50 visible users, and 30 media streams can be published simultaneously. For more information, see [Room Capacity](https://docs.byteplus.com/en/byteplus-rtc/docs/257549).
     * @param enable Visibility of the user in the room. <br>
     *        - true: The user can publish media streams. The other users in the room get informed of the behaviors of the user, such as joining room, starting video capture, and leaving room.
     *        - false: The user cannot publish media streams. The other users in the room do not get informed of the behaviors of the user, such as joining room, starting video capture, or leaving room.
     * @return
     *        - 0: Success.
     *        - < 0 : Failure. See ReturnStatus{@link #ReturnStatus}.
     *        You will receive onUserVisibilityChanged{@link #IRTCRoomEventHandler#onUserVisibilityChanged} after calling this API. (Available since v3.54)
     *        - If you call this API before joining room, and the set value is different from the default value, you will receive the callback when you join the room.
     *        - If you call this API after joining room, and the current visibility is different from the previous value, you will receive the callback.
     *        - When reconnecting after losing internet connection, if the visibility changes, you will receive the callback.
     * @note
     *        - You can call this API whether the user is in a room or not.
     *        - When you call this API while you are in the room, the other users in the room will be informed via the following callback.
     *   &#x0020;  • When you switch from `false` to `true`, they will receive onUserJoined{@link #IRTSRoomEventHandler#onUserJoined}. <br>
     *   &#x0020;  • When you switch from `true` to `false`, they will receive onUserLeave{@link #IRTSRoomEventHandler#onUserLeave}. <br>
     *   &#x0020;  • The invisible user will receive the warning code, `kWarningCodePublishStreamForbidden`, when trying to publish media streams.
     */
    virtual int setUserVisibility(bool enable) = 0;
    /**
     * @locale zh
     * @type api
     * @brief 通过设置 IRTCRoom{@link #IRTCRoom} 对象的事件句柄，监听此对象对应的回调事件。
     * @param room_event_handler 参见 IRTCRoomEventHandler{@link #IRTCRoomEventHandler}
     * @return
     *        - 0: 调用成功。
     *        - < 0 : 调用失败。查看 ReturnStatus{@link #ReturnStatus} 获得更多错误说明
     * @list 房间管理
     */
    /**
     * @locale en
     * @type api
     * @brief Listens for event callbacks related to the IRTCRoom{@link #IRTCRoom} instance by setting the event handler of this instance.
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
     * @brief 加入 RTC 房间。 <br>
     *        调用 createRTCRoom{@link #IRTCEngine#createRTCRoom} 创建房间后，调用此方法加入房间，同房间内其他用户进行音视频通话。   
     * @param token 动态密钥，用于对登录用户进行鉴权验证。 <br>
     *        进入房间需要携带 Token。测试时可使用控制台生成临时 Token，正式上线需要使用密钥 SDK 在您的服务端生成并下发 Token。Token 有效期及生成方式参看[使用 Token 完成鉴权](70121)。 <br>
     *       - 使用不同 App ID 的 App 是不能互通的。
     *       - 请务必保证生成 Token 使用的 App ID 和创建引擎时使用的 App ID 相同，否则会导致加入房间失败。
     * @param user_info 用户信息，参看 UserInfo{@link #UserInfo}。
     * @param user_visibility 用户可见性。建议在进房时将用户可见性都设置为 `false`，并在用户需要发送音视频流时再通过 setUserVisibility{@link #IRTCRoom#setUserVisibility} 设置为 `true`。从而避免因房间内用户达到数量上限所导致的进房失败。默认情况下，一个 RTC 房间最多同时容纳 50 名可见用户，其中最多 30 人可同时上麦，更多信息参看[用户和媒体流上限](https://www.volcengine.com/docs/6348/257549)。
     * @param config 房间参数配置，设置房间模式以及是否自动发布或订阅流。具体配置模式参看 RTCRoomConfig{@link #RTCRoomConfig}。
     * @return
     *        -  0: 成功。触发以下回调：
     *          - 本端收到房间状态通知 onRoomStateChanged{@link #IRTSRoomEventHandler#onRoomStateChanged} 回调。
     *          - 本端收到本地流发布状态通知 onVideoPublishStateChanged{@link #IRTCRoomEventHandler#onVideoPublishStateChanged}、onAudioPublishStateChanged{@link #IRTCRoomEventHandler#onAudioPublishStateChanged} 回调。
     *          - 本端收到流订阅状态通知 onVideoSubscribeStateChanged{@link #IRTCRoomEventHandler#onVideoSubscribeStateChanged}、onAudioSubscribeStateChanged{@link #IRTCRoomEventHandler#onAudioSubscribeStateChanged} 回调。
     *          - 本端收到房间内已发布流的通知 onUserPublishStreamVideo{@link #IRTCRoomEventHandler#onUserPublishStreamVideo}、onUserPublishStreamAudio{@link #IRTCRoomEventHandler#onUserPublishStreamAudio} 回调。 
     *          - 如果本端用户为可见用户，房间内其他用户收到 onUserJoined{@link #IRTCRoomEventHandler#onUserJoined} 回调通知。
     *        - -1：room_id / user_info.uid 包含了无效的参数。
     *        - -2：已经在房间内。接口调用成功后，只要收到返回值为 0 ，且未调用 leaveRoom{@link #IRTSRoom#leaveRoom} 成功，则再次调用进房接口时，无论填写的房间 ID 和用户 ID 是否重复，均触发此返回值。
     *        调用失败时，具体失败原因会通过 onRoomStateChanged{@link #IRTSRoomEventHandler#onRoomStateChanged} 回调告知。
     * @note
     *       - 同一个 App ID 的同一个房间内，每个用户的用户 ID 必须是唯一的。如果两个用户的用户 ID 相同，则后进房的用户会将先进房的用户踢出房间，并且先进房的用户会收到 onRoomStateChanged{@link #IRTSRoomEventHandler#onRoomStateChanged} 回调通知，错误类型详见 ErrorCode{@link #ErrorCode} 中的 kErrorCodeDuplicateLogin。
     *       - 用户加入房间成功后，在本地网络状况不佳的情况下，SDK 可能会与服务器失去连接，并触发 onConnectionStateChanged{@link #IRTCEngineEventHandler#onConnectionStateChanged} 回调。此时 SDK 会自动重试，直到成功重连。重连成功后，本地会收到 onRoomStateChanged{@link #IRTSRoomEventHandler#onRoomStateChanged} 回调通知。
     * @list 房间管理
     */
    /**
     * @locale en
     * @type api
     * @brief Join an RTC room. <br>
     *        After creating a room by calling createRTCRoom{@link #IRTCEngine#createRTCRoom}, call this API to join the room and make audio & video calls with other users in the room.
     * @param token Dynamic key for authenticating the logged-in user. <br>
     *         You need to bring Token to enter the room. When testing, you can use the console to generate temporary tokens. The official launch requires the use of the key SDK to generate and issue tokens at your server level. See [Authentication with Token](70121) for token validity and generation method. <br>
     *        - Apps with different App IDs cannot communicate with each other.
     *        - Make sure that the App ID used to generate the Token is the same as the App ID used to create the engine, otherwise it will cause the join room to fail.
     * @param user_info User information. See UserInfo{@link #UserInfo}.
     * @param user_visibility User visibility. We recommend setting the user visibility to `false` when entering the room, and then setting it to `true` via setUserVisibility{@link #IRTCRoom#setUserVisibility} when the user needs to send audio & video streams. Joining fails when the number of users in an RTC room reaches the upper limit. An RTC room can accommodate a maximum of 50 visible users, and 30 media streams can be published simultaneously. For more information, see [Capability of Users and Streams](https://docs.byteplus.com/en/byteplus-rtc/docs/257549).
     * @param config Room parameter configuration, set the room mode and whether to automatically publish or subscribe to the flow. See RTCRoomConfig{@link #RTCRoomConfig} for the specific configuration mode.
     * @return
     *         - 0: Success. 
     *            - Local users receive notifications of the room state via onRoomStateChanged{@link #IRTSRoomEventHandler#onRoomStateChanged}. 
     *            - Local users receive notifications of the local-stream state via onVideoPublishStateChanged{@link #IRTCRoomEventHandler#onVideoPublishStateChanged}, onAudioPublishStateChanged{@link #IRTCRoomEventHandler#onAudioPublishStateChanged}.
     *            - Local users receive notifications of the subscribed-stream state via onUserPublishStreamVideo{@link #IRTCRoomEventHandler#onUserPublishStreamVideo}, onUserPublishStreamAudio{@link #IRTCRoomEventHandler#onUserPublishStreamAudio}.
     *            - Local users receive notifications of the published-stream state via onUserPublishStreamVideo{@link #IRTCRoomEventHandler#onUserPublishStreamVideo}, onUserPublishStreamAudio{@link #IRTCRoomEventHandler#onUserPublishStreamAudio}.
     *            - If the local user is also a visible user, the other participants receive onUserJoined{@link #IRTCRoomEventHandler#onUserJoined} callback.
     *         - -1: Room_id/user_info.uid contains invalid parameters.
     *         - -2: Already in the room. After the interface call is successful, as long as the return value of 0 is received and leaveRoom{@link #IRTSRoom#leaveRoom} is not called successfully, this return value is triggered when the room entry interface is called again, regardless of whether the filled room ID and user ID are duplicated.
     *         The reason for the failure will be communicated via the onRoomStateChanged{@link #IRTSRoomEventHandler#onRoomStateChanged} callback.
     * @note
     *        - In the same room with the same App ID, the user ID of each user must be unique. If two users have the same user ID, the user who entered the room later will kick the user who entered the room out of the room, and the user who entered the room will receive the onRoomStateChanged{@link #IRTSRoomEventHandler#onRoomStateChanged} callback notification. For the error type. See kErrorCodeDuplicateLogin in ErrorCode{@link #ErrorCode}.
     *        - After the user successfully joins the room, the SDK may lose connection to the server in case of poor local network conditions. At this point, onConnectionStateChanged{@link #IRTCEngineEventHandler#onConnectionStateChanged} callback will be triggered and the SDK will automatically retry until it successfully reconnects to the server. After a successful reconnection, you will receive a local onRoomStateChanged{@link #IRTSRoomEventHandler#onRoomStateChanged} callback notification.
     * @list Room Management
     */
    virtual int joinRoom(const char* token, const UserInfo& user_info, bool user_visibility, const RTCRoomConfig& config) = 0;
    /**
     * @locale zh
     * @type api
     * @valid since 3.60. 自 3.60 起，该接口替代了 `publishStream` 和 `unpublishStream` 方法来实现下述功能。如果你已升级至 3.60 及以上版本，且仍在使用这两个方法，请迁移至该接口。
     * @region 房间管理
     * @brief 手动发布/取消发布本地摄像头采集的视频流。
     * @param publish 指定是否发布本地摄像头采集的视频流。<br>
     *        - `true`: 发布。
     *        - `false`: 取消发布。
     * @return
     *        - 0: 调用成功。
     *        - < 0 : 调用失败。查看 ReturnStatus{@link #ReturnStatus} 获得更多错误说明
     * @note
     *        - 如果你已经在用户进房时通过调用 joinRoom{@link #IRTCRoom#joinRoom} 成功选择了自动发布，则无需再调用本接口。
     *        - 调用 setUserVisibility{@link #IRTCRoom#setUserVisibility} 方法将自身设置为不可见后无法调用该方法，需将自身切换至可见后方可调用该方法发布摄像头视频流。
     *        - 如果你需要发布麦克风采集到的音频流，调用 publishStreamAudio{@link #IRTCRoom#publishStreamAudio}。
     *        - 如果你需要向多个房间发布流，调用 startForwardStreamToRooms{@link #IRTCRoom#startForwardStreamToRooms}。
     *        - 调用此方法后，房间中的所有远端用户会收到 onUserPublishStreamVideo{@link #IRTCRoomEventHandler#onUserPublishStreamVideo} 回调通知，订阅了视频流的远端用户会收到 onFirstRemoteVideoFrameDecoded{@link #IRTCEngineEventHandler#onFirstRemoteVideoFrameDecoded} 回调。
     * @list 音视频传输
     */
    /**
     * @locale en
     * @type api
     * @valid since 3.60. Since version 3.60, this interface replaces the `publishStream` and `unpublishStream` methods for the below functions. If you have upgraded to version 3.60 or later and are still using these methods, please migrate to this interface.
     * @brief Publishes or stops publishing video streams captured by the local camera in the current room.
     * @param publish Whether to publish the video stream.<br>
     *        - `true`: Publish.
     *        - `false`: Stop publishing.
     * @return
     *        - 0: Success.
     *        - < 0 : Fail. See ReturnStatus{@link #ReturnStatus} for more details
     * @note
     *        - You don't need to call this API if you set it to Auto-publish when calling joinRoom{@link #IRTCRoom#joinRoom}.
     *        - An invisible user cannot publish media streams. Call setUserVisibility{@link #IRTCRoom#setUserVisibility} to change your visibility in the room.
     *        - Call publishStreamAudio{@link #IRTCRoom#publishStreamAudio} to start or stop publishing the audio stream captured by the microphone.
     *        - Call startForwardStreamToRooms{@link #IRTCRoom#startForwardStreamToRooms} to forward the published streams to the other rooms.
     *        - After you call this API, the other users in the room will receive onUserPublishStreamVideo{@link #IRTCRoomEventHandler#onUserPublishStreamVideo}. Those who successfully received your streams will receive onFirstRemoteVideoFrameDecoded{@link #IRTCEngineEventHandler#onFirstRemoteVideoFrameDecoded} at the same time.
     */
    virtual int publishStreamVideo(bool publish) = 0;

    /**
     * @locale zh
     * @type api
     * @valid since 3.60. 自 3.60 起，该接口替代了 `publishStream` 和 `unpublishStream` 方法来实现下述功能。如果你如果你已升级至 3.60 及以上版本，并且仍在使用这两个方法，请迁移到此接口。
     * @region 房间管理
     * @brief 发布/取消发布本地麦克风采集的音频流。
     * @param publish 指定是否发布本地麦克风采集的音频流。<br>
     *        - `true`: 发布。
     *        - `false`: 取消发布。
     * @return
     *        - 0: 调用成功。
     *        - < 0 : 调用失败。查看 ReturnStatus{@link #ReturnStatus} 获得更多错误说明
     * @note
     *        - 如果你已经在用户进房时通过调用 joinRoom{@link #IRTCRoom#joinRoom} 成功选择了自动发布，则无需再调用本接口。
     *        - 调用 setUserVisibility{@link #IRTCRoom#setUserVisibility} 方法将自身设置为不可见后无法调用该方法，需将自身切换至可见后方可调用该方法发布音频流。
     *        - 如果你需要发布摄像头采集到的视频流，调用 publishStreamVideo{@link #IRTCRoom#publishStreamVideo}。
     *        - 如果你需要向多个房间发布流，调用 startForwardStreamToRooms{@link #IRTCRoom#startForwardStreamToRooms}。
     *        - 调用此方法后，房间中的所有远端用户会收到 onUserPublishStreamAudio{@link #IRTCRoomEventHandler#onUserPublishStreamAudio} 回调通知，其中成功收到了音频流的远端用户会收到 onFirstRemoteAudioFrame{@link #IRTCEngineEventHandler#onFirstRemoteAudioFrame} 回调。
     * @list 音视频传输
     * @order 0
     */
    /**
     * @locale en
     * @type api
     * @valid since 3.60. Since version 3.60, this interface replaces the `publishStream` and `unpublishStream` methods for the below functions. If you have upgraded to version 3.60 or later and are still using these methods, please migrate to this interface.
     * @brief Publishes or stops publishing audio streams captured by the local microphone in the current room.
     * @param publish Whether to publish the audio stream.<br>
     *        - `true`: Publish.
     *        - `false`: Stop publishing.
     * @return
     *        - 0: Success.
     *        - < 0 : Fail. See ReturnStatus{@link #ReturnStatus} for more details
     * @note
     *        - You don't need to call this API if you set it to Auto-publish when calling joinRoom{@link #IRTCRoom#joinRoom}.
     *        - An invisible user cannot publish media streams. Call setUserVisibility{@link #IRTCRoom#setUserVisibility} to change your visibility in the room.
     *        - Call publishStreamVideo{@link #IRTCRoom#publishStreamVideo} to start or stop publishing the video stream captured by the camera.
     *        - Call startForwardStreamToRooms{@link #IRTCRoom#startForwardStreamToRooms} to forward the published streams to the other rooms.
     *        - After you call this API, the other users in the room will receive onUserPublishStreamAudio{@link #IRTCRoomEventHandler#onUserPublishStreamAudio}. Those who successfully received your streams will receive onFirstRemoteAudioFrame{@link #IRTCEngineEventHandler#onFirstRemoteAudioFrame} at the same time.
     * @list Audio & Video Transport
     */
    virtual int publishStreamAudio(bool publish) = 0;

    /**  
     * @locale zh
     * @type api
     * @valid since 3.60. 自 3.60 起，该接口替代了 `subscribeStream` 和 `unsubscribeStream` 方法来实现下述功能。如果你已升级至 3.60 及以上版本，且仍在使用这两个方法，请迁移至该接口。
     * @brief 订阅/取消订阅房间内指定的远端视频流（通过摄像头采集的）。
     * @param stream_id 目标远端视频流 ID。
     * @param subscribe 是否订阅该视频流。<br>
     *        - `true`: 订阅。
     *        - `false`: 取消订阅。
     * @return 方法调用结果： <br>
     *        - 0：成功；
     *        - <0：失败。具体失败原因参看 ReturnStatus{@link #ReturnStatus}。
     * @note
     *        - 若当前用户在调用本接口时已经订阅该远端视频流（手动订阅或自动订阅），则将根据本次传入的参数，更新订阅配置。
     *        - 你必须先通过 onUserPublishStreamVideo{@link #IRTCRoomEventHandler#onUserPublishStreamVideo} 回调获取当前房间里的远端摄像头流信息，然后调用本方法按需订阅。
     *        - 调用该方法后，你会收到 onVideoSubscribeStateChanged{@link #IRTCRoomEventHandler#onVideoSubscribeStateChanged} 通知方法调用结果。
     *        - 成功订阅远端用户的媒体流后，订阅关系将持续到调用 subscribeStreamVideo{@link #IRTCRoom#subscribeStreamVideo} 取消订阅或本端用户退房。
     *        - 关于其他调用异常，你会收到 onVideoSubscribeStateChanged{@link #IRTCRoomEventHandler#onVideoSubscribeStateChanged} 回调通知，具体异常原因参看 SubscribeStateChangeReason{@link #SubscribeStateChangeReason}。
     * @list 音视频传输
     * @order 3
     */
    /**
     * @locale en
     * @type api
     * @valid since 3.60. Since version 3.60, this interface replaces the `subscribeStream` and `unsubscribeStream` methods for the below functions. If you have upgraded to version 3.60 or later and are still using these methods, please migrate to this interface.
     * @brief Subscribes to or unsubscribes from the specific remote video streams captured by the local camera.
     * @param stream_id The ID of the remote video stream.
     * @param subscribe Whether to subscribe to the video stream.<br>
     *        - `true`: Subscribe.
     *        - `false`: Unsubscribe.
     * @return API call result: <br>
     *        - 0: Success.
     *        - <0: Failure. See ReturnStatus{@link #ReturnStatus} for specific reasons.
     * @note
     *        - Calling this API to update the subscribe configuration when the user has subscribed the remote user either by calling this API or by auto-subscribe.
     *        - You must first get the remote stream information through onUserPublishStreamVideo{@link #IRTCRoomEventHandler#onUserPublishStreamVideo} before calling this API to subscribe to streams accordingly.
     *        - After calling this API, you will be informed of the calling result with onVideoSubscribeStateChanged{@link #IRTCRoomEventHandler#onVideoSubscribeStateChanged}.
     *        - Once the local user subscribes to the stream of a remote user, the subscription to the remote user will sustain until the local user leaves the room or unsubscribe from it by calling subscribeStreamVideo{@link #IRTCRoom#subscribeStreamVideo}.
     *        - Any other exceptions will be included in onVideoSubscribeStateChanged{@link #IRTCRoomEventHandler#onVideoSubscribeStateChanged}, see SubscribeStateChangeReason{@link #SubscribeStateChangeReason} for the reasons.
     * @list Audio & Video Transport
     */
    virtual int subscribeStreamVideo(const char *stream_id, bool subscribe) = 0;

    /**
     * @locale zh
     * @type api
     * @valid since 3.60. 自 3.60 起，该接口替代了 `subscribeStream` 和 `unsubscribeStream` 方法来实现下述功能。如果你已升级至 3.60 及以上版本，且仍在使用这两个方法，请迁移至该接口。
     * @brief 订阅/取消订阅房间内指定的远端音频流（通过麦克风采集的）。
     * @param stream_id 目标远端音频流 ID。
     * @param subscribe 是否订阅该音频流。<br>
     *        - `true`: 订阅。
     *        - `false`: 取消订阅。
     * @return 方法调用结果： <br>
     *        - 0：成功；
     *        - <0：失败。具体失败原因参看 ReturnStatus{@link #ReturnStatus}。
     * @note
     *        - 若当前用户在调用本接口时已经订阅该远端用户（手动订阅或自动订阅），则将根据本次传入的参数，更新订阅配置。
     *        - 你必须先通过 onUserPublishStreamAudio{@link #IRTCRoomEventHandler#onUserPublishStreamAudio} 回调获取当前房间里的远端麦克风流信息，然后调用本方法按需订阅。
     *        - 调用该方法后，你会收到 onAudioSubscribeStateChanged{@link #IRTCRoomEventHandler#onAudioSubscribeStateChanged} 通知方法调用结果。
     *        - 成功订阅远端用户的媒体流后，订阅关系将持续到调用 subscribeStreamAudio{@link #IRTCRoom#subscribeStreamAudio} 取消订阅或本端用户退房。
     *        - 关于其他调用异常，你会收到 onAudioSubscribeStateChanged{@link #IRTCRoomEventHandler#onAudioSubscribeStateChanged} 回调通知，具体异常原因参看 SubscribeStateChangeReason{@link #SubscribeStateChangeReason}。
     * @list 音视频传输
     */
    /**
     * @locale en
     * @type api
     * @valid since 3.60. Since version 3.60, this interface replaces the `subscribeStream` and `unsubscribeStream` methods for the below functions. If you have upgraded to version 3.60 or later and are still using these methods, please migrate to this interface.
     * @brief Subscribes to or unsubscribes from the specific remote audio streams captured by the local microphone.
     * @param stream_id The ID of the target remote audio stream.
     * @param subscribe Whether to subscribe to the audio stream.<br>
     *        - `true`: Subscribe.
     *        - `false`: Unsubscribe.   
     * @return API call result: <br>
     *        - 0: Success.
     *        - <0: Failure. See ReturnStatus{@link #ReturnStatus} for specific reasons.
     * @note
     *        - Calling this API to update the subscribe configuration when the user has subscribed the remote user either by calling this API or by auto-subscribe.
     *        - You must first get the remote stream information through onUserPublishStreamAudio{@link #IRTCRoomEventHandler#onUserPublishStreamAudio} before calling this API to subscribe to streams accordingly.
     *        - After calling this API, you will be informed of the calling result with onAudioSubscribeStateChanged{@link #IRTCRoomEventHandler#onAudioSubscribeStateChanged}.
     *        - Once the local user subscribes to the stream of a remote user, the subscription to the remote user will sustain until the local user leaves the room or unsubscribe from it by calling subscribeStreamAudio{@link #IRTCRoom#subscribeStreamAudio}.
     *        - Any other exceptions will be included in onAudioSubscribeStateChanged{@link #IRTCRoomEventHandler#onAudioSubscribeStateChanged}, see ErrorCode{@link #ErrorCode} for the reasons.
     * @list Audio & Video Transport
     */
    virtual int subscribeStreamAudio(const char *stream_id, bool subscribe) = 0;

    /** 
     * @locale zh
     * @type api
     * @valid since 3.60. 
     * @brief 设置期望订阅的远端视频流类型。比如大流、中流、小流等。
     * @param stream_id 目标要订阅的远端视频流 ID。
     * @param stream_type 远端视频流类型，参看 SimulcastStreamType{@link #SimulcastStreamType}。
     * @return 方法调用结果： <br>
     *        - 0：成功；
     *        - <0：失败。具体失败原因参看 ReturnStatus{@link #ReturnStatus}。
     * @note
     *        - 该方法仅在发布端调用 setLocalSimulcastMode{@link #IRTCEngine#setLocalSimulcastMode} 开启了发送多路视频流的情况下生效。
     *        - 若发布端开启了推送多路流功能，但订阅端不对流参数进行设置，则默认接受发送端设置的分辨率最大的一路视频流。该方法可在进房后调用。
     *        - 仅对远端发布的摄像头视频流生效。
     * @list 视频管理
     * @order 1
     */
    /** 
     * @locale en
     * @type api
     * @valid since 3.60.
     * @brief Sets the type of the remote video stream to subscribe to.
     * @param stream_id ID of the remote video stream you expect to subscribe to.
     * @param stream_type Video stream type, see SimulcastStreamType{@link #SimulcastStreamType}. 
     * @return API call result: 
     *        - 0: Success.
     *        - <0: Failure. See ReturnStatus{@link #ReturnStatus} for specific reasons.
     * @note
     *        - This API only works after the publisher calls setLocalSimulcastMode{@link #IRTCEngine#setLocalSimulcastMode} to enable publishing multiple-quality video streams, in which case the subscriber will receive the stream from the publisher that is closest to the set configuration; otherwise the subscriber will only receive one video stream with a resolution of 640px × 360px and a frame rate of 15fps.<br>
     *        - If you don't call this API after the publisher enables the function of publishing multiple streams, you will receive by default the video stream with the largest resolution set by the publisher.
     *        - This API only affects camera video streams published by remote users.
     * @list Video Management
     */
    virtual int setRemoteSimulcastStreamType(const char* stream_id, SimulcastStreamType stream_type) = 0;

    /**
     * @locale zh
     * @type api
     * @list 视频管理
     * @brief 设置期望订阅的远端视频流的参数。
     * @param stream_id 期望配置订阅参数的远端视频流 ID。
     * @param remote_video_config 期望配置的远端视频流参数，参看 RemoteVideoConfig{@link #RemoteVideoConfig}。
     * @return 方法调用结果： <br>
     *        + 0：成功；<br>
     *        + !0：失败。
     * @note 
     *        + 若使用 342 及以前版本的 SDK，调用该方法前请联系技术支持人员开启按需订阅功能。  <br>
     *        + 该方法需在进房后调用。
     *        + 该方法仅在发布端调用 setLocalSimulcastMode{@link #IRTCEngine#setLocalSimulcastMode} 开启了发送多路视频流的情况下生效，此时订阅端将收到来自发布端与期望设置的参数最相近的一路流；否则订阅端只会收到一路参数为分辨率 640px × 360px、帧率 15fps 的视频流。  <br>
     *        + 若发布端开启了推送多路流功能，但订阅端不对流参数进行设置，则默认接受发送端设置的分辨率最大的一路视频流。  <br>     
     *        + SDK 会根据发布端和所有订阅端的设置灵活调整视频流的参数，具体调整策略详见[推送多路流](https://www.volcengine.com/docs/6348/70139)文档。
     */
    /** 
     * @locale en
     * @type api
     * @brief Sets your expected configuration of the remote video stream that you want to subscribe to or have subscribed to.
     * @param stream_id ID of the remote video stream you want to configure subscription parameters for.
     * @param remote_video_config Parameters you want to configure for the remote video stream, see RemoteVideoConfig{@link #RemoteVideoConfig}.
     * @return API call result: <br>
     *        + 0: Success.<br>
     *        + !0: Failure. See ReturnStatus{@link #ReturnStatus} for specific reasons.
     * @note 
     *        + This API only works after the publisher calls setLocalSimulcastMode{@link #IRTCEngine#setLocalSimulcastMode} to enable publishing multiple video streams, in which case the subscriber will receive the stream from the publisher that is closest to the set configuration; otherwise the subscriber will only receive one video stream with a resolution of 640px × 360px and a frame rate of 15fps..  <br>
     *        + If you don't call this API after the publisher enables the function of publishing multiple streams, you will receive by default the video stream with the largest resolution set by the publisher.<br>
     *        + You should call this API in the room. 
     *        + SDK will automatically select the stream to be published or subcribed based on the settings of both sides see for details.
     * @list Video Management
     */
    virtual int setRemoteVideoConfig(const char *stream_id, const RemoteVideoConfig &remote_video_config) = 0;
    /**
     * @locale zh
     * @type api
     * @valid Available since 3.60
     * @brief 设置当前推流的流附加信息。
     * @param extra_info 流附加信息。长度不超过1024的字符串。
     * @return 方法调用结果： <br>
     *        + 0：成功；<br>
     *        + !0：失败。
     * @note 
     *        + 可通过此函数设置当前推流的流附加信息。流附加信息是流 ID 的附加信息标识，不同于流 ID 在推流过程中不可修改，流附加信息可以在对应流 ID 的推流中途修改。开发者可根据流附加信息来实现流 ID 相关的可变内容的同步。
     *        + 该方法在进房前后均可调用
     *        + 相同房间内的其他用户会通过 onRoomStreamExtraInfoUpdate{@link #IRTCRoomEventHandler#onRoomStreamExtraInfoUpdate} 回调函数获得通知。
     */ 
    /** 
     * @locale en
     * @type api
     * @valid Available since 3.60
     * @brief Sets the stream extra info.
     * @param extra_info Stream extra info. The length cannot exceed 1024 bytes.
     * @return API call result: 
     *        - 0: Success.
     *        - <0: Failure. See ReturnStatus{@link #ReturnStatus} for specific reasons.
     * @note
     *        - This function can be used to set the stream extra information for the current streaming. The stream additional information serves as an additional information identifier for the stream ID. Unlike the stream ID, which cannot be modified during the streaming process, the stream additional information can be updated midway through the streaming of the corresponding stream ID. Developers can leverage the stream additional information to achieve synchronization of variable content related to the stream ID.
     *        - This method can be called either before or after entering the room.
     *        - Other users in the same room will receive notifications via the onRoomStreamExtraInfoUpdate{@link #IRTCRoomEventHandler#onRoomStreamExtraInfoUpdate} callback function.
     * @list 
     */
    virtual int setStreamExtraInfo(const char* extra_info) = 0;
    /**
     * @locale zh
     * @type api
     * @valid since 3.60. 自 3.60 起，该接口替代了 `subscribeAllStreams` 和 `unsubscribeAllStreams` 方法来实现下述功能。如果你已升级至 3.60 及以上版本，且仍在使用这两个方法，请迁移至该接口。
     * @brief 订阅/取消订阅房间内所有远端视频流（通过摄像头采集的）。
     * @param subscribe 是否订阅所有远端视频流。<br>
     *                - `true`: 订阅。
     *                - `false`: 取消订阅。
     * @return
     *        - 0： 成功。
     *        - < 0： 失败。查看 ReturnStatus{@link #ReturnStatus} 获得更多错误说明。
     * @note
     *        - 多次调用订阅接口时，将根据末次调用接口和传入的参数，更新订阅配置：subscribeStreamVideo{@link #IRTCRoom#subscribeStreamVideo}、subscribeStreamAudio{@link #IRTCRoom#subscribeStreamAudio}。
     *        - 开启音频选路后，如果房间内的媒体流超过上限，建议通过调用单流订阅接口逐一指定需要订阅的媒体流。
     *        - 调用该方法后，你会收到 onVideoSubscribeStateChanged{@link #IRTCRoomEventHandler#onVideoSubscribeStateChanged}、onAudioSubscribeStateChanged{@link #IRTCRoomEventHandler#onAudioSubscribeStateChanged} 通知方法调用结果，包含异常原因。
     *        - 成功订阅远端用户的媒体流后，订阅关系将持续到调用 subscribeStreamVideo{@link #IRTCRoom#subscribeStreamVideo}、subscribeStreamAudio{@link #IRTCRoom#subscribeStreamAudio} 取消订阅或本端用户退房。
     * @list 音视频传输
     */
    /**
     * @locale en
     * @type api
     * @valid since 3.60. Since version 3.60, this interface replaces the `subscribeAllStreams` and `unsubscribeAllStreams` methods for the below functions. If you have upgraded to version 3.60 or later and are still using these methods, please migrate to this interface.
     * @brief Subscribes to or unsubscribes from all remote video streams (by the camera).
     * @param subscribe Whether to subscribe to all remote video streams.<br>
     *                  - `true`: Subscribe.
     *                  - `false`: Unsubscribe.
     * @return API call result: <br>
     *        - 0: Success.
     *        - < 0: Failure. See ReturnStatus{@link #ReturnStatus} for more details.
     * @note
     *        - If the subscription options conflict with the previous ones, they are subject to the configurations in the last call.
     *        - With the Audio selection enabled, if the number of media streams exceeds the limit, we recommend you subscribe each target media stream with the API follows: subscribeStreamVideo{@link #IRTCRoom#subscribeStreamVideo}, subscribeStreamAudio{@link #IRTCRoom#subscribeStreamAudio}.
     *        - After calling this API, you will be informed of the calling result with onVideoSubscribeStateChanged{@link #IRTCRoomEventHandler#onVideoSubscribeStateChanged}, onAudioSubscribeStateChanged{@link #IRTCRoomEventHandler#onAudioSubscribeStateChanged}. You can also find exceptions cause from the callbacks above.
     *        - Once the local user subscribes to the stream of a remote user, the subscription to the remote user will sustain until the local user leaves the room or unsubscribe from it by calling subscribeStreamVideo{@link #IRTCRoom#subscribeStreamVideo}, subscribeStreamAudio{@link #IRTCRoom#subscribeStreamAudio}.
     * @list Audio & Video Transport
     */
    virtual int subscribeAllStreamsVideo(bool subscribe) = 0;

    /**
     * @locale zh
     * @type api
     * @valid since 3.60. 自 3.60 起，该接口替代了 `subscribeAllStreams` 和 `unsubscribeAllStreams` 方法来实现下述功能。如果你已升级至 3.60 及以上版本，且仍在使用这两个方法，请迁移至该接口。
     * @region 房间管理
     * @brief 订阅/取消订阅房间内所有远端音频流（通过麦克风采集的）。
     * @param subscribe 是否订阅所有远端音频流。<br>
     *                - `true`: 订阅。
     *                - `false`: 取消订阅。
     * @return 方法调用结果： <br>
     *        - 0： 成功。
     *        - < 0： 失败。查看 ReturnStatus{@link #ReturnStatus} 获得更多错误说明。
     * @note
     *        - 多次调用订阅接口时，将根据末次调用接口和传入的参数，更新订阅配置：subscribeStreamVideo{@link #IRTCRoom#subscribeStreamVideo}、subscribeStreamAudio{@link #IRTCRoom#subscribeStreamAudio}。
     *        - 开启音频选路后，如果房间内的媒体流超过上限，建议通过调用单流订阅接口逐一指定需要订阅的媒体流。
     *        - 调用该方法后，你会收到 onVideoSubscribeStateChanged{@link #IRTCRoomEventHandler#onVideoSubscribeStateChanged}、onAudioSubscribeStateChanged{@link #IRTCRoomEventHandler#onAudioSubscribeStateChanged} 通知方法调用结果，包含异常原因。
     *        - 成功订阅远端用户的媒体流后，订阅关系将持续到调用 subscribeStreamVideo{@link #IRTCRoom#subscribeStreamVideo}、subscribeStreamAudio{@link #IRTCRoom#subscribeStreamAudio} 取消订阅或本端用户退房。
     * @list 音视频传输
     * 
     */
    /**
     * @locale en
     * @type api
     * @valid since 3.60. Since version 3.60, this interface replaces the `subscribeAllStreams` and `unsubscribeAllStreams` methods for the below functions. If you have upgraded to version 3.60 or later and are still using these methods, please migrate to this interface.
     * @brief Subscribes to or unsubscribes from all remote audio streams (by the microphone).
     * @param subscribe Whether to subscribe to all remote audio streams.<br>
     *                  - `true`: Subscribe.
     *                  - `false`: Unsubscribe.
     * @return API call result: <br>
     *        - 0: Success
     *        - < 0: Failure. See ReturnStatus{@link #ReturnStatus} for more details.
     * @note
     *        - If the subscription options conflict with the previous ones, they are subject to the configurations in the last call.
     *        - With the Audio selection enabled, if the number of media streams exceeds the limit, we recommend you subscribe each target media stream with the API follows: subscribeStreamVideo{@link #IRTCRoom#subscribeStreamVideo}, subscribeStreamAudio{@link #IRTCRoom#subscribeStreamAudio}.
     *        - After calling this API, you will be informed of the calling result with onVideoSubscribeStateChanged{@link #IRTCRoomEventHandler#onVideoSubscribeStateChanged}, onAudioSubscribeStateChanged{@link #IRTCRoomEventHandler#onAudioSubscribeStateChanged}. You can also find exceptions cause from the callbacks above.
     *        - Once the local user subscribes to the stream of a remote user, the subscription to the remote user will sustain until the local user leaves the room or unsubscribe from it by calling subscribeStreamVideo{@link #IRTCRoom#subscribeStreamVideo}, subscribeStreamAudio{@link #IRTCRoom#subscribeStreamAudio}.
     * @list Audio & Video Transport
     */
    virtual int subscribeAllStreamsAudio(bool subscribe) = 0;

    /**
     * @locale zh
     * @type api
     * @valid since 3.60. 自 3.60 起，该接口替代了 `pauseAllSubscribedStreamAudio` 方法来实现下述功能。如果你已升级至 3.60 及以上版本，且仍在使用该方法，请迁移至该接口。
     * @brief 暂停接收所有远端视频流。
     * @return
     *        - 0: 调用成功。
     *        - < 0 : 调用失败。查看 ReturnStatus{@link #ReturnStatus} 获得更多错误说明。
     * @note
     *        - 该方法仅暂停远端流的接收，并不影响远端流的采集和发送。
     *        - 该方法不改变用户的订阅状态以及订阅流的属性。
     *        - 若想恢复接收远端流，需调用 resumeAllSubscribedStreamVideo{@link #IRTCRoom#resumeAllSubscribedStreamVideo}。
     *        - 多房间场景下，仅暂停接收发布在当前所在房间的流。
     * @list 音视频传输
     */
    /**
     * @locale en
     * @type api
     * @valid since 3.60. Since version 3.60, this interface replaces the `pauseAllSubscribedStreamAudio` for the below functions. If you have upgraded to version 3.60 or later and are still using these methods, please migrate to this interface.
     * @brief Pauses receiving all the video streams of all remote users.
     * @return
     *        - 0: Success.
     *        - < 0 : Fail. See ReturnStatus{@link #ReturnStatus} for more details.
     * @note
     *         - Calling this API does not change the capture state and the transmission state of the remote clients.
     *         - Calling this API does not cancel the subscription or change any subscription configuration.
     *         - To resume, call resumeAllSubscribedStreamVideo{@link #IRTCRoom#resumeAllSubscribedStreamVideo}.
     *         - In a multi-room scenario, this API only pauses the reception of streams published in the current room.
     * @list Audio & Video Transport
     */
    virtual int pauseAllSubscribedStreamVideo() = 0;
    /**
     * @locale zh
     * @type api
     * @valid since 3.60. 自 3.60 起，该接口替代了 `pauseAllSubscribedStream` 方法来实现下述功能。如果你已升级至 3.60 及以上版本，且仍在使用该方法，请迁移至该接口。
     * @brief 暂停接收所有远端音频流。
     * @return
     *        - 0: 调用成功。
     *        - < 0 : 调用失败。查看 ReturnStatus{@link #ReturnStatus} 获得更多错误说明
     * @note
     *        - 该方法仅暂停远端流的接收，并不影响远端流的采集和发送；
     *        - 该方法不改变用户的订阅状态以及订阅流的属性。
     *        - 若想恢复接收远端流，需调用 resumeAllSubscribedStreamVideo{@link #IRTCRoom#resumeAllSubscribedStreamVideo}。
     *        - 多房间场景下，仅暂停接收发布在当前所在房间的流。
     * @list 音视频传输
     */
    /**
     * @locale en
     * @type api
     * @valid since 3.60. Since version 3.60, this interface replaces the `pauseAllSubscribedStream` for the below functions. If you have upgraded to version 3.60 or later and are still using these methods, please migrate to this interface.
     * @brief Pauses receiving the audio streams of all remote users.
     * @return
     *        - 0: Success.
     *        - < 0 : Fail. See ReturnStatus{@link #ReturnStatus} for more details
     * @note
     *         - Calling this API does not change the capture state and the transmission state of the remote clients.
     *         - Calling this API does not cancel the subscription or change any subscription configuration.
     *         - To resume, call resumeAllSubscribedStreamVideo{@link #IRTCRoom#resumeAllSubscribedStreamVideo}.
     *         - In a multi-room scenario, this API only pauses the reception of streams published in the current room.
     * @list Audio & Video Transport
     */
    virtual int pauseAllSubscribedStreamAudio() = 0;

    /**
     * @locale zh
     * @type api
     * @valid since 3.60. 自 3.60 起，该接口替代了 `resumeAllSubscribedStream` 方法来实现下述功能。如果你已升级至 3.60 及以上版本，且仍在使用该方法，请迁移至该接口。
     * @brief 恢复接收所有远端视频流。
     * @return
     *        - 0: 调用成功。
     *        - < 0 : 调用失败。查看 ReturnStatus{@link #ReturnStatus} 获得更多错误说明。
     * @note
     *        - 该方法仅恢复远端流的接收，并不影响远端流的采集和发送。
     *        - 该方法不改变用户的订阅状态以及订阅流的属性。
     * @list 音视频传输
     */
    /**
     * @locale en
     * @type api
     * @valid since 3.60. Since version 3.60, this interface replaces the `resumeAllSubscribedStream` method to implement the below functions. If you have upgraded to version 3.60 or later and are still using this method, please migrate to this interface.
     * @brief Resumes receiving all the video streams of all remote users.
     * @return
     *        - 0: Success.
     *        - < 0 : Fail. See ReturnStatus{@link #ReturnStatus} for more details.
     * @note
     *         - Calling this API does not change the capture state and the transmission state of the remote clients.
     *         - Calling this API does not change any subscription configuration.
     * @list Audio & Video Transport
     */
    virtual int resumeAllSubscribedStreamVideo() = 0;
    /**
     * @locale zh
     * @type api
     * @valid since 3.60. 自 3.60 起，该接口替代了 `resumeAllSubscribedStream` 方法来实现下述功能。如果你已升级至 3.60 及以上版本，且仍在使用该方法，请迁移至该接口。
     * @brief 恢复接收所有远端音频流。
     * @return
     *        - 0: 调用成功。
     *        - < 0 : 调用失败。查看 ReturnStatus{@link #ReturnStatus} 获得更多错误说明
     * @note
     *        - 该方法仅恢复远端流的接收，并不影响远端流的采集和发送。
     *        - 该方法不改变用户的订阅状态以及订阅流的属性。
     * @list 音视频传输
     */
    /**
     * @locale en
     * @type api
     * @valid since 3.60. Since version 3.60, this interface replaces the `resumeAllSubscribedStream` method to implement the below functions. If you have upgraded to version 3.60 or later and are still using this method, please migrate to this interface.
     * @brief Resumes receiving all the audio streams of all remote users.
     * @return
     *        - 0: Success.
     *        - < 0 : Fail. See ReturnStatus{@link #ReturnStatus} for more details
     * @note
     *         - Calling this API does not change the capture state and the transmission state of the remote clients.
     *         - Calling this API does not change any subscription configuration.
     * @list Audio & Video Transport
     */
    virtual int resumeAllSubscribedStreamAudio() = 0;
    /**
    * @locale zh
     * @hidden for internal use only.
     */
    /**
    * @locale en
     * @hidden for internal use only.
     */
    virtual int enableSubscribeLocalStream(bool enable) = 0;
    /**
     * @locale zh
     * @type api
     * @brief 设置发流端音画同步。 <br>
     *        当同一用户同时使用两个通话设备分别采集发送音频和视频时，有可能会因两个设备所处的网络环境不一致而导致发布的流不同步，此时你可以在视频发送端调用该接口，SDK 会根据音频流的时间戳自动校准视频流，以保证接收端听到音频和看到视频在时间上的同步性。
     * @param audio_user_id 音频发送端的用户 ID，将该参数设为空则可解除当前音视频的同步关系。
     * @return
     *        - 0: 调用成功。调用该接口后音画同步状态发生改变时，你会收到 onAVSyncStateChange{@link #IRTCRoomEventHandler#onAVSyncStateChange} 回调。
     *        - < 0 : 调用失败。你也可以通过监听 onAVSyncEvent{@link #IRTCRoomEventHandler#onAVSyncEvent} 获取错误详情。同一 RTC 房间内允许存在多个音视频同步关系，但需注意单个音频源不支持与多个视频源同时同步。
     * @note
     *        - 该方法在进房前后均可调用。
     *        - 进行音画同步的音频发布用户 ID 和视频发布用户 ID 须在同一个 RTC 房间内。
     *        - 如需更换同步音频源，再次调用该接口传入新的 `audio_user_id` 即可；如需更换同步视频源，需先解除当前的同步关系，后在新视频源端开启同步。
     * @list 音视频传输
     */
    /**
     * @locale en
     * @type api
     * @brief Synchronizes published audio and video. <br>
     *        When the same user simultaneously uses separate devices to capture and publish audio and video, there is a possibility that the streams are out of sync due to the network disparity. In this case, you can call this API on the video publisher side and the SDK will automatically line the video stream up according to the timestamp of the audio stream, ensuring that the audio the receiver hears corresponds to the video the receiver watches.
     * @param audio_user_id The ID of audio publisher. You can stop the current A/V synchronization by setting this parameter to null.
     * @return
     *        - 0: Success. When the A/V synchronization state changes, you will receive onAVSyncStateChange{@link #IRTCRoomEventHandler#onAVSyncStateChange}.
     *        - < 0 : Fail. You can also learn the error details via onAVSyncEvent{@link #IRTCRoomEventHandler#onAVSyncEvent}. More than one pair of audio and video can be synchronized simultaneously in the same RTC room, but you should note that one single audio source cannot be synchronized with multiple video sources at the same time.
     * @note
     *        - You can call this API anytime before or after entering the room.
     *        - The source user IDs of the audio and video stream to be synchronized must be in the same RTC room.
     *        - If you want to change the audio source, call this API again with a new `audio_user_id`. If you want to change the video source, you need to stop the current synchronization first, then call this API on the new video publisher side.
     * @list Audio & Video Transport
     */
    virtual int setMultiDeviceAVSync(const char* audio_user_id) = 0;

    /**
     * @locale zh
     * @hidden for internal use only.
     */
    /**
     * @locale en
     * @hidden for internal use only.
     */
    virtual int setCustomUserRole(const char* role) = 0;
    /**
     * @locale zh
     * @type api
     * @brief 开始跨房间转发媒体流。 <br>
     *        在调用 joinRoom{@link #IRTCRoom#joinRoom} 后调用本接口，实现向多个房间转发媒体流，适用于跨房间连麦等场景。
     * @param configuration 跨房间媒体流转发指定房间的信息。参看 ForwardStreamConfiguration{@link #ForwardStreamConfiguration}。
     * @return
     *        - 0： 成功。
     *        - < 0： 失败。查看 ReturnStatus{@link #ReturnStatus} 获得更多错误说明。
     * @note
     *        - 调用本方法后，将在本端触发 onForwardStreamStateChanged{@link #IRTCRoomEventHandler#onForwardStreamStateChanged} 回调。
     *        - 调用本方法后，你可以通过监听 onForwardStreamEvent{@link #IRTCRoomEventHandler#onForwardStreamEvent} 回调来获取各个目标房间在转发媒体流过程中的相关事件。
     *        - 开始转发后，目标房间中的用户将接收到本地用户进房 onUserJoined{@link #IRTCRoomEventHandler#onUserJoined} 和发流 onUserPublishStreamVideo{@link #IRTCRoomEventHandler#onUserPublishStreamVideo}、onUserPublishStreamAudio{@link #IRTCRoomEventHandler#onUserPublishStreamAudio} 回调。
     *        - 调用本方法后，可以调用 updateForwardStreamToRooms{@link #IRTCRoom#updateForwardStreamToRooms} 更新目标房间信息，例如，增加或减少目标房间等。
     *        - 调用本方法后，可以调用 stopForwardStreamToRooms{@link #IRTCRoom#stopForwardStreamToRooms} 停止向所有房间转发媒体流。
     *        - 调用本方法后，可以调用 pauseForwardStreamToAllRooms{@link #IRTCRoom#pauseForwardStreamToAllRooms} 暂停向所有房间转发媒体流。
     * @list 音视频传输
     */
    /**
     * @locale en
     * @type api
     * @brief Start relaying media stream across rooms. <br>
     *        After calling joinRoom{@link #IRTCRoom#joinRoom}, you can call this method to publish the media stream to multiple rooms that applies to scenarios such as anOnline talent contest and so on.
     * @param configuration Information of the rooms where you want to relay the media stream to. Refer to ForwardStreamConfiguration{@link #ForwardStreamConfiguration} for more information.
     * @return
     *        - 0: Success <br>
     *        - < 0: Failure. See ReturnStatus{@link #ReturnStatus} for more details.
     * @note
     *        - Call this method will trigger onForwardStreamStateChanged{@link #IRTCRoomEventHandler#onForwardStreamStateChanged}.
     *        - After calling this method, listen the events from each room during the relaying by registering onForwardStreamEvent{@link #IRTCRoomEventHandler#onForwardStreamEvent}.
     *        - Once the relaying begins, the other users in the room will receive callback of onUserJoined{@link #IRTCRoomEventHandler#onUserJoined}, onUserPublishStreamVideo{@link #IRTCRoomEventHandler#onUserPublishStreamVideo}, onUserPublishStreamAudio{@link #IRTCRoomEventHandler#onUserPublishStreamAudio}.
     *        - Call updateForwardStreamToRooms{@link #IRTCRoom#updateForwardStreamToRooms} to add or remove the target room(s) after calling this method.
     *        - Call stopForwardStreamToRooms{@link #IRTCRoom#stopForwardStreamToRooms} to stop relaying to all rooms after calling this method.
     *        - Call pauseForwardStreamToAllRooms{@link #IRTCRoom#pauseForwardStreamToAllRooms} to pause relaying to all rooms after calling this method.
     * @list Audio & Video Transport
     */
    virtual int startForwardStreamToRooms(const ForwardStreamConfiguration& configuration) = 0;
    /**
     * @locale zh
     * @type api
     * @brief 更新跨房间媒体流转发信息。 <br>
     *        通过 startForwardStreamToRooms{@link #IRTCRoom#startForwardStreamToRooms} 发起媒体流转发后，可调用本方法增加或者减少目标房间，或更新房间密钥。 <br>
     *        调用本方法增加或删减房间后，将在本端触发 onForwardStreamStateChanged{@link #IRTCRoomEventHandler#onForwardStreamStateChanged} 回调，包含发生了变动的目标房间中媒体流转发状态。
     * @param configuration 跨房间媒体流转发目标房间信息。参看 ForwardStreamConfiguration{@link #ForwardStreamConfiguration}。
     * @return
     *        - 0： 成功。
     *        - < 0： 失败。查看 ReturnStatus{@link #ReturnStatus} 获得更多错误说明。
     * @note
     *        增加或删减目标房间后，新增目标房间中的用户将接收到本地用户进房 onUserJoined{@link #IRTCRoomEventHandler#onUserJoined} 和发布 onUserPublishStreamVideo{@link #IRTCRoomEventHandler#onUserPublishStreamVideo}、onUserPublishStreamAudio{@link #IRTCRoomEventHandler#onUserPublishStreamAudio} 回调。
     * @list 音视频传输
     */
    /**
     * @locale en
     * @type api
     * @brief Update information of the rooms where you want to relay the media stream to after calling startForwardStreamToRooms{@link #IRTCRoom#startForwardStreamToRooms}. <br>
     *        Adding and removing rooms by calling this method will trigger onForwardStreamStateChanged{@link #IRTCRoomEventHandler#onForwardStreamStateChanged} on the local.
     * @param configuration Information of the rooms where you want to relay the media stream to. Refer to ForwardStreamConfiguration{@link #ForwardStreamConfiguration} for more information.
     * @return
     *         - 0: Success.
     *         - < 0: Failure. See ReturnStatus{@link #ReturnStatus} for more details.
     * @note
     *        Users in the room which is added or removed from the target list by calling this method will receive onUserJoined{@link #IRTCRoomEventHandler#onUserJoined}, onUserPublishStreamVideo{@link #IRTCRoomEventHandler#onUserPublishStreamVideo}, onUserPublishStreamAudio{@link #IRTCRoomEventHandler#onUserPublishStreamAudio}.
     */
    virtual int updateForwardStreamToRooms(const ForwardStreamConfiguration& configuration) = 0;
    /**
     * @locale zh
     * @type api
     * @brief 停止跨房间媒体流转发。 <br>
     *        通过 startForwardStreamToRooms{@link #IRTCRoom#startForwardStreamToRooms} 发起媒体流转发后，可调用本方法停止向所有目标房间转发媒体流。
     * @return
     *        - 0: 调用成功。
     *        - < 0 : 调用失败。查看 ReturnStatus{@link #ReturnStatus} 获得更多错误说明。
     * @note
     *        - 调用本方法后，将在本端触发 onForwardStreamStateChanged{@link #IRTCRoomEventHandler#onForwardStreamStateChanged} 回调。
     *        - 调用本方法后，原目标房间中的用户将接收到本地用户停止发布 onUserPublishStreamVideo{@link #IRTCRoomEventHandler#onUserPublishStreamVideo}、onUserPublishStreamAudio{@link #IRTCRoomEventHandler#onUserPublishStreamAudio} 回调和退房 onUserLeave{@link #IRTSRoomEventHandler#onUserLeave} 的回调。
     *        - 如果需要停止向指定的房间转发媒体流，请调用 updateForwardStreamToRooms{@link #IRTCRoom#updateForwardStreamToRooms} 更新房间信息。
     *        - 如果需要暂停转发，请调用 pauseForwardStreamToAllRooms{@link #IRTCRoom#pauseForwardStreamToAllRooms}，并在之后随时调用 resumeForwardStreamToAllRooms{@link #IRTCRoom#resumeForwardStreamToAllRooms} 快速恢复转发。
     * @list 音视频传输
     */
    /**
     * @locale en
     * @type api
     * @brief Call to this method to stop relaying media stream to all rooms after calling startForwardStreamToRooms{@link #IRTCRoom#startForwardStreamToRooms}.
     * @return
     *        - 0: Success.
     *        - < 0 : Fail. See ReturnStatus{@link #ReturnStatus} for more details.
     * @note
     *        - Call this method will trigger onForwardStreamStateChanged{@link #IRTCRoomEventHandler#onForwardStreamStateChanged}.
     *        - The other users in the room will receive callback of onUserJoined{@link #IRTCRoomEventHandler#onUserJoined}, onUserPublishStreamVideo{@link #IRTCRoomEventHandler#onUserPublishStreamVideo}, onUserPublishStreamAudio{@link #IRTCRoomEventHandler#onUserPublishStreamAudio} when you stop relaying.
     *        - To stop relaying media stream to specific rooms, call updateForwardStreamToRooms{@link #IRTCRoom#updateForwardStreamToRooms} instead.
     *        - To resume the relaying in a short time, call pauseForwardStreamToAllRooms{@link #IRTCRoom#pauseForwardStreamToAllRooms} instead and then call resumeForwardStreamToAllRooms{@link #IRTCRoom#resumeForwardStreamToAllRooms} to recsume after that.
     * @list Audio & Video Transport
     */
    virtual int stopForwardStreamToRooms() = 0;
    /**
     * @locale zh
     * @type api
     * @brief 暂停跨房间媒体流转发。 <br>
     *        通过 startForwardStreamToRooms{@link #IRTCRoom#startForwardStreamToRooms} 发起媒体流转发后，可调用本方法暂停向所有目标房间转发媒体流。 <br>
     *        调用本方法暂停向所有目标房间转发后，你可以随时调用 resumeForwardStreamToAllRooms{@link #IRTCRoom#resumeForwardStreamToAllRooms} 快速恢复转发。
     * @return
     *        - 0: 调用成功。
     *        - < 0 : 调用失败。查看 ReturnStatus{@link #ReturnStatus} 获得更多错误说明。
     * @note 调用本方法后，目标房间中的用户将接收到本地用户停止发布 onUserPublishStreamVideo{@link #IRTCRoomEventHandler#onUserPublishStreamVideo}、onUserPublishStreamAudio{@link #IRTCRoomEventHandler#onUserPublishStreamAudio} 回调和退房 onUserLeave{@link #IRTSRoomEventHandler#onUserLeave} 的回调。
     * @list Audio & Video Transport
     */
     /**
     * @locale en
     * @type api
     * @region Multi-room
     * @brief Call this method to pause relaying media stream to all rooms after calling startForwardStreamToRooms{@link #IRTCRoom#startForwardStreamToRooms}. <br>
     *        After that, call resumeForwardStreamToAllRooms{@link #IRTCRoom#resumeForwardStreamToAllRooms} to resume.
     * @return
     *        - 0: Success.
     *        - < 0 : Fail. See ReturnStatus{@link #ReturnStatus} for more details.
     * @note The other users in the room will receive callback of onUserPublishStreamVideo{@link #IRTCRoomEventHandler#onUserPublishStreamVideo}, onUserPublishStreamAudio{@link #IRTCRoomEventHandler#onUserPublishStreamAudio} when you pause relaying.
     */
    virtual int pauseForwardStreamToAllRooms() = 0;
    /**
     * @locale zh
     * @type api
     * @brief 恢复跨房间媒体流转发。 <br>
     *        调用 pauseForwardStreamToAllRooms{@link #IRTCRoom#pauseForwardStreamToAllRooms} 暂停转发之后，调用本方法恢复向所有目标房间转发媒体流。
     * @return
     *        - 0: 调用成功。
     *        - < 0 : 调用失败。查看 ReturnStatus{@link #ReturnStatus} 获得更多错误说明。
     * @note
     *        目标房间中的用户将接收到本地用户进房 onUserJoined{@link #IRTCRoomEventHandler#onUserJoined} 和发布 onUserPublishStreamVideo{@link #IRTCRoomEventHandler#onUserPublishStreamVideo}、onUserPublishStreamAudio{@link #IRTCRoomEventHandler#onUserPublishStreamAudio} 回调。
     * @list 音视频传输
     */
    /**
     * @locale en
     * @type api
     * @brief Call this method to resume relaying to all rooms from the pause by calling pauseForwardStreamToAllRooms{@link #IRTCRoom#pauseForwardStreamToAllRooms}.
     * @return
     *        - 0: Success.
     *        - < 0 : Fail. See ReturnStatus{@link #ReturnStatus} for more details.
     * @note The other users in the room will receive callback of onUserJoined{@link #IRTCRoomEventHandler#onUserJoined}, onUserPublishStreamVideo{@link #IRTCRoomEventHandler#onUserPublishStreamVideo}, onUserPublishStreamAudio{@link #IRTCRoomEventHandler#onUserPublishStreamAudio} when you resume relaying.
     */
    virtual int resumeForwardStreamToAllRooms() = 0;
    /**
     * @locale zh
     * @hidden(Linux)
     * @type api
     * @brief 获取范围语音接口实例。
     * @return 方法调用结果： <br>
     *        - IRangeAudio：成功，返回一个 IRangeAudio{@link #IRangeAudio} 实例。
     *        - nullptr：失败，当前 SDK 不支持范围语音功能。
     * @note 首次调用该方法须在创建房间后、加入房间前。范围语音相关 API 和调用时序详见[范围语音](https://www.volcengine.com/docs/6348/114727)。
     * @list 高级功能
     */
    /**
     * @locale en
     * @hidden(Linux)
     * @type api
     * @brief Gets range audio instance.
     * @return API call result: <br>
     *        - IRangeAudio: Success. You will get an IRangeAudio{@link #IRangeAudio} returned from the SDK.
     *        - nullptr: Failure. The current SDK does not offer range audio function.
     * @note The first time this API is called must be between you create a room and you actually enter the room. Refer to [Range Voice](https://docs.byteplus.com/byteplus-rtc/docs/114727) for more information.
     * @list Advanced Features
     */
    virtual IRangeAudio* getRangeAudio() = 0;
    /**
     * @locale zh
     * @hidden(Linux)
     * @type api
     * @brief 获取空间音频接口实例。
     * @return 方法调用结果： <br>
     *        - ISpatialAudio：成功，返回一个 ISpatialAudio{@link #ISpatialAudio} 实例。
     *        - nullptr：失败，当前 SDK 不支持空间音频功能。
     * @note
     *        - 首次调用该方法须在创建房间后、加入房间前。空间音频相关 API 和调用时序详见[空间音频](https://www.volcengine.com/docs/6348/93903)。
     *        - 只有在使用支持真双声道播放的设备时，才能开启空间音频效果；
     *        - 机型性能不足可能会导致音频卡顿，使用低端机时，不建议开启空间音频效果；
     *        - SDK 最多支持 30 个用户同时开启空间音频功能。
     * @list 高级功能
     */
    /**
     * @locale en
     * @hidden(Linux)
     * @type api
     * @brief Gets spatial audio instance.
     * @return API call result: <br>
     *        - ISpatialAudio: Success. You will get an ISpatialAudio{@link #ISpatialAudio} returned from the SDK.
     *        - nullptr: Failure. The current SDK does not offer spatial audio function.
     * @note
     *        - The first time this API is called must be between you create a room and you actually enter the room.  Refer to [Spatial Audio](https://docs.byteplus.com/byteplus-rtc/docs/93903) for more information.
     *        - Only using real dual-channel playback device can you actually enjoy spatial audio effect.
     *        - Low-end device may have audio stalling issues due to its poor performance, so it is not recommended to enable spatial audio function on such kind of devices.
     *        - SDK currently supports up to 30 users publishing audio with spatial audio enabled at the same time in the same room.
     * @list Advanced Features
     */
    virtual ISpatialAudio* getSpatialAudio() = 0;
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
    /**
     * @locale zh
     * @hidden for internal use only on Windows and Android
     * @type api
     * @brief 接收端调用本接口获取全景视频接口引擎。
     * @return 方法调用结果： <br>
     *        - IPanoramicVideo：成功，返回一个 IPanoramicVideo{@link #IPanoramicVideo} 实例。
     *        - null：失败，当前 SDK 不支持全景视频功能。
     * @note
     *        - 接收端的调用时序为 createRTCEngine{@link #IRTCEngine#createRTCEngine} >  createRTCRoom{@link #IRTCEngine#createRTCRoom} > 本接口 > joinRoom{@link #IRTCRoom#joinRoom}。
     *        - 获取 IPanoramicVideo 实例后，再调用 updateQuaternionf{@link #IPanoramicVideo#updateQuaternionf} 更新头位姿四元数。
     *        - 你需要使用外部渲染器对接收到的全景视频进行渲染，参见 setRemoteVideoSink{@link #IRTCEngine#setRemoteVideoSink}。
     * @list 视频管理
     */
    /**
     * @locale en
     * @hidden for internal use only on Windows and Android
     * @type api
     * @brief Gets panoramic video engine.
     * @return API call result: <br>
     *        - IPanoramicVideo: Success. You will get an IPanoramicVideo{@link #IPanoramicVideo} returned from the SDK.
     *        - null: Failure. The current SDK does not offer range audio function.
     * @note
     *        - The API call sequence on the subscriber side is createRTCEngine{@link #IRTCEngine#createRTCEngine} > createRTCRoom{@link #IRTCEngine#createRTCRoom} > this API > joinRoom{@link #IRTCRoom#joinRoom}.
     *        - After calling this API, you can call updateQuaternionf{@link #IPanoramicVideo#updateQuaternionf} to update the position of the head represented as a quaternion.
     *        - Refer to setRemoteVideoSink{@link #IRTCEngine#setRemoteVideoSink} for details on how to render the panoramic video with an external renderer.
     * @list Video Management
     */
    virtual IPanoramicVideo* getPanoramicVideo() = 0;
    /**
     * @locale zh
     * @valid since 3.52.
     * @type api
     * @brief 设置本端发布流在音频选路中的优先级。
     * @param audio_selection_priority 本端发布流在音频选路中的优先级，默认正常参与音频选路。参见 AudioSelectionPriority{@link #AudioSelectionPriority}。
     * @note
     *       - 在控制台上为本 appId 开启音频选路后，调用本接口才会生效。进房前后调用均可生效。更多信息参见[音频选路](https://www.volcengine.com/docs/6348/113547)。
     *       - 如果本端用户同时加入不同房间，使用本接口进行的设置相互独立。
     * @list 音频管理
     */
    /**
     * @locale en
     * @valid since 3.52.
     * @type api
     * @brief Set the priority of the local audio stream to be published.
     * @param audio_selection_priority The priority of the local audio stream which defaults to be subscribable only up to the result of the Audio Selection. Refer to AudioSelectionPriority{@link #AudioSelectionPriority}.
     * @note
     *       - You must enable Audio Selection in the RTC console before using this API. You can call this API whether the user has joined a room. Refer to [Audio Selection](https://docs.byteplus.com/byteplus-rtc/docs/113547).
     *       - The setting is independent in each room that the user joins.
     * @list Audio Management
     */
    virtual int setAudioSelectionConfig(AudioSelectionPriority audio_selection_priority) = 0;
    /**
     * @locale zh
     * @valid since 3.52.
     * @type api
     * @brief 设置/更新房间附加信息，可用于标识房间状态或属性，或灵活实现各种业务逻辑。
     * @param key 房间附加信息键值，长度小于 10 字节。 <br>
     *        同一房间内最多可存在 5 个 key，超出则会从第一个 key 起进行替换。
     * @param value 房间附加信息内容，长度小于 128 字节。
     * @return
     *        - 0: 方法调用成功，返回本次调用的任务编号；
     *        - <0: 方法调用失败，具体原因详见 SetRoomExtraInfoResult{@link #SetRoomExtraInfoResult}。
     * @note
     *       - 在设置房间附加信息前，必须先调用 joinRoom{@link #IRTCRoom#joinRoom} 加入房间。
     *       - 调用该方法后，会收到一次 onSetRoomExtraInfoResult{@link #IRTCRoomEventHandler#onSetRoomExtraInfoResult} 回调，提示设置结果。
     *       - 调用该方法成功设置附加信息后，同一房间内的其他用户会收到关于该信息的回调 onRoomExtraInfoUpdate{@link #IRTCRoomEventHandler#onRoomExtraInfoUpdate}。
     *       - 新进房的用户会收到进房前房间内已有的全部附加信息通知。
     * @list 房间管理
     */
    /**
     * @locale en
     * @valid since 3.52.
     * @type api
     * @brief Sets extra information about the room the local user joins.
     * @param key Key of the extra information, less than 10 bytes in length. <br>
     *        A maximum of 5 keys can exist in the same room, beyond which the first key will be replaced.
     * @param value Content of the extra information, less than 128 bytes in length.
     * @return API call result: <br>
     *        - 0: Success with a taskId returned.
     *        - <0: Failure. See SetRoomExtraInfoResult{@link #SetRoomExtraInfoResult} for the reasons.
     * @note
     *        - Call joinRoom{@link #IRTCRoom#joinRoom} first before you call this API to set extra information.
     *        - After calling this API, you will receive onSetRoomExtraInfoResult{@link #IRTCRoomEventHandler#onSetRoomExtraInfoResult} callback informing you the result of the setting.
     *        - After the extra information is successfully set, other users in the same room will receive the information through onRoomExtraInfoUpdate{@link #IRTCRoomEventHandler#onRoomExtraInfoUpdate} callback.
     *        - Users who join the room later will be notified of all extra information in the room set prior to entering.
     * @list Room Management
     */
    virtual int64_t setRoomExtraInfo(const char*key,const char*value) = 0;
    /**
     * @locale zh
     * @valid since 3.52
     * @type api
     * @brief 识别或翻译房间内所有用户的语音，形成字幕。 <br>
     *        调用该方法时，可以在 SubtitleMode{@link #SubtitleMode} 中选择语音识别或翻译模式。如果选择识别模式，语音识别文本会通过 onSubtitleMessageReceived{@link #IRTCRoomEventHandler#onSubtitleMessageReceived} 事件回调给你； <br>
     *        如果选择翻译模式，你会同时收到两个 onSubtitleMessageReceived{@link #IRTCRoomEventHandler#onSubtitleMessageReceived} 回调，分别包含字幕原文及字幕译文。 <br>
     *        调用该方法后，用户会收到 onSubtitleStateChanged{@link #IRTCRoomEventHandler#onSubtitleStateChanged} 回调，通知字幕是否开启。
     * @param subtitle_config 字幕配置信息。参看 SubtitleConfig{@link #SubtitleConfig}。
     * @return
     *        -  0: 调用成功。
     *        - !0: 调用失败。
     * @note
     *        - 使用字幕功能前，你需要在 [RTC 控制台](https://console.volcengine.com/rtc/cloudRTC?tab=subtitle?from=doc) 开启实时字幕功能。
     *        - 如果你需要使用流式语音识别模式，你应在 [语音技术控制台](https://console.volcengine.com/speech/service/16?from=doc) 创建流式语音识别应用。创建时，服务类型应选择 `流式语音识别`，而非 `音视频字幕生成`。创建后，在 [RTC 控制台](https://console.volcengine.com/rtc/cloudRTC?tab=subtitle?from=doc) 上启动流式语音识别，并填写创建语音技术应用时获取的相关信息，包括：APP ID，Access Token，和 Cluster ID。
     *        - 如果你需要使用实时语音翻译模式，你应开通机器翻译服务，参考 [开通服务](https://www.volcengine.com/docs/4640/130262)。完成开通后，在 [RTC 控制台](https://console.volcengine.com/rtc/cloudRTC?tab=subtitle?from=doc) 上启用实时语音翻译模式。
     *        - 此方法需要在进房后调用。
     *        - 如需指定源语言，你需要在调用 `joinRoom` 接口进房时，通过 extraInfo 参数传入格式为`"语种英文名": "语种代号"` JSON 字符串，例如设置源语言为英文时，传入 `"source_language": "en"`。如未指定源语言，SDK 会将系统语种设定为源语言。如果你的系统语种不是中文、英文和日文，此时 SDK 会自动将中文设为源语言。
     *          - 识别模式下，你可以传入 [RTC 控制台](https://console.volcengine.com/rtc/cloudRTC?tab=subtitle?from=doc)上预设或自定义的语种英文名和语种代号。识别模式下支持的语言参看[识别模式语种支持](https://www.volcengine.com/docs/6561/109880#场景-语种支持)。
     *          - 翻译模式下，你需要传入机器翻译规定的语种英文名和语种代号。翻译模式下支持的语言及对应的代号参看[翻译模式语言支持](https://www.volcengine.com/docs/4640/35107)。
     * @list 字幕翻译服务
     */
    /**
     * @locale en
     * @hidden currently not available
     * @type api
     * @brief Recognizes or translates the speech of all speakers in the room and converts the results into captions. <br>
     *        When calling this method, you can choose the subtitle mode in the SubtitleMode{@link #SubtitleMode}. If you choose the recognition mode, you will receive the onSubtitleMessageReceived{@link #IRTCRoomEventHandler#onSubtitleMessageReceived} callback which contains the transcribed text. <br>
     *        If you choose the translation mode, you will receive two onSubtitleMessageReceived{@link #IRTCRoomEventHandler#onSubtitleMessageReceived} callbacks simultaneously, one contains the transcribed text and the other contains the translated text. <br>
     *        After calling this method, you will receive the onSubtitleStateChanged{@link #IRTCRoomEventHandler#onSubtitleStateChanged} to inform you of whether subtitles are on.
     * @param subtitle_config Subtitle configurations. Refer to SubtitleConfig{@link #SubtitleConfig} for details.
     * @return
     *         - 0: Success.
     *         - < 0: Failure. See ReturnStatus{@link #ReturnStatus} for more details.
     * @note
     *        - Call this method after joining the room.
     *        - You can set your source language to Chinese by calling `joinRoom`  and importing a json formatted string `"source_language": "zh"` through the parameter of extraInfo, to English by importing `"source_language": "en"` , and to Japanese by importing `"source_language": "ja"` . If you don't set the source language, SDK will set the language of your system as the source language. If the language of your system is not Chinese, English or Japanese, SDK will set Chinese as the source language.
     * @list Subtitles
     */
    virtual int startSubtitle(const SubtitleConfig& subtitle_config) = 0;
    /**
     * @locale zh
     * @valid since 3.52
     * @type api
     * @brief 关闭字幕。 <br>
     *        调用该方法后，用户会收到 onSubtitleStateChanged{@link #IRTCRoomEventHandler#onSubtitleStateChanged} 回调，通知字幕是否关闭。
     * @return
     *        -  0: 调用成功。
     *        - !0: 调用失败。
     * @list 字幕翻译服务
     */
    /**
     * @locale en
     * @hidden currently not available
     * @type api
     * @brief Turns off subtitles. <br>
     *        After calling this method, you will receive the onSubtitleStateChanged{@link #IRTCRoomEventHandler#onSubtitleStateChanged} to inform you of whether subtitles are off.
     * @return
     *         - 0: Success.
     *         - < 0: Failure. See ReturnStatus{@link #ReturnStatus} for more details.
     * @list Subtitles
     */
    virtual int stopSubtitle() = 0;
    /**
     * @locale zh
     * @valid since 3.53
     * @type api
     * @brief 获取房间 ID。
     * @return 房间 ID。
     * @list Subtitles
     */
    /**
     * @locale en
     * @valid since 3.53
     * @type api
     * @brief Get room ID.
     * @return Room ID.
     * @list Subtitles
     */
    virtual const char* getRoomId() = 0;

    /**
     * @locale zh
     * @type api
     * @valid since 3.60.
     * @brief 获取通话 ID。
     *        该方法需要在加入房间后调用。当创建一个房间开启音视频通话后，系统会为该房间生成一个对应的通话 ID，标识此房间的通话。
     * @return 通话 ID。
     * @list 房间管理
     */
    /**
     * @locale en
     * @type api
     * @valid since 3.60.
     * @brief Gets call ID. 
     *        Call this method after joining a room. After creating a room, a call ID is generated to identify the call for this room.
     * @return Call ID.
     * @list Room Management
     */
    virtual const char* getCallId() = 0;
};

} // namespace bytertc
