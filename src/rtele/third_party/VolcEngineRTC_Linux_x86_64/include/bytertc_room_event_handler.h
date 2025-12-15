/*
 * Copyright (c) 2020 The VolcEngineRTC project authors. All Rights Reserved.
 * @brief VolcEngineRTC Room Event Handler Interface
*/

#pragma once

#include "rtc/bytertc_defines.h"
#include "bytertc_rts_room_event_handler.h"

namespace bytertc {
/**
 * @locale zh
 * @type callback
 * @brief RTS房间事件回调接口 <br>
 * 注意：回调函数是在 SDK 内部线程（非 UI 线程）同步抛出来的，请不要做耗时操作或直接操作 UI，否则可能导致 app 崩溃。
 * @list 
 */
/**
 * @locale en
 * @type callback
 * @brief  RTS room event callback interface <br>
 * Note: Callback functions are thrown synchronously in a non-UI thread within the SDK. Therefore, you must not perform any time-consuming operations or direct UI operations within the callback function, as this may cause the app to crash.
 * @list 
 */
class IRTCRoomEventHandler : public IRTSRoomEventHandler {
public:
    /**
     * @locale zh
     * @hidden constructor/destructor
     */
    /**
     * @locale en
     * @hidden constructor/destructor
     */
    virtual ~IRTCRoomEventHandler() {
    }
    /**
     * @locale zh
     * @hidden
     * @type callback
     * @brief 房间状态改变回调，加入房间、异常退出房间、发生房间相关的警告或错误时会收到此回调。
     * @param room_id 房间 ID。
     * @param uid 用户 ID。
     * @param state 房间状态码。 <br>
     *              - 0: 加入房间成功。
     *              - 1: 加入房间失败、异常退房、发生房间相关的警告或错误。
     *              - 2: 离开房间。
     * @param reason 房间状态发生变化的原因。参见 RoomStateChangeReason{@link #RoomStateChangeReason}。
     * @list 房间管理
     */
    /**
     * @locale en
     * @hidden
     * @type callback
     * @brief Callback on room state changes. Via this callback you get notified of room relating warnings,  errors and events. For example, the user joins the room, the user is removed from the room, and so on.
     * @param room_id  Room ID.
     * @param uid  User ID.
     * @param state Room state code. <br>
     *              - 0: Join room success.
     *              - 1: Failed to join a room, abnormal exit, room-related warnings or errors.
     *              - 2: Leave room.
     * @param reason The reason why room state changes. See RoomStateChangeReason{@link #RoomStateChangeReason}.
     */
    virtual void onRoomStateChangedWithReason(
            const char* room_id, const char* uid, RoomState state, RoomStateChangeReason reason) {
        (void)room_id;
        (void)uid;
        (void)state;
        (void)reason;
    }
    /**
     * @locale zh
     * @type callback
     * @valid since 3.60.
     * @region 房间管理
     * @brief 视频发布状态改变回调。
     * @param stream_id 流 ID。
     * @param stream_info 流信息。
     * @param state 发布状态码，参看 PublishState{@link #PublishState}。
     * @param reason 本端视频流发布状态改变的具体原因，参看 PublishStateChangeReason{@link #PublishStateChangeReason}。
     * @note
     *       除了状态改变时触发，本端用户进房时也会收到本回调。
     * @list 音视频传输
     * @order 0
     */
    /**
     * @locale en
     * @type callback
     * @valid since 3.60.
     * @brief Callback on video-stream state changes.
     * @param stream_id Stream ID.
     * @param stream_info Stream information.
     * @param state State code. See PublishState{@link #PublishState} for more.
     * @param reason See PublishStateChangeReason{@link #PublishStateChangeReason} for more.
     * @note
     *       Besides, you will also receive this callback when the local user joins the room.
     */
    virtual void onVideoPublishStateChanged(
            const char* stream_id, const StreamInfo& stream_info, PublishState state, PublishStateChangeReason reason) {
        (void)stream_id;
        (void)stream_info;
        (void)state;
        (void)reason;
    }

    /**
     * @locale zh
     * @type callback
     * @valid since 3.60.
     * @region 房间管理
     * @brief 音频发布状态改变回调。
     * @param stream_id 流 ID。
     * @param stream_info 流信息。
     * @param state 发布状态码，参看 PublishState{@link #PublishState}。
     * @param reason 本端音频流发布状态改变的具体原因，参看 PublishStateChangeReason{@link #PublishStateChangeReason}。
     * @note
     *       除了状态改变时触发，本端用户进房时也会收到本回调。
     * @list 音视频传输
     * @order 0
     */
    /**
     * @locale en
     * @type callback
     * @valid since 3.60.
     * @brief Callback on audio-stream state changes.
     * @param stream_id Stream ID.
     * @param stream_info Stream information.
     * @param state State code. See PublishState{@link #PublishState} for more.
     * @param reason See PublishStateChangeReason{@link #PublishStateChangeReason} for more.
     * @note
     *       Besides, you will also receive this callback when the local user joins the room.
     */
    virtual void onAudioPublishStateChanged(
            const char* stream_id, const StreamInfo& stream_info, PublishState state, PublishStateChangeReason reason) {
        (void)stream_id;
        (void)stream_info;
        (void)state;
        (void)reason;
    }

    /**
     * @locale zh
     * @type callback
     * @valid since 3.60.
     * @region 房间管理
     * @brief 视频订阅状态发生改变回调。
     * @param stream_id 流 ID。
     * @param stream_info 流信息。
     * @param state 订阅状态码，参看 SubscribeState{@link #SubscribeState}。
     * @param reason 视频订阅状态改变的具体原因，参看 SubscribeStateChangeReason{@link #SubscribeStateChangeReason}。
     * @list 音视频传输
     * @order 0
     */
    /**
     * @locale en
     * @type callback
     * @valid since 3.60.
     * @brief Callback on state changes of the subscription to camera-video streams.
     * @param stream_id Stream ID.
     * @param stream_info Stream information.
     * @param state Room state code. See SubscribeState{@link #SubscribeState} for more.
     * @param reason See SubscribeStateChangeReason{@link #SubscribeStateChangeReason} for more.
     */
    virtual void onVideoSubscribeStateChanged(
            const char* stream_id, const StreamInfo& stream_info, SubscribeState state, SubscribeStateChangeReason reason) {
        (void)stream_id;
        (void)stream_info;
        (void)state;
        (void)reason;
    }

    /**
     * @locale zh
     * @type callback
     * @valid since 3.60.
     * @region 房间管理
     * @brief 音频订阅状态发生改变回调。
     * @param stream_id 流 ID。
     * @param stream_info 流信息。
     * @param state 订阅状态码，参看 SubscribeState{@link #SubscribeState}。
     * @param reason 音频订阅状态改变的具体原因，参看 SubscribeStateChangeReason{@link #SubscribeStateChangeReason}。
     * @list 音视频传输
     * @order 0
     */
    /**
     * @locale en
     * @type callback
     * @valid since 3.60.
     * @region Multi-room
     * @brief Callback on state changes of the subscription to microphone-audio streams.
     * @param stream_id  Stream ID.
     * @param stream_info  Stream information.
     * @param state Room state code. See SubscribeState{@link #SubscribeState} for more.
     * @param reason The reason why room state changes. See SubscribeStateChangeReason{@link #SubscribeStateChangeReason} for more.
     */
    virtual void onAudioSubscribeStateChanged(
            const char* stream_id, const StreamInfo& stream_info, SubscribeState state, SubscribeStateChangeReason reason) {
        (void)stream_id;
        (void)stream_info;
        (void)state;
        (void)reason;
    }

    /**
     * @locale zh
     * @hidden
     * @deprecated since 3.41 and will be deleted in 3.51, use onRoomStateChanged{@link #IRTSRoomEventHandler#onRoomStateChanged} and onStreamStateChanged instead.
     * @type callback
     * @brief SDK 发生警告回调。 <br>
     *        SDK 内部遇到可恢复错误时，在尝试自动恢复的同时，会通过此回调通知用户。此回调事件仅用作通知。
     * @param warn 警告码，参看 WarningCode{@link #WarningCode}。
     * @list 房间管理
     */
    /**
     * @locale en
     * @hidden
     * @deprecated since 3.45 and will be deleted in 3.51, use onRoomStateChanged{@link #IRTSRoomEventHandler#onRoomStateChanged} and onStreamStateChanged instead.
     * @type callback
     * @brief SDK Warning callback occurred. When a recoverable error is encountered inside the <br>
     *        SDK, the user is notified via this callback while attempting automatic recovery. This callback event is only used as a notification.
     * @param warn Code. See WarningCode{@link #WarningCode}.
     * @list Room Management
     */
    BYTERTC_DEPRECATED virtual void onRoomWarning(int warn) {
        (void)warn;
    }
    /**
     * @locale zh
     * @type callback
     * @brief 当 SDK 检测到 Token 的进房权限将在 30 秒内过期时，触发该回调。
     *        收到该回调后，你需调用 updateToken{@link #IRTSRoom#updateToken} 更新 Token 进房权限。
     * @note 若未能及时更新 Token 进房权限导致其过期实效： <br>
     *        - 用户此时尝试进房会收到  回调，提示错误码为 `-1000` Token 过期；
     *        - 用户已在房间内则会被移出房间，本地用户会收到  回调，提示错误码为 `-1009` Token 过期，同时远端用户会收到 onUserLeave{@link #IRTSRoomEventHandler#onUserLeave} 回调，提示原因为 `2` Token 进房权限过期。
     * @list 引擎管理
     */
     /**
      * @locale en
      * @type callback
      * @brief The callback is triggered when the SDK detects the joining room privilege from the Token will expires within 30s.
      *        After receiving this callback, you must call updateToken{@link #IRTSRoom#updateToken} to update the joining room privilege Token.
      * @note After a user's joining room privilege expires: <br>
      *        - When attempting to join a room, the user will receive  with the error code "-1000" indicating that the Token is invalid.
      *        - If the user is already in the room, he/she will be removed from the room and receive  with the error code "-1009" indicating that the Token is expired. Remote users in the room will receive onUserLeave{@link #IRTSRoomEventHandler#onUserLeave} with the reason "2" indicating that the Token is invalid.
      * @list Engine Management
      */
    virtual void onTokenWillExpire() {

    }
    /**
     * @locale zh
     * @type callback
     * @brief Token 发布权限过期前 30 秒将触发该回调。 <br>
     *        收到该回调后，你需调用 updateToken{@link #IRTSRoom#updateToken} 更新 Token 发布权限。
     * @note  Token 发布权限过期后：
     *        - 已发布流或尝试发布流时，本端会收到 onVideoPublishStateChanged{@link #IRTCRoomEventHandler#onVideoPublishStateChanged}、onAudioPublishStateChanged{@link #IRTCRoomEventHandler#onAudioPublishStateChanged} 回调，提示`kPublishStateChangeReasonNoPublishPermission`，没有发布权限。
     *        - 发布中的流将停止发布。远端用户会收到 onUserPublishStreamVideo{@link #IRTCRoomEventHandler#onUserPublishStreamVideo}、onUserPublishStreamAudio{@link #IRTCRoomEventHandler#onUserPublishStreamAudio} 回调，提示该流已停止发布。
     * @list 引擎管理
     */
    /**
     * @locale en
     * @type callback
     * @brief Callback triggered 30s before the publishing privilege of the Token expires. <br>
     *        After receiving this callback, you must call updateToken{@link #IRTSRoom#updateToken} to update the publishing privilege Token.
     * @note Once a user's publishing privilege expires: <br>
     *        - Either during stream publishing or when attempting to publish a stream, the user will get the notification of `kPublishStateChangeReasonNoPublishPermission` via onVideoPublishStateChanged{@link #IRTCRoomEventHandler#onVideoPublishStateChanged}, onAudioPublishStateChanged{@link #IRTCRoomEventHandler#onAudioPublishStateChanged}.
     *        - The publishing streams of the user will be removed, and remote users in the room will get informed via onUserPublishStreamVideo{@link #IRTCRoomEventHandler#onUserPublishStreamVideo}, onUserPublishStreamAudio{@link #IRTCRoomEventHandler#onUserPublishStreamAudio}.
     * @list Engine Management
     */
    virtual void onPublishPrivilegeTokenWillExpire() {

    }
    /**
     * @locale zh
     * @type callback
     * @brief Token 订阅权限过期前 30 秒将触发该回调。 <br>
     *        收到该回调后，你需调用 updateToken{@link #IRTSRoom#updateToken} 更新 Token 订阅权限有效期。
     * @note 若收到该回调后未及时更新 Token，Token 订阅权限过期后，尝试新订阅流会失败，已订阅的流会取消订阅，并且会收到 onVideoSubscribeStateChanged{@link #IRTCRoomEventHandler#onVideoSubscribeStateChanged}、onAudioSubscribeStateChanged{@link #IRTCRoomEventHandler#onAudioSubscribeStateChanged} 回调，提示`kSubscribeStateChangeReasonNoSubscribePermission`，没有订阅权限。
     * @list 引擎管理
     */
    /**
     * @locale en
     * @type callback
     * @brief Callback triggered 30s before the subscribing privilege of the Token expires. <br>
     *        After receiving this callback, you must call updateToken{@link #IRTSRoom#updateToken} to update the subscribing privilege Token.
     * @note After a user's subscribing privilege expires, the user will get the notification of `kSubscribeStateChangeReasonNoSubscribePermission` via onVideoSubscribeStateChanged{@link #IRTCRoomEventHandler#onVideoSubscribeStateChanged}, onAudioSubscribeStateChanged{@link #IRTCRoomEventHandler#onAudioSubscribeStateChanged}.
     * @list Engine Management
     */
    virtual void onSubscribePrivilegeTokenWillExpire() {

    }
    /**
     * @locale zh
     * @type callback
     * @brief 房间内通话统计信息回调。 <br>
     *        用户进房开始通话后，每 2s 收到一次本回调。
     * @param stats 当前 RtcEngine 统计数据，详见 RtcRoomStats{@link #RtcRoomStats}
     * @list 房间管理
     */
    /**
     * @locale en
     * @type callback
     * @brief  In-room call statistics callback. <br>
     *        After the user enters the room and starts the call, he receives this callback every 2s.
     * @param stats Current RtcEngine statistics. See RtcRoomStats{@link #RtcRoomStats}
     * @list Room Management
     */
    virtual void onRoomStats(const RtcRoomStats& stats) {
        (void)stats;
    }
   /**
    * @locale zh
    * @hidden for internal use only
    * @type callback
    * @brief 房间事件回调。
    * @param room_id 房间 ID。
    * @param uid 用户 ID。
    * @param state 房间事件状态。详见 RoomEvent{@link #RoomEvent}。
    * @param info 房间封禁时，包含封禁时间。详见 RoomEventInfo{@link #RoomEventInfo}。
    * @list 房间管理
    */
   /**
    * @locale en
    * @hidden for internal use only
    * @type callback
    * @brief Callback on room event.
    * @param room_id  Room ID.
    * @param uid  User ID.
    * @param state Room event code. See RoomEvent{@link #RoomEvent}.
    * @param info When room is forbidden, it contains forbidden time. See RoomEventInfo{@link #RoomEventInfo}.
    */
    virtual void onRoomEvent(const char* room_id, const char* uid, RoomEvent state, const RoomEventInfo& info) {
        (void)room_id;
        (void)uid;
        (void)state;
        (void)info;
    }
    /**
     * @locale zh
     * @hidden(Linux)
     * @type callback
     * @brief 本地流数据统计以及网络质量回调。 <br>
     *        本地用户发布流成功后，SDK 会周期性（2s）的通过此回调事件通知用户发布的流在此次统计周期内的质量统计信息。 <br>
     *        统计信息通过 LocalStreamStats{@link #LocalStreamStats} 类型的回调参数传递给用户，其中包括发送音视频比特率、发送帧率、编码帧率，网络质量等。
     * @param stream_id 流 ID。
     * @param stream_info 流信息。
     * @param stats 当前 RtcEngine 统计数据，详见 LocalStreamStats{@link #LocalStreamStats}。
     * @list 音视频传输
     */
    /**
     * @locale en
     * @hidden(Linux)
     * @type callback
     * @brief Local stream data statistics and network quality callback. <br>
     *        After the local user publishes the flow successfully, the SDK will periodically (2s) notify the user through this callback event of the quality statistics of the published flow during this reference period. <br>
     *        Statistics are passed to the user through the callback parameters of the LocalStreamStats{@link #LocalStreamStats} type, including the sent audio & video bit rate, sent frame rate, encoded frame rate, network quality, etc.
     * @param stream_id Stream ID.
     * @param stream_info Stream information.
     * @param stats Current RtcEngine statistics. See LocalStreamStats{@link #LocalStreamStats}
     * @list Audio & Video Transport
     */
    virtual void onLocalStreamStats(const char* stream_id, const bytertc::StreamInfo& stream_info, const LocalStreamStats& stats) {
        (void)stream_id;
        (void)stream_info;
        (void)stats;
    }
    /**
     * @locale zh
     * @type callback
     * @brief 本地订阅的远端音/视频流数据统计以及网络质量回调。 <br>
     *        本地用户订阅流成功后，SDK 会周期性（2s）的通过此回调事件通知用户订阅的流在此次统计周期内的质量统计信息，包括：发送音视频比特率、发送帧率、编码帧率，网络质量等。
     * @param stream_id 流 ID。
     * @param stream_info 流信息。
     * @param stats 当前 RtcEngine 统计数据，详见 RemoteStreamStats{@link #RemoteStreamStats}
     * @list 音视频传输
     */
    /**
     * @locale en
     * @type callback
     * @brief Remote audio/video stream statistics and network quality callbacks for local subscriptions. <br>
     *        After the local user subscribes to the stream successfully, the SDK will periodically (2s) notify the user through this callback event of the quality statistics of the subscribed stream during this reference period, including: sending audio & video bit rate, sending frame rate, encoded frame rate, network quality, etc.
     * @param stream_id Stream ID.
     * @param stream_info Stream information.
     * @param stats Current RtcEngine statistics. See RemoteStreamStats{@link #RemoteStreamStats}
     * @list Audio & Video Transport
     */
    virtual void onRemoteStreamStats(const char* stream_id, const bytertc::StreamInfo& stream_info, const RemoteStreamStats& stats) {
        (void)stream_id;
        (void)stream_info;
        (void)stats;
    }
    /**
     * @locale zh
     * @type callback
     * @brief 远端可见用户加入房间，或房内不可见用户切换为可见的回调。 <br>
     *        1. 远端用户调用 setUserVisibility{@link #IRTCRoom#setUserVisibility} 方法将自身设为可见后加入房间时，房间内其他用户将收到该事件。 <br>
     *        2. 远端可见用户断网后重新连入房间时，房间内其他用户将收到该事件。 <br>
     *        3. 房间内隐身远端用户调用 setUserVisibility{@link #IRTCRoom#setUserVisibility} 方法切换至可见时，房间内其他用户将收到该事件。 <br>
     *        4. 新进房用户会收到进房前已在房内的可见用户的进房回调通知。
     * @param user_info 用户信息，详见 UserInfo{@link #UserInfo}
     * @list 房间管理
     */
     /**
      * @locale en
      * @type callback
      * @brief Callback for when a remote visible user joins the room, or when an invisible user in the room switches to being visible.<br>
      *        1. When a remote user sets themselves to be visible by calling the setUserVisibility {@link #IRTCRoom#setUserVisibility} method and then joins the room, other users in the room will receive this event.<br>
      *        2. When a remote visible user who has been disconnected re-enters the room, other users in the room will receive this event.<br>
      *        3. When a hidden remote user in the room switches to being visible by calling the setUserVisibility {@link #IRTCRoom#setUserVisibility} method, other users in the room will receive this event.<br>
      *        4. A new user entering the room will receive an entry callback notification for visible users who were already in the room before they entered.
      * @param user_info See UserInfo{@link #UserInfo}.
      * @list Room Management
      */
    virtual void onUserJoined(const UserInfo& user_info) {
        (void)user_info;
    }
    /**
     * @locale zh
     * @hidden
     * @deprecated since 3.36 and will be deleted in 3.51, use onUserUnpublishStream and onUserUnpublishScreen instead.
     * @type callback
     * @brief 房间内的远端用户停止发布音视频流时，本地用户会收到此回调。
     * @param stream 流的属性。参看 MediaStreamInfo{@link #MediaStreamInfo} 。
     * @param reason 远端流移除的原因。参看 StreamRemoveReason{@link #StreamRemoveReason} 。
     * @list 房间管理
     */
    /**
     * @locale en
     * @hidden
     * @deprecated since 3.45 and will be deleted in 3.51, use onUserUnpublishStream and onUserUnpublishScreen instead.
     * @type callback
     * @brief Local users will receive this callback when a remote user in the room stops publishing audio & video streams. Properties of the
     * @param stream Stream. See MediaStreamInfo{@link #MediaStreamInfo}.
     * @param reason The reason for the removal of the remote stream. See StreamRemoveReason{@link #StreamRemoveReason}.
     * @list Room Management
     */
    BYTERTC_DEPRECATED virtual void onStreamRemove(const MediaStreamInfo& stream, StreamRemoveReason reason) {
        (void)stream;
    }
    /**
     * @locale zh
     * @hidden
     * @deprecated since 3.36 and will be deleted in 3.51, use onUserPublishStream and onUserPublishScreen instead.
     * @type callback
     * @brief 房间内的用户发布新的音视频流时，房间内的其他用户会收到此回调。包括移除后又重新发布的流。
     * @param stream 流属性，参看 MediaStreamInfo{@link #MediaStreamInfo} 。
     * @list 房间管理
     */
    /**
     * @locale en
     * @hidden
     * @deprecated since 3.45 and will be deleted in 3.51, use onUserPublishStream and onUserPublishScreen instead.
     * @type callback
     * @brief When users in the room post a new audio & video stream, other users in the room will receive this callback. Includes streams that are removed and republished.
     * @param stream Stream properties. See MediaStreamInfo{@link #MediaStreamInfo}.
     * @list Room Management
     */
    BYTERTC_DEPRECATED virtual void onStreamAdd(const MediaStreamInfo& stream) {
        (void)stream;
    }

    /**
     * @locale zh
     * @type callback
     * @valid since 3.60.
     * @region 房间管理
     * @brief 房间内远端摄像头采集的媒体流的回调。
     * @param stream_id 流 ID。
     * @param stream_info 流信息。
     * @param is_publish 流是否发布。
     *         + `true`: 流发布
     *         + `false`: 流移除
     * @note 当房间内的远端用户调用 publishStreamVideo{@link #IRTCRoom#publishStreamVideo} 成功发布由摄像头采集的媒体流时，本地用户会收到该回调，此时本地用户可以自行选择是否调用 subscribeStreamVideo{@link #IRTCRoom#subscribeStreamVideo} 订阅此流。
     * @list 音视频传输
     */
    /**
     * @locale en
     * @type callback
     * @valid since 3.60. Since version 3.60, this callback replaces the `onUserPublishStream` and `onUserUnpublishStream` methods for the functions described below. If you have upgraded to SDK version 3.60 or later and are still using these methods, please migrate to this callback.
     * @brief Callback on a remote user publishing or stopping publishing remote camera-video streams in the room.
     * @param stream_id Stream ID.
     * @param stream_info Stream information.
     * @param is_publish Whether the stream is published.
     *         + `true`: Published
     *         + `false`: Removed
     * @note You will receive this callback after a remote user successfully published media streams captured by camera/microphone in the room with publishStreamVideo{@link #IRTCRoom#publishStreamVideo}. Then you can choose whether to call subscribeStreamVideo{@link #IRTCRoom#subscribeStreamVideo} to subscribe to the streams or not.
     */
    virtual void onUserPublishStreamVideo(const char* stream_id, const StreamInfo& stream_info, bool is_publish){
        (void)stream_id;
        (void)stream_info;
        (void)is_publish;
    }

    /**
     * @locale zh
     * @type callback
     * @valid since 3.60. 自 3.60 起，该回调替代了 `onUserPublishStream`、`onUserUnpublishStream` 方法来实现下述功能。如果你已升级至 3.60 及以上版本 SDK，且还在使用这两个方法，请迁移至该回调。
     * @brief 房间内远端麦克风采集的媒体流的回调。
     * @param stream_id 流 ID。
     * @param stream_info 流信息。
     * @param is_publish 流是否发布。
     *         + `true`: 流发布
     *         + `false`: 流移除
     * @note 当房间内的远端用户调用 publishStreamAudio{@link #IRTCRoom#publishStreamAudio} 成功发布由麦克风采集的媒体流时，本地用户会收到该回调，此时本地用户可以自行选择是否调用 subscribeStreamAudio{@link #IRTCRoom#subscribeStreamAudio} 订阅此流。
     * @list 音视频传输
     * @order 2
     */
    /**
     * @locale en
     * @type callback
     * @valid since 3.60. Since version 3.60, this callback replaces the `onUserPublishStream` and `onUserUnpublishStream` methods for the functions described below. If you have upgraded to SDK version 3.60 or later and are still using these methods, please migrate to this callback.
     * @brief Callback on a remote user publishing or stopping publishing remote microphone-audio streams in the room.
     * @param stream_id Stream ID.
     * @param stream_info Stream information.
     * @param is_publish Whether the stream is published.
     *         + `true`: Published
     *         + `false`: Removed
     * @note You will receive this callback after a remote user successfully published media streams captured by camera/microphone in the room with publishStreamAudio{@link #IRTCRoom#publishStreamAudio}. Then you can choose whether to call subscribeStreamAudio{@link #IRTCRoom#subscribeStreamAudio} to subscribe to the streams or not.
     */
    virtual void onUserPublishStreamAudio(const char* stream_id, const StreamInfo& stream_info, bool is_publish){
        (void)stream_id;
        (void)stream_info;
        (void)is_publish;
    }

    /**
     * @locale zh
     * @hidden for internal use only
     * @type callback
     * @brief 当发布流成功的时候回调该事件。
     * @param user_id 发布流的用户 ID。
     * @param is_screen 该流是否为屏幕流。 <br>
     *             - Ture: 屏幕流；
     *             - False: 非屏幕流。
     * @list 房间管理
     */
    /**
     * @locale en
     * @hidden for internal use only
     * @type callback
     * @brief Callback the event when the release flow is successful.
     * @param user_id The user ID of the published stream.
     * @param is_screen Whether the stream is a screen stream. <br>
     *              - Ture: Screen stream;
     *              - False: Non-screen stream.
     * @list Room Management
     */
    virtual void onStreamPublishSuccess(const char* user_id, bool is_screen) {
        (void)is_screen;
    }

    /**
     * @locale zh
     * @hidden for internal use only
     */
     /**
     * @locale en
     * @hidden for internal use only
     */ 
    virtual void onRoomModeChanged(RtcRoomMode mode) {
        (void)mode;
    }

    /**
     * @locale zh
     * @hidden for internal use only
     * @brief callback when the maximum screen share fps is changed
     * @param fps maximum screen share fps
     */
    /**
     * @locale en
     * @hidden for internal use only
     * @brief callback when the maximum screen share fps is changed
     * @param fps maximum screen share fps
     */
    virtual void onMaximumScreenShareFpsUpdated(int fps) {
        (void)fps;
    }

    /**
     * @locale zh
     * @hidden for internal use only
     * @brief 最大屏幕共享帧率改变时的回调
     * @param screen_pixels 为了保持帧率而推荐的最大视频宽度×高度的值。
     */
    /**
     * @locale en
     * @hidden for internal use only
     * @brief Callback when the maximum screen shared pixels (resolution) is chang
     * @param screenPixels The recommended maximum video width × height value to maintain the frame rate.
     */
    virtual void onMaximumScreenSharePixelsUpdated(int screen_pixels) {
        (void)screen_pixels;
    }
    /**
     * @locale zh
     * @type callback
     * @brief 通过调用服务端 BanUserStream/UnbanUserStream 方法禁用/解禁指定房间内指定用户视频流的发送时，触发此回调。
     * @param uid 被禁用/解禁的视频流用户 ID
     * @param banned 视频流发送状态 <br>
     *        - true: 视频流发送被禁用
     *        - false: 视频流发送被解禁
     * @note
     *        - 房间内指定用户被禁止/解禁视频流发送时，房间内所有用户都会收到该回调。
     *        - 若被封禁用户断网或退房后再进房，则依然是封禁状态，且房间内所有人会再次收到该回调。
     *        - 指定用户被封禁后，房间内其他用户退房后再进房，会再次收到该回调。
     *        - 同一房间解散后再次创建，房间内状态清空。
     * @list 音视频传输
     */
    /**
     * @locale en
     * @type callback
     * @brief All the users in the room will get informed via this callback when a user is banned or the ban of the user has been lifted by calling BanUserStream/UnbanUserStream on the server side.
     * @param uid User ID of the video stream that was disabled/unbanned
     * @param banned Video stream sending status <br>
     *         - true: Video stream sending was disabled
     *         - false: Video stream sending was unbanned
     * @note
     *         - When the specified user in the room is disabled/unbanned Video stream sending, all users in the room will receive the callback.
     *        - If the banned user leaves or disconnects and then rejoins the room, the user is still banned from publishing audio stream, and all users in the room will be informed via the callback.
     *         - After the specified user is banned, other users in the room will check out and enter the room again, and will receive the callback again.
     *         - If the Audio selection is enabled in the console, only the user whose stream is banned will receive this callback.
     * @list Audio & Video Transport
     */
    virtual void onVideoStreamBanned(const char* uid, bool banned) {
        (void)uid;
        (void)banned;
    }
    /**
     * @locale zh
     * @type callback
     * @brief 发布端调用 setMultiDeviceAVSync{@link #IRTCRoom#setMultiDeviceAVSync} 后音视频同步状态发生改变时，会收到此回调。
     * @param state 音视频同步状态，参看 AVSyncState{@link #AVSyncState}。
     * @list 音视频传输
     */
    /**
     * @locale en
     * @type callback
     * @brief Stream publisher will receive this callback when the A/V synchronization state changes after setMultiDeviceAVSync{@link #IRTCRoom#setMultiDeviceAVSync} is called.
     * @param state A/V synchronization state, see AVSyncState{@link #AVSyncState}.
     * @list Audio & Video Transport
     */
    virtual void onAVSyncStateChange(AVSyncState state) {
        (void)state;
    }
    /**
     * @locale zh
     * @type callback
     * @brief 通过调用服务端 BanUserStream/UnbanUserStream 方法禁用/解禁指定房间内指定用户音频流的发送时，触发此回调。
     * @param uid 被禁用/解禁的音频流用户 ID
     * @param banned 音频流发送状态 <br>
     *        - true: 音频流发送被禁用
     *        - false: 音频流发送被解禁
     * @note
     *        - 房间内指定用户被禁止/解禁音频流发送时，房间内所有用户都会收到该回调。
     *        - 若被封禁用户断网或退房后再进房，则依然是封禁状态，且房间内所有人会再次收到该回调。
     *        - 指定用户被封禁后，房间内其他用户退房后再进房，会再次收到该回调。
     *        - 在控制台开启音频选路后，只有被封禁/解禁用户会收到该回调。
     *        - 同一房间解散后再次创建，房间内状态清空。
     * @list 音频管理
     */
    /**
     * @locale en
     * @type callback
     * @brief All the users in the room will get informed via this callback when a user is banned or the ban of the user has been lifted by calling BanUserStream/UnbanUserStream on the server side.
     * @param uid Disabled/unbanned audio stream user ID
     * @param banned Audio stream sending status <br>
     *         - true: audio stream sending is disabled
     *         - false: audio stream sending is unbanned
     * @note
     *         - Specified users in the room are prohibited/all users in the room when audio stream sending is unbanned Will receive the callback.
     *        - If the banned user leaves or disconnects and then rejoins the room, the user is still banned from publishing audio stream, and all users in the room will be informed via the callback.
     *         - After the specified user is banned, other users in the room will check out and enter the room again, and will receive the callback again.
     *         - The same room is created again after dissolution, and the state in the room is empty.
     *         - If the Audio selection is enabled in the console, only the user whose stream is banned will receive this callback.
     * @list Audio Management
     */
    virtual void onAudioStreamBanned(const char* uid, bool banned) {
        (void)uid;
        (void)banned;
    }
    /**
     * @locale zh
     * @type callback
     * @brief 跨房间媒体流转发状态和错误回调
     * @param infos 跨房间媒体流转发目标房间信息数组，详见 ForwardStreamStateInfo{@link #ForwardStreamStateInfo}
     * @param info_count 数组长度，代表目标房间数
     * @list 音视频传输
     */
    /**
     * @locale en
     * @type callback
     * @brief Callback returning the state and errors during relaying the media stream to each of the rooms
     * @param infos Array of the state and errors of each target room. Refer to ForwardStreamStateInfo{@link #ForwardStreamStateInfo} for more information.
     * @param info_count The number of the target rooms
     * @list Audio & Video Transport
     */
    virtual void onForwardStreamStateChanged(ForwardStreamStateInfo* infos, int info_count) {
    }
    /**
     * @locale zh
     * @type callback
     * @brief 跨房间媒体流转发事件回调
     * @param infos 跨房间媒体流转发目标房间事件数组，详见 ForwardStreamEventInfo{@link #ForwardStreamEventInfo}
     * @param info_count 数组长度，代表目标房间数
     * @list 音视频传输
     */
    /**
     * @locale en
     * @type callback
     * @brief Callback returning the events during relaying the media stream to each room
     * @param infos Array of the event of each target room. Refer to ForwardStreamEventInfo{@link #ForwardStreamEventInfo} for more information.
     * @param info_count The number of the target rooms
     * @list Audio & Video Transport
     */
    virtual void onForwardStreamEvent(ForwardStreamEventInfo* infos, int info_count) {
    }
    /**
     * @locale zh
     * @type callback
     * @brief 加入房间并发布或订阅流后， 以每 2 秒一次的频率，报告本地用户和已订阅的远端用户的上下行网络质量信息。
     * @param local_quality 本端网络质量，详见 NetworkQualityStats{@link #NetworkQualityStats}。
     * @param remote_qualities 已订阅用户的网络质量，详见 NetworkQualityStats{@link #NetworkQualityStats}。
     * @param remote_quality_num `remoteQualities` 数组长度
     * @note 更多通话中的监测接口，详见[通话中质量监测](https://www.volcengine.com/docs/6348/106866)。
     * @list 音视频传输
     */
    /**
     * @locale en
     * @type callback
     * @brief Report the network quality of the users every 2s after the local user joins the room and starts publishing or subscribing to a stream.
     * @param local_quality Local network quality. Refer to NetworkQualityStats{@link #NetworkQualityStats} for details.
     * @param remote_qualities Network quality of the subscribed users. Refer to NetworkQualityStats{@link #NetworkQualityStats} for details.
     * @param remote_quality_num Array length of `remoteQualities`
     * @note See [In-call Stats](106866) for more information.
     * @list Audio & Video Transport
     */
    virtual void onNetworkQuality(const NetworkQualityStats& local_quality, const NetworkQualityStats* remote_qualities, int remote_quality_num) {
    }
    /**
     * @locale zh
     * @valid since 3.52.
     * @type callback
     * @brief 调用 setRoomExtraInfo{@link #IRTCRoom#setRoomExtraInfo} 设置房间附加信息结果的回调。
     * @param task_id 调用 setRoomExtraInfo 的任务编号。
     * @param result 设置房间附加信息的结果，详见 SetRoomExtraInfoResult{@link #SetRoomExtraInfoResult}
     * @list 房间管理
     */
    /**
     * @locale en
     * @valid since 3.52.
     * @type callback
     * @brief Callback on the result of calling setRoomExtraInfo{@link #IRTCRoom#setRoomExtraInfo} to set extra information about the room.
     * @param task_id The task ID of the API call.
     * @param result See SetRoomExtraInfoResult{@link #SetRoomExtraInfoResult} for the setting results and reasons.
     * @list Room Management
     */
    virtual void onSetRoomExtraInfoResult(int64_t task_id, SetRoomExtraInfoResult result) {
        (void)task_id;
        (void)result;
    }
    /**
     * @locale zh
     * @valid since 3.52.
     * @type callback
     * @brief 接收同一房间内，其他用户调用 setRoomExtraInfo{@link #IRTCRoom#setRoomExtraInfo} 设置的房间附加信息的回调。
     * @param key 房间附加信息的键值
     * @param value 房间附加信息的内容
     * @param last_update_user_id 最后更新本条信息的用户 ID。
     * @param last_update_time_ms 最后更新本条信息的 Unix 时间，单位：毫秒。
     * @note 新进房的用户会收到进房前房间内已有的全部附加信息通知。
     * @list 房间管理
     */
    /**
     * @locale en
     * @valid since 3.52.
     * @type callback
     * @brief Callback used to receive the extra information set by the other users in the same room with setRoomExtraInfo{@link #IRTCRoom#setRoomExtraInfo}.
     * @param key Key of the extra information.
     * @param value Content of the extra information.
     * @param last_update_user_id The ID of the last user who updated this information.
     * @param last_update_time_ms The Unix time in ms when this information was last updated.
     * @note The user will receive the all the historical information in the room once the user joins the room.
     * @list Room Management
     */
    virtual void onRoomExtraInfoUpdate(const char*key, const char* value, const char* last_update_user_id, int64_t last_update_time_ms) {
        (void)key;
        (void)value;
        (void)last_update_user_id;
        (void)last_update_time_ms;
    }
    /**
     * @locale zh
     * @valid since 3.60.
     * @type callback
     * @brief 接收同一房间内，其他用户调用 setStreamExtraInfo{@link #IRTCRoom#setStreamExtraInfo} 设置的流附加信息的回调。
     * @param stream_id 流 ID
     * @param stream_info 流信息，详见 StreamInfo{@link #StreamInfo}
     * @param extra_info 流附加信息的内容
     * @note 新进房的用户会收到进房前房间内已有的全部附加信息通知。
     * @list 房间管理
     */
    /**
     * @locale en
     * @valid since 3.60.
     * @type callback
     * @brief Callback used to receive the extra information set by the other users in the same room with setStreamExtraInfo{@link #IRTCRoom#setStreamExtraInfo}.
     * @param stream_id Stream ID
     * @param stream_info Stream information. Refer to StreamInfo{@link #StreamInfo} for details.
     * @param extra_info Content of the extra information.
     * @note The user will receive the all the historical information in the room once the user joins the room.
     * @list Room Management
     */
    virtual void onRoomStreamExtraInfoUpdate(const char* stream_id, const StreamInfo& stream_info, const char* extra_info) {
        (void)stream_id;
        (void)stream_info;
        (void)extra_info;
    }
    /**
     * @locale zh
     * @valid since 3.54
     * @type callback
     * @brief 用户调用 setUserVisibility{@link #IRTCRoom#setUserVisibility} 设置用户可见性的回调。
     * @param current_user_visibility 当前用户的可见性。 <br>
     *        - true: 可见，用户可以在房间内发布音视频流，房间中的其他用户将收到用户的行为通知，例如进房、开启视频采集和退房。
     *        - false: 不可见，用户不可以在房间内发布音视频流，房间中的其他用户不会收到用户的行为通知，例如进房、开启视频采集和退房。
     * @param error_code 设置用户可见性错误码，参看 UserVisibilityChangeError{@link #UserVisibilityChangeError}。
     * @list 房间管理
     */
    /**
     * @locale en
     * @valid since 3.54
     * @type callback
     * @brief Callback for user to set user visibility by calling setUserVisibility{@link #IRTCRoom#setUserVisibility}.
     * @param current_user_visibility Visibility of the current user. <br>
     *        - true: Visible. The user can publish media streams. The other users in the room get informed of the behaviors of the user, such as joining room, starting video capture, and leaving room.
     *        - false: Invisible. The user cannot publish media streams. The other users in the room do not get informed of the behaviors of the user, such as joining room, starting video capture, or leaving room.
     * @param error_code Error code for setting user visibility. See UserVisibilityChangeError{@link #UserVisibilityChangeError}.
     * @list Room Management
     */
    virtual void onUserVisibilityChanged(bool current_user_visibility, UserVisibilityChangeError error_code) {
        (void)current_user_visibility;
        (void)error_code;
    }
    /**
     * @locale zh
     * @type callback
     * @brief 字幕状态发生改变回调。 <br>
     *         当用户调用 startSubtitle{@link #IRTCRoom#startSubtitle} 和 stopSubtitle{@link #IRTCRoom#stopSubtitle} 使字幕状态发生改变或出现错误时，触发该回调。
     * @param state 字幕状态。参看 SubtitleState{@link #SubtitleState}。
     * @param error_code 字幕任务错误码。参看 SubtitleErrorCode{@link #SubtitleErrorCode}。
     * @param error_message 与第三方服务有关的错误信息。
     * @list 字幕翻译服务
     */
    /**
     * @locale en
     * @hidden currently not available
     * @type callback
     * @brief  Callback on subtitle states. <br>
     *         After you call startSubtitle{@link #IRTCRoom#startSubtitle} and stopSubtitle{@link #IRTCRoom#stopSubtitle}, you will receive this callback which informs you of the states and error codes of the subtitling task, as well as detailed information on the third party services' errors.
     * @param state The states of subtitles. Refer to SubtitleState{@link #SubtitleState} for details.
     * @param error_code  Error codes of the subtitling task. Refer to SubtitleErrorCode{@link #SubtitleErrorCode}.
     * @param error_message Detailed information on the third party services' errors.
     * @list Subtitles
     */
    virtual void onSubtitleStateChanged(SubtitleState state, SubtitleErrorCode error_code, const char* error_message) {
    }
    /**
     * @locale zh
     * @type callback
     * @brief 字幕相关内容回调。 <br>
     *         当用户调用 startSubtitle{@link #IRTCRoom#startSubtitle} 后会收到此回调，通知字幕的相关信息。
     * @param subtitles 字幕消息内容。参看 SubtitleMessage{@link #SubtitleMessage}。
     * @param cnt 字幕消息个数。
     * @list 字幕翻译服务
     */
    /**
     * @locale en
     * @hidden currently not available
     * @type callback
     * @brief  Callback on subtitle messages. <br>
     *         After calling startSubtitle{@link #IRTCRoom#startSubtitle}, you will receive this callback which informs you of the related information on subtitles.
     * @param subtitles Subtitle messages. Refer to SubtitleMessage{@link #SubtitleMessage} for details.
     * @param cnt The number of subtitle messages.
     * @list Subtitles
     */
    virtual void onSubtitleMessageReceived(const SubtitleMessage* subtitles, int cnt) {
    }

    /**
     * @locale zh
     * @type callback
     * @valid since 3.60.
     * @brief 发布端调用 setMultiDeviceAVSync{@link #IRTCRoom#setMultiDeviceAVSync} 后音视频同步状态发生错误时，会收到此回调。
     * @param room_id 房间 ID。
     * @param uid 用户 ID。
     * @param event 音视频同步状态错误，参看 AVSyncEvent{@link #AVSyncEvent}。
     * @list 音视频传输
     */
    /**
     * @locale en
     * @type callback
     * @valid since 3.60.
     * @brief Stream publisher will receive this callback when the A/V synchronization error occurs after setMultiDeviceAVSync{@link #IRTCRoom#setMultiDeviceAVSync} is called.
     * @param room_id  Room ID.
     * @param uid  User ID.
     * @param event A/V synchronization error, see AVSyncEvent{@link #AVSyncEvent}.
     * @list Audio & Video Transport
     */
    virtual void onAVSyncEvent(const char* room_id, const char* uid, AVSyncEvent event) {
    }
};

} // namespace bytertc
