/*
 * Copyright (c) 2022 The VolcEngineRTC project authors. All Rights Reserved.
 * @brief VolcEngineRTC Room Event Handler Interface
*/

#pragma once

#include "rtc/bytertc_defines.h"

namespace bytertc {
/**
 * @locale zh
 * @type callback
 * @brief 实时信令事件回调接口
 */
/**
 * @locale en
 * @type callback
 * @brief  Audio & video room event callback interface
 */
class IRTSRoomEventHandler {
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
    virtual ~IRTSRoomEventHandler() {}

    /**
     * @locale zh
     * @type callback
     * @region 房间管理
     * @brief 房间状态改变回调，加入房间、异常退出房间、发生房间相关的警告或错误时会收到此回调。
     * @param room_id 房间 ID。
     * @param uid 用户 ID。
     * @param state 房间状态码。  <br>
     *              + 0: 加入房间成功。  <br>
     *              + !0: 加入房间失败、异常退房、发生房间相关的警告或错误。具体原因参看 ErrorCode{@link #ErrorCode} 及 WarningCode{@link #WarningCode}。
     * @param extra_info 额外信息，如 `{"elapsed":1187,"join_type":0}`。
     *                  `join_type`表示加入房间的类型，`0`为首次进房，`1`为重连进房。
     *                  `elapsed`表示加入房间耗时，即本地用户从调用 joinRoom{@link #IRTSRoom#joinRoom} 到加入房间成功所经历的时间间隔，单位为 ms。
     */
    /**
     * @locale en
     * @type callback
     * @region Multi-room
     * @brief Callback on room state changes. Via this callback you get notified of room relating warnings,  errors and events. For example, the user joins the room, the user is removed from the room, and so on.
     * @param room_id  Room ID.
     * @param uid  User ID.
     * @param state Room state code. <br>
     *              + 0: Join room success. <br>
     *              + !0: Failed to join a room, abnormal exit, room-related warnings or errors. See ErrorCode{@link #ErrorCode} and WarningCode{@link #WarningCode} for specific indications.
     * @param extra_info Extra information.
     *                 `join_type` indicates the type of room the user joins. `0` means the user joins the room for the first time, and `1` means the user rejoins the room. <br>
     *                 `elapsed` indicates the time interval from calling joinRoom{@link #IRTSRoom#joinRoom} to successfully joining room, in ms.
     */
    virtual void onRoomStateChanged(const char* room_id, const char* uid,
            int state, const char* extra_info) {
        (void)room_id;
        (void)state;
        (void)extra_info;
    }

    /**
     * @locale zh
     * @type callback
     * @region 房间管理
     * @brief 离开房间回调。  <br>
     *        用户调用 leaveRoom{@link #IRTSRoom#leaveRoom} 方法后，SDK 会停止所有的发布订阅流，并在释放所有与通话相关的音频资源后，通过此回调通知用户离开房间成功。  <br>
     * @notes  <br>
     *       + 用户调用 leaveRoom{@link #IRTSRoom#leaveRoom} 方法离开房间后，如果立即调用 destroy{@link #IRTSRoom#destroy} 销毁房间实例或 destroyRTS{@link #destroyRTS} 方法销毁 RTS 引擎，则将无法收到此回调事件。  <br>
     *       + 离开房间后，如果 App 需要使用系统音频设备，则建议收到此回调后再初始化音频设备，否则可能由于 SDK 占用音频设备而导致初始化失败。  <br>
     */
    /**
     * @locale en
     * @type callback
     * @region Room management
     * @brief Leave the room callback.   <br>
     *        After the user calls the leaveRoom{@link #IRTSRoom#leaveRoom} method, the SDK stops all publishing subscription streams and notifies the user to leave the room successfully through this callback after releasing all audio & video resources related to the call. <br>
     * @notes   <br>
     *        + After the user calls the leaveRoom{@link #IRTSRoom#leaveRoom} method to leave the room, if destroy{@link #IRTSRoom#destroy} is called to destroy the room instance or destroyRTS{@link #destroyRTS} method is called to destroy the RTS engine immediately, this callback event will not be received. <br>
     *        + If the app needs to use the system audio & video device after leaving the room, it is recommended to initialize the audio & video device after receiving this callback, otherwise the initialization may fail due to the SDK occupying the audio & video device. <br>
     */
    virtual void onLeaveRoom(const RtcRoomStats& stats) {
        (void)stats;
    }

    /**
     * @locale zh
     * @type callback
     * @region 房间管理
     * @brief 远端用户加入房间的回调。
     * @param user_info 用户信息，详见 UserInfo{@link #UserInfo}
     */
    /**
     * @locale en
     * @type callback
     * @region Room management
     * @brief Callback about remote users joining the room.
     * @param user_info See UserInfo{@link #UserInfo}.
     */
    virtual void onUserJoined(const UserInfo& user_info) {
        (void)user_info;
    }

    /**
     * @locale zh
     * @type callback
     * @brief 远端用户离开房间时，本地用户会收到此事件
     * @param uid 离开房间的远端用户 ID。  <br>
     * @param reason 用户离开房间的原因。参看 UserOfflineReason{@link #UserOfflineReason}。
     */
    /**
     * @locale en
     * @type callback
     * @brief This callback is triggered when a remote user is disconnected.
     * @param uid ID of the user who leaves the room. <br>
     * @param reason Reason to leave the room. See UserOfflineReason{@link #UserOfflineReason}.
     */
    virtual void onUserLeave(const char* uid, UserOfflineReason reason) {
        (void)uid;
        (void)reason;
    }

    /**
     * @locale zh
     * @type callback
     * @region 实时消息通信
     * @brief 接收到房间内广播消息的回调。
     * @param uid  <br>
     *        消息发送者 ID
     * @param message  <br>
     *        收到的消息内容
     * @notes 同一房间内其他用户调用 sendRoomMessage{@link #IRTSRoom#sendRoomMessage} 发送广播消息时会收到该回调。
     */
    /**
     * @locale en
     * @type callback
     * @region Real-time messaging
     * @brief Receives a callback for broadcast messages in the room.
     * @param  uid <br>
     *         User ID of the message sender
     * @param  message <br>
     *         Received message content
     * @notes Other users in the same room will receive this callback when they call sendRoomMessage{@link #IRTSRoom#sendRoomMessage} to send a broadcast message.
     */
    virtual void onRoomMessageReceived(const char* uid, const char* message) {
        (void)uid;
        (void)message;
    }

    /**
     * @locale zh
     * @type callback
     * @region 实时消息通信
     * @brief 接收到房间内二进制广播消息的回调。
     * @param uid  <br>
     *        消息发送者 ID
     * @param size <br>
     *        收到的二进制消息长度
     * @param message <br>
     *        收到的二进制消息内容
     * @notes 同一房间内其他用户调用 sendRoomBinaryMessage{@link #IRTSRoom#sendRoomBinaryMessage} 发送二进制广播消息时会收到该回调。
     */
    /**
     * @locale en
     * @type callback
     * @region Real-time messaging
     * @brief Receives a callback to a binary broadcast message in the room.
     * @param  uid <br>
     *         User ID of the message sender
     * @param  size <br>
     *         Binary message length received
     * @param  message <br>
     *        Binary message content received
     * @notes Other users in the same room call sendRoomBinaryMessage{@link #IRTSRoom#sendRoomBinaryMessage} Receive this callback when sending a binary broadcast message.
     */
    virtual void onRoomBinaryMessageReceived(const char* uid, int size, const uint8_t* message) {
        (void)uid;
        (void)size;
        (void)message;
    }

    /**
     * @locale zh
     * @type callback
     * @region 实时消息通信
     * @brief 当调用 sendRoomMessage{@link #IRTSRoom#sendRoomMessage} 函数发送消息后，回调此条消息的发送结果（反馈）。
     * @param msgid  <br>
     *        本条消息的 ID
     * @param error  <br>
     *        消息发送结果  <br>
     *        详见 RoomMessageSendResult{@link #RoomMessageSendResult}
     */
    /**
     * @locale en
     * @type callback
     * @region Real-time messaging
     * @brief When the sendRoomMessage{@link #IRTSRoom#sendRoomMessage} function is called to send a message, the sending result of this message (feedback) is called back.
     * @param msgid <br>
     *         ID of this message
     * @param error <br>
     *         Message sending result <br>
     *         See RoomMessageSendResult{@link #RoomMessageSendResult}
     */
    virtual void onRoomMessageSendResult(int64_t msgid, int error) {
        (void)msgid;
        (void)error;
    }

    /**
     * @locale zh
     * @type callback
     * @brief 该接口将于 3.64 onUserMessageReceived{@link #IRTCRoomEventHandler#onUserMessageReceived-2} 代替。
     * @brief 收到来自房间中其他用户通过 sendUserMessage{@link #IRTCRoom#sendUserMessage} 发来的点对点文本消息时，会收到此回调。
     * @param uid <br>
     *        消息发送者 ID 。
     * @param message <br>
     *        收到的文本消息内容。
     * @note
     *        - 你必须先调用 sendUserMessage{@link #IRTCRoom#sendUserMessage} 方法，才能收到该回调。
     * @list 消息
     */
    /**
     * @locale en
     * @type callback
     * @brief Receive this callback when you receive a text message (P2P) from another user in the room.
     * @param uid <br>
     *         User ID of the message sender.
     * @param message <br>
     *         The content of the received text message.
     * @note
     *         - You must call the sendUserMessage{@link #IRTCRoom#sendUserMessage} method before you can receive the callback.
     * @list Messaging
     */
    virtual void onUserMessageReceived(const char* uid, const char* message) {
        (void)uid;
        (void)message;
    }

    /**
     * @locale zh
     * @type callback
     * @brief 该接口将于 3.64 onUserBinaryMessageReceived{@link #IRTCRoomEventHandler#onUserBinaryMessageReceived-2} 代替。
     * @brief 收到来自房间中其他用户通过 sendUserBinaryMessage{@link #IRTCRoom#sendUserBinaryMessage} 发来的点对点二进制消息时，会收到此回调。
     * @param uid 消息发送者 ID 。
     * @param size 消息长度
     * @param message 收到的二进制消息内容。
     * @list 消息
     */
    /**
     * @locale en
     * @type callback
     * @brief A single user receives a callback (P2P) of binary messages from other uid-owned users in the same room.
     * @param uid User ID of the message sender.
     * @param size Message length.
     * @param message The content of the received binary message.
     * @list Messaging
     */
    virtual void onUserBinaryMessageReceived(const char* uid, int size, const uint8_t* message) {
        (void)uid;
        (void)size;
        (void)message;
    }

    /**
     * @locale zh
     * @type callback
     * @brief 向房间内单个用户发送文本或二进制消息后（P2P），消息发送方会收到该消息发送结果回调。
     * @param msgid <br>
     *            本条消息的 ID。
     * @param error <br>
     *            文本或二进制消息发送结果，详见 UserMessageSendResult{@link #UserMessageSendResult}
     * @note
     *        - 你必须先调用 sendUserMessage{@link #IRTCRoom#sendUserMessage} 或 sendUserBinaryMessage{@link #IRTCRoom#sendUserBinaryMessage} 接口，才能收到此回调。
     * @list 消息
     */
    /**
     * @locale en
     * @type callback
     * @brief After sending a text or binary message (P2P) to a single user in the room, the message sender receives a callback with the result of the message.
     * @param msgid <br>
     *             The ID of this message.
     * @param error <br>
     *             Text or binary message sending results. See UserMessageSendResult{@link #UserMessageSendResult}
     * @note
     *         - You must first call the sendUserMessage{@link #IRTCRoom#sendUserMessage} or sendUserBinaryMessage{@link #IRTCRoom#sendUserBinaryMessage} interface to receive this callback.
     * @list Messaging
     */
    virtual void onUserMessageSendResult(int64_t msgid, int error) {
        (void)msgid;
        (void)error;
    }

    /**
     * @locale zh
     * @type callback
     * @brief 接收到房间内广播消息的回调。
     * @param msgid <br>
     *        消息编号
     * @param uid <br>
     *        消息发送者 ID
     * @param message <br>
     *        收到的消息内容
     * @note 同一房间内其他用户调用 sendRoomMessage{@link #IRTCRoom#sendRoomMessage} 发送广播消息时会收到该回调。
     * @list 消息
     */
    /**
     * @locale en
     * @type callback
     * @brief Receives a callback for broadcast messages in the room.
     * @param msgid <br>
     *         Message number
     * @param uid <br>
     *         User ID of the message sender
     * @param message <br>
     *         Received message content
     * @note Other users in the same room will receive this callback when they call sendRoomMessage{@link #IRTCRoom#sendRoomMessage} to send a broadcast message.
     * @list Messaging
     */
    virtual void onRoomMessageReceived(int64_t msgid, const char* uid, const char* message) {
        (void)msgid;
        (void)uid;
        (void)message;
    }
    /**
     * @locale zh
     * @type callback
     * @brief 收到房间内广播二进制消息的回调。 <br>
     *        房间内其他用户调用 sendRoomBinaryMessage{@link #IRTCRoom#sendRoomBinaryMessage} 发送广播二进制消息时，收到此回调。
     * @param msgid <br>
     *        消息编号
     * @param uid <br>
     *        消息发送者 ID
     * @param message <br>
     *        收到的二进制消息内容
     * @param size <br>
     *        收到的二进制消息长度
     * @note 同一房间内其他用户调用 sendRoomBinaryMessage{@link #IRTCRoom#sendRoomBinaryMessage} 发送二进制广播消息时会收到该回调。
     * @list 消息
     */
    /**
     * @locale en
     * @type callback
     * @brief Receives a callback to a binary broadcast message in the room.
     * @param msgid <br>
     *        Message number
     * @param uid <br>
     *         User ID of the message sender
     * @param message <br>
     *        Binary message content received
     * @param size <br>
     *         Length of the received binary message
     * @note Other users in the same room call sendRoomBinaryMessage{@link #IRTCRoom#sendRoomBinaryMessage} Receive this callback when sending a binary broadcast message.
     * @list Messaging
     */
    virtual void onRoomBinaryMessageReceived(int64_t msgid, const char* uid, const uint8_t* message, int size) {
        (void)msgid;
        (void)uid;
        (void)message;
        (void)size;
    }
    /**
     * @locale zh
     * @type callback
     * @brief 收到来自房间中其他用户通过 sendUserMessage{@link #IRTCRoom#sendUserMessage} 发来的点对点文本消息时，会收到此回调。
     * @param msgid <br>
     *        消息编号。
     * @param uid <br>
     *        消息发送者 ID 。
     * @param message <br>
     *        收到的文本消息内容。
     * @note
     *        - 你必须先调用 sendUserMessage{@link #IRTCRoom#sendUserMessage} 方法，才能收到该回调。
     * @list 消息
     */
    /**
     * @locale en
     * @type callback
     * @brief Receive this callback when you receive a text message (P2P) from another user in the room.
     * @param msgid <br>
     *         Message number.
     * @param uid <br>
     *         User ID of the message sender.
     * @param message <br>
     *         The content of the received text message.
     * @note
     *         - You must call the sendUserMessage{@link #IRTCRoom#sendUserMessage} method before you can receive the callback.
     * @list Messaging
     */
    virtual void onUserMessageReceived(int64_t msgid, const char* uid, const char* message) {
        (void)msgid;
        (void)uid;
        (void)message;
    }
    /**
     * @locale zh
     * @type callback
     * @brief 收到来自房间中其他用户通过 sendUserBinaryMessage{@link #IRTCRoom#sendUserBinaryMessage} 发来的点对点二进制消息时，会收到此回调。
     * @param msgid 消息编号。
     * @param uid 消息发送者 ID 。
     * @param message 收到的二进制消息内容。
     * @param size 消息长度
     * @list 消息
     */
    /**
     * @locale en
     * @type callback
     * @brief A single user receives a callback (P2P) of binary messages from other uid-owned users in the same room.
     * @param msgid Message number.
     * @param uid User ID of the message sender.
     * @param message The content of the received binary message.
     * @param size Message length.
     * @list Messaging
     */
    virtual void onUserBinaryMessageReceived(int64_t msgid, const char* uid, const uint8_t* message, int size) {
        (void)msgid;
        (void)uid;
        (void)message;
        (void)size;
    }
};

}  // namespace bytertc

