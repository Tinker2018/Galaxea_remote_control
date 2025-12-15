/*
 * Copyright (c) 2021 The VolcEngineRTC project authors. All Rights Reserved.
 * @brief VolcEngineRTC Video Processor Interface
*/

#pragma once

#include "bytertc_video_defines.h"

namespace bytertc {
/**
 * @locale zh
 * @type keytype
 * @brief WTN 媒体流状态
 */
/**
 * @locale en
 * @type keytype
 * @brief State for WTN streams
 */
enum WTNSubscribeState {
    /**
     * @locale zh
     * @brief 订阅 WTN 媒体流
     */
    /**
     * @locale en
     * @brief Subscribe WTN media stream
     */
    kWTNSubscribeStateSubscribed,
    /**
     * @locale zh
     * @brief 取消订阅 WTN 媒体流
     */
    /**
     * @locale en
     * @brief Unsubscribe WTN media stream
     */
    kWTNSubscribeStateUnsubscribed
};
/**
 * @locale zh
 * @type keytype
 * @brief WTN 媒体流订阅状态改变的原因
 */
/**
 * @locale en
 * @type keytype
 * @brief Reasons for change in WTN media stream's Subscription Status
 */
enum WTNSubscribeStateChangeReason {
    /**
     * @locale zh
     * @brief 订阅 WTN 媒体流成功
     */
    /**
     * @locale en
     * @brief Subscribe WTN media stream successfully
     */
    kWTNSubscribeStateChangeReasonSubscribe = 0,
    /**
     * @locale zh
     * @brief 其他原因订阅失败
     */
    /**
     * @locale en
     * @brief Failed to subscribe WTN media stream for other reasons.
     */
    kWTNSubscribeStateChangeReasonUnsubscribe = 1300,
    /**
     * @locale zh
     * @brief 订阅失败，拉流时远端未发布
     */
    /**
     * @locale en
     * @brief Failed to subscribe because the remote end did not publish.
     */
    kWTNSubscribeStateChangeReasonRemoteUnpublish = 1301, 
    /**
     * @locale zh
     * @brief 订阅失败，超出单端订阅上限。一个引擎实例最多拉 5 路流。
     */
    /**
     * @locale en
     * @brief Failed to subscribe because the single-end subscription limit has been exceeded. An engine instance can pull a maximum of 5 streams.
     */
    kWTNSubscribeStateChangeReasonOverClientSubscribeStreamLimit = 1310,  
    /**
     * @locale zh
     * @brief 订阅失败。超出单流订阅人数上限。该限制由 RTC 服务端决定。
     */
    /**
     * @locale en
     * @brief Failed to subscribe because the single-stream subscription limit has been exceeded. The limit is set by RTC server.
     */
    kWTNSubscribeStateChangeReasonOverStreamSubscribeUserLimit = 1311,  
    /**
     * @locale zh
     * @brief 订阅失败。超出单流订阅请求 QPS 上限。该限制由 RTC 服务端决定。
     */
    /**
     * @locale en
    * @brief Failed to subscribe because the limit of the single-stream subscription request has been exceeded. The limit is set by RTC server.
     */
    kWTNSubscribeStateChangeReasonOverStreamSubscribeRequestLimit = 1312 
};

/**
 * @locale zh
 * @type callback
 * @brief WTN 事件回调接口。 <br>
 * 注意：回调函数是在 SDK 内部线程（非 UI 线程）同步抛出来的，请不要做耗时操作或直接操作 UI，否则可能导致 app 崩溃。
 */
/**
 * @locale en
 * @type callback
 * @brief WTN event handler. <br>
 * Note: Callback functions are thrown synchronously in a non-UI thread within the SDK. Therefore, you must not perform any time-consuming operations or direct UI operations within the callback function, as this may cause the app to crash.
 */
class IWTNStreamEventHandler {
public:
    /**
     * @hidden Constructor/Destructor
     * @brief Destructor
     */
    virtual ~IWTNStreamEventHandler() {
    }
    /**
     * @locale zh
     * @type callback
     * @valid since 3.60.
     * @brief 通话中本地设备接收订阅的远端 WTN 视频流的流 ID 以及远端 WTN 视频流统计信息。
     * @param stream_id WTN 流 ID
     * @param stats 远端 WTN 视频流的统计信息，详见 RemoteVideoStats{@link #RemoteVideoStats}。
     * @list WTN
     * @order 0
     */
    /**
     * @locale en
     * @type callback
     * @valid since 3.60.
     * @brief Remote WTN video stream ID and statistics. <br>
     * @param stream_id ID of the WTN stream
     * @param stats Remote WTN video stream statistics. See RemoteVideoStats{@link #RemoteVideoStats}.
     * @list WTN
     * @order 0
     */
    virtual void onWTNRemoteVideoStats(const char* stream_id, RemoteVideoStats stats) {
        (void)stream_id;
        (void)stats;
    }
    /**
     * @locale zh
     * @type callback
     * @valid since 3.60.
     * @brief 通话中本地设备接收订阅的远端 WTN 音频流的流 ID 以及远端 WTN 音频流统计信息。
     * @param stream_id WTN 流 ID
     * @param stats 远端 WTN 音频流的统计信息，详见 RemoteAudioStats{@link #RemoteAudioStats}
     * @list WTN
     * @order 1
     */
    /**
     * @locale en
     * @type callback
     * @valid since 3.60.
     * @brief Remote WTN audio stream ID and statistics. <br>
     * @param stream_id ID of the WTN stream
     * @param stats Remote WTN audio stream statistics. See RemoteAudioStats{@link #RemoteAudioStats}
     * @list WTN
     * @order 1
     */
    virtual void onWTNRemoteAudioStats(const char* stream_id, RemoteAudioStats stats) {
        (void)stream_id;
        (void)stats;
    }

    /**
     * @locale zh
     * @type callback
     * @valid since 3.60. 自 3.60 起，该回调替代了 `onPlayPublicStreamResult` 方法中的 WTN 视频流订阅状态变化通知功能。如果你已升级至 3.60 及以上版本，并且仍在使用该方法，请迁移到此回调。
     * @brief WTN 视频流订阅状态变化回调 <br>
     *        通过 subscribeWTNVideoStream{@link #IWTNStream#subscribeWTNVideoStream} 订阅 WTN 视频流后，可以通过本回调获取订阅结果。
     * @param stream_id WTN 视频流的 ID
     * @param state_code 视频流状态码，参看 WTNSubscribeState{@link #WTNSubscribeState}。
     * @param reason 订阅状态发生变化的原因，参看 WTNSubscribeStateChangeReason{@link #WTNSubscribeStateChangeReason}。
     * @list WTN
     * @order 2
     */
    /**
     * @locale en
     * @type callback
     * @valid since 3.60. Since version 3.60, this callback replaces `onPlayPublicStreamResult` for reporting the subscription status changes of WTN video streams. If you have upgraded to version 3.60 or later and are still using this method, please migrate to this callback.
     * @brief Callback for the change of WTN video stream's subscription status<br>
     *        You will be informed of the result of subscribing to the WTN video stream by this callback after calling subscribeWTNVideoStream{@link #IWTNStream#subscribeWTNVideoStream}.
     * @param stream_id ID of the WTN video stream
     * @param state_code video stream state code. See See WTNSubscribeState{@link #WTNSubscribeState} for specific indications.
     * @param reason The reason why subscription state changes. See WTNSubscribeStateChangeReason{@link #WTNSubscribeStateChangeReason}.
     * @list WTN
     * @order 2
     */
    virtual void onWTNVideoSubscribeStateChanged(const char* stream_id, WTNSubscribeState state_code, WTNSubscribeStateChangeReason reason) {
        (void)stream_id;
        (void)state_code;
        (void)reason;
    }

    /**
     * @locale zh
     * @type callback
     * @valid since 3.60. 自 3.60 起，该回调替代了 `onPlayPublicStreamResult` 中的 WTN 音频流订阅状态变化通知功能。如果你已升级至 3.60 及以上版本，并且仍在使用该方法，请迁移到此回调。
     * @brief WTN 音频流订阅状态变化回调 <br>
     *        通过 subscribeWTNAudioStream{@link #IWTNStream#subscribeWTNAudioStream} 订阅 WTN 音频流后，可以通过本回调获取订阅结果。
     * @param stream_id WTN 音频流的 ID
     * @param state_code 音频流状态码，参看 WTNSubscribeState{@link #WTNSubscribeState}。
     * @param reason 订阅状态发生变化的原因，参看 WTNSubscribeStateChangeReason{@link #WTNSubscribeStateChangeReason}。
     * @list WTN
     * @order 2
     */
    /**
     * @locale en
     * @type callback
     * @valid since 3.60. Since version 3.60, this callback replaces `onPlayPublicStreamResult` for reporting the subscription status changes of WTN audio streams. If you have upgraded to version 3.60 or later and are still using this method, please migrate to this callback.
     * @brief Callback for the change of WTN audio stream's subscription status<br>
     *        You will be informed of the result of subscribing to the WTN audio stream by this callback after calling subscribeWTNAudioStream{@link #IWTNStream#subscribeWTNAudioStream}.
     * @param stream_id ID of the WTN audio stream
     * @param state_code Audio stream state code. See See WTNSubscribeState{@link #WTNSubscribeState} for specific indications.
     * @param reason The reason why subscription state changes. See WTNSubscribeStateChangeReason{@link #WTNSubscribeStateChangeReason}.
     * @list WTN
     * @order 2
     */
    virtual void onWTNAudioSubscribeStateChanged(const char* stream_id, WTNSubscribeState state_code, WTNSubscribeStateChangeReason reason) {
        (void)stream_id;
        (void)state_code;
        (void)reason;
    }

    /**
     * @locale zh
     * @type callback
     * @valid since 3.60. 自 3.60 起，该回调替代了 `onFirstPublicStreamAudioFrame`。如果你已升级至 3.60 及以上版本，并且仍在使用该方法，请迁移到此回调。
     * @brief WTN 流的首帧音频解码成功 <br>
     *        关于订阅 WTN 音频流，详见 subscribeWTNAudioStream{@link #IWTNStream#subscribeWTNAudioStream}。
     * @param stream_id WTN 流 ID
     * @list WTN
     * @order 3
     */
    /**
     * @locale en
     * @type callback
     * @valid since 3.60. Since version 3.60, this callback replaces `onFirstPublicStreamAudioFrame`. If you have upgraded to version 3.60 or later and are still using this method, please migrate to this callback.
     * @brief Callback of successfully decoding of the first audio frame of the WTN stream <br>
     *        Refer to subscribeWTNAudioStream{@link #IWTNStream#subscribeWTNAudioStream} for details about subscribing to a WTN audio stream.
     * @param stream_id ID of the WTN stream
     * @list WTN
     * @order 3
     */
    virtual void onWTNFirstRemoteAudioFrame(const char* stream_id) {
        (void)stream_id;
    }

    /**
     * @locale zh
     * @type callback
     * @valid since 3.60. 自 3.60 起，该回调替代了 `onFirstPublicStreamVideoFrameDecoded`。如果你已升级至 3.60 及以上版本，并且仍在使用该方法，请迁移到此回调。
     * @brief WTN 流的首帧视频解码成功 <br>
     *        关于 订阅 WTN 流，详见 subscribeWTNVideoStream{@link #IWTNStream#subscribeWTNVideoStream}。
     * @param stream_id WTN 流 ID
     * @param info 视频帧信息。详见 VideoFrameInfo{@link #VideoFrameInfo}。
     * @list WTN
     * @order 4
     */
    /**
     * @locale en
     * @type callback
     * @valid since 3.60. Since version 3.60, this callback replaces `onFirstPublicStreamVideoFrameDecoded`. If you have upgraded to version 3.60 or later and are still using this method, please migrate to this callback.
     * @brief Callback of successfully decoding of the first video frame of the WTN stream <br>
     *        Refer to subscribeWTNVideoStream{@link #IWTNStream#subscribeWTNVideoStream} for details about subscribing to a WTN video stream.
     * @param stream_id ID of the WTN stream
     * @param info Information of the video stream. Refer to VideoFrameInfo{@link #VideoFrameInfo} for more details.
     * @list WTN
     * @order 4
     */
    virtual void onWTNFirstRemoteVideoFrameDecoded(const char* stream_id, const VideoFrameInfo& info) {
        (void)stream_id;
        (void)info;
    }

    /**
     * @locale zh
     * @type callback
     * @valid since 3.60. 自 3.60 起，该回调替换了 `onPublicStreamSEIMessageReceived` 来实现下述功能。如果你已升级至 3.60 及以上版本，并且仍在使用该方法，请迁移到此回调。
     * @brief 回调 WTN 流中包含的 SEI 信息。 <br>
     *        调用 subscribeWTNAudioStream{@link #IWTNStream#subscribeWTNAudioStream}/subscribeWTNVideoStream{@link #IWTNStream#subscribeWTNVideoStream}接口拉 WTN 音频流/视频流后，通过此回调收到 WTN 流中的 SEI 消息。
     * @param stream_id WTN 流 ID。
     * @param channel_id SEI 的消息传输通道，取值范围 `[0 - 255]`。通过此参数，你可以为不同接受方设置不同的 ChannelID，这样不同接收方可以根据回调中的 ChannelID 选择应关注的 SEI 信息。
     * @param message 收到的 SEI 消息内容。 <br>
     *                通过调用客户端 `sendSEIMessage` 插入的 SEI 信息。 <br>
     *                当 WTN 流中的多路视频流均包含有 SEI 信息：SEI 不互相冲突时，将通过多次回调分别发送；SEI 在同一帧有冲突时，则只有一条流中的 SEI 信息被透传并融合到 WTN 流中。
     * @param length SEI 信息的长度。
     * @note 通过 Open API 插入的自定义信息，应通过回调 onWTNDataMessageReceived{@link #IWTNStreamEventHandler#onWTNDataMessageReceived} 获取。
     * @list WTN
     * @order 5
     */
    /**
     * @locale en
     * @type callback
     * @valid since 3.60. Since version 3.60, this callback replaces `onPublicStreamSEIMessageReceived` for the following function. If you have upgraded to version 3.60 or later and are still using this method, please migrate to this callback.
     * @brief Callback on receiving the SEI message carried by the WTN stream. <br>
     *        After calling subscribeWTNAudioStream{@link #IWTNStream#subscribeWTNAudioStream}/subscribeWTNVideoStream{@link #IWTNStream#subscribeWTNVideoStream}, you will receive this callback if the WTN stream has an SEI message.
     * @param stream_id The ID of the WTN stream.
     * @param channel_id SEI message channel ID. The value range is `[0 - 255]`. With this parameter, you can set different ChannelIDs for different recipients. In this way, different recipients can choose the SEI information  based on the ChannelID received in the callback.
     * @param message The received SEI(supplemental enhancement information) message. <br>
     *                The SEI message inserted by calling `sendSEIMessage` in the SDK. <br>
     *                You receive SEI from all the video streams if the SEI messages do not have conflicts. However, if the SEI messages from different video streams have conflicts, you will receive only one of them.
     * @param length The length of the SEI message.
     * @note You also need to listen to onWTNDataMessageReceived{@link #IWTNStreamEventHandler#onWTNDataMessageReceived} to receive data messages inserted via Open API in the server.
     */
    virtual void onWTNSEIMessageReceived(const char* stream_id, int channel_id, const uint8_t* message, int length) {
        (void)stream_id;
        (void)channel_id;
        (void)message;
        (void)length;
    }

    /**
     * @locale zh
     * @type callback
     * @valid since 3.60.
     * @brief 回调 WTN 流中包含的数据信息。 <br>
     *        通过 subscribeWTNAudioStream{@link #IWTNStream#subscribeWTNAudioStream}/subscribeWTNVideoStream{@link #IWTNStream#subscribeWTNVideoStream} 订阅 WTN 流后，通过监听本回调获取 WTN 流中的数据消息，包括调用 Open API 发送的 SEI 消息和音量回调。
     * @param stream_id  WTN 流 ID
     * @param message 收到的数据消息内容，如下： <br>
     * - 调用 WTN 流 OpenAPI 发送的自定义消息。
     * - 媒体流音量变化，需要通过 WTN 流 OpenAPI 开启回调。JSON 格式说明如下：<br/>
     * {<br/>
     * "Type" : "VolumeIndication", //具体业务类型<br/>
     * "VolumeInfos" : [ // 业务类型对应信息<br/>
     * {<br/>
     * "RoomId":"1000001", // 房间 ID<br/>
     * "UserId":"1000001", // 用户 ID<br/>
     * "StreamType":0, // 0:摄像头流；1:屏幕流<br/>
     * "LinearVolume":1 // 线性音量大小<br/>
     * }<br/>
     * @param message_length 消息的长度
     * @param source_type 数据消息来源，参看 DataMessageSourceType{@link #DataMessageSourceType}。
     * @note 通过调用客户端 API 插入的 SEI 信息，应通过回调 onWTNSEIMessageReceived{@link #IWTNStreamEventHandler#onWTNSEIMessageReceived} 获取。
     * @list WTN
     * @order 6
     */
    /**
     * @locale en
     * @type callback
     * @valid since 3.60.
     * @brief The data information contained in the callback of WTN stream.. <br>
     *        After calling subscribeWTNAudioStream{@link #IWTNStream#subscribeWTNAudioStream}/subscribeWTNVideoStream{@link #IWTNStream#subscribeWTNVideoStream}, you can listen to this callback and retrieve SEI messages inserted by Open API calls or audio volume.
     * @param stream_id ID of the WTN stream
     * @param message The received data message's content is as follows: <br>
     * - Data inserted by calling the OpenAPI.
     * - Media volume in JSON, before you can receive which you need to enable it via calling the OpenAPI. Refer to the definition below to parse the JSON string.
     * {<br/>
     * "Type" : "VolumeIndication", //Specific business type.<br/>
     * "VolumeInfos" : [ // Business type corresponding information<br/>
     * {<br/>
     * "RoomId":"1000001", // Room ID<br/>
     * "UserId":"1000001", // User ID<br/>
     * "StreamType":0, // 0:camera stream；1:screen stream<br/>
     * "LinearVolume":1 // LinearVolume<br/>
     * }<br/>
     * @param message_length Length of the message
     * @param source_type Message source. See DataMessageSourceType{@link #DataMessageSourceType}.
     * @note You also need to listen to onWTNSEIMessageReceived{@link #IWTNStreamEventHandler#onWTNSEIMessageReceived} to receive SEI inserted via API by the client SDK.
     * @list WTN
     * @order 6
     */
    virtual void onWTNDataMessageReceived(const char* stream_id,
        const uint8_t* message,
        int message_length,
        DataMessageSourceType source_type) {
        (void)stream_id;
        (void)message;
        (void)message_length;
        (void)source_type;
    }
};

/**
 * @locale zh
 * @type api
 * @region WTN
 * @brief WTN 流处理
 */
/**
 * @locale en
 * @type api
 * @region WTN
 * @brief WTN stream process
 */
class IWTNStream
{
public:
    /**
     * @hidden constructor/destructor
     */
    virtual ~IWTNStream() = default;

    /** 
     * @locale zh
     * @type api
     * @valid since 3.60. 自 3.60 起，此接口替代了 `startPlayPublicStream` 和 `stopPlayPublicStream` 方法来订阅/取消订阅指定 WTN 视频流，如果你使用了这两个方法，请迁移至此接口。
     * @brief 订阅/取消订阅指定 WTN 视频流 <br>
     *        无论用户是否在房间内，都可以调用本接口订阅/取消订阅指定的 WTN 音频流。
     * @param stream_id WTN 流 ID，如果指定流暂未发布，则本地客户端将在其开始发布后接收到流数据。
     * @param subscribe 是否订阅 WTN 流 <br>
     *       - true：订阅
     *       - false：取消订阅
     * @return
     *        - 0: 成功。同时将收到 onWTNVideoSubscribeStateChanged{@link #IWTNStreamEventHandler#onWTNVideoSubscribeStateChanged} 回调。
     *        - !0: 失败。当参数不合法或参数为空，调用失败。
     * @note
     *        - 一个客户端最多同时播放 5 路 WTN 流，请及时调用 subscribeWTNVideoStream{@link #IWTNStream#subscribeWTNVideoStream}/subscribeWTNAudioStream{@link #IWTNStream#subscribeWTNAudioStream} 取消订阅 WTN 流，避免订阅的 WTN 流数量超限。
     *        - 在调用本接口之前，建议先绑定渲染视图。
     *              - 调用 setWTNRemoteVideoCanvas{@link #IWTNStream#setWTNRemoteVideoCanvas} 绑定内部渲染视图
     *              - 调用 setWTNRemoteVideoSink{@link #IWTNStream#setWTNRemoteVideoSink} 绑定自定义渲染视图
     *        - 调用本接口后，可以通过 onWTNFirstRemoteVideoFrameDecoded{@link #IWTNStreamEventHandler#onWTNFirstRemoteVideoFrameDecoded} 回调 WTN 视频流的首帧解码情况。
     *        - 调用本接口后，可以通过 onWTNSEIMessageReceived{@link #IWTNStreamEventHandler#onWTNSEIMessageReceived} 回调 WTN 流中包含的 SEI 信息。
     * @list WTN
     * @order 0
     */
    /**
     * @locale en
     * @type api
     * @valid since 3.60. Since version 3.60, this interface replaces the `startPlayPublicStream` and `stopPlayPublicStream` methods for subscribing to and unsubscribing from WTN video streams. If you are using these methods, please migrate to this interface.
     * @brief Subscribe/ unsubscribe WTN video stream <br>
     *        A user can call this method to subscribe or unsubscribe a WTN stream whether he/she has joined the room or not.
     * @param stream_id ID of the WTN stream. If the stream has not been published then, the local client will receive the WTN stream once it starts publishing.
     * @param subscribe Whether to subscribe to the WTN stream. <br>
     *       - true: Subscribe
     *       - false: Unsubscribe
     * @return
     *        - 0: Success. You will also be informed by onWTNVideoSubscribeStateChanged{@link #IWTNStreamEventHandler#onWTNVideoSubscribeStateChanged}.
     *        - !0: Failure because of invalid parameter or empty parameters.
     * @note
     *        - A client can play up to 5 WTN streams simultaneously. Please call subscribeWTNVideoStream{@link #IWTNStream#subscribeWTNVideoStream}/subscribeWTNAudioStream{@link #IWTNStream#subscribeWTNAudioStream} in time to cancel the subscription to WTN streams to avoid exceeding the limit of subscribed WTN streams.
     *        - We recommend to bind a view for the WTN stream before calling this API to subscribe a WTN stream.
     *              - Internal renderer: By calling setWTNRemoteVideoCanvas{@link #IWTNStream#setWTNRemoteVideoCanvas} or
     *              - Custom renderer: By calling setWTNRemoteVideoSink{@link #IWTNStream#setWTNRemoteVideoSink}
     *        - After calling this API, you will be informed once the first frame has been decoded successfully by onWTNFirstRemoteVideoFrameDecoded{@link #IWTNStreamEventHandler#onWTNFirstRemoteVideoFrameDecoded}.
     *        - If the WTN stream contains SEI information, you will be informed by oonWTNSEIMessageReceived{@link #IWTNStreamEventHandler#onWTNSEIMessageReceived}.
     * @list WTN
     * @order 0
     */
    virtual int subscribeWTNVideoStream(const char* stream_id, bool subscribe) = 0;

    /** 
     * @locale zh
     * @type api
     * @valid since 3.60. 自 3.60 起，此接口替代了 `startPlayPublicStream` 和 `stopPlayPublicStream` 方法来订阅/取消订阅指定 WTN 音频流，如果你使用了这两个方法，请迁移至此接口。
     * @brief 订阅/取消订阅指定 WTN 音频流 <br>
     *        无论用户是否在房间内，都可以调用本接口订阅/取消订阅指定的 WTN 音频流。
     * @param stream_id WTN 流 ID，如果指定流暂未发布，则本地客户端将在其开始发布后接收到流数据。
     * @param subscribe 是否订阅 WTN 流 <br>
     *       - true：订阅
     *       - false：取消订阅
     * @return
     *        - 0: 成功。同时将收到 onWTNAudioSubscribeStateChanged{@link #IWTNStreamEventHandler#onWTNAudioSubscribeStateChanged} 回调。
     *        - !0: 失败。当参数不合法或参数为空，调用失败。
     * @note
     *        - 一个客户端最多同时播放 5 路 WTN 流，请及时调用 subscribeWTNVideoStream{@link #IWTNStream#subscribeWTNVideoStream}/subscribeWTNAudioStream{@link #IWTNStream#subscribeWTNAudioStream} 取消订阅WTN 流，避免订阅的 WTN 流数量超限。
     *        - 在调用本接口之前，建议先绑定渲染视图。
     *              - 调用 setWTNRemoteVideoCanvas{@link #IWTNStream#setWTNRemoteVideoCanvas} 绑定内部渲染视图
     *              - 调用 setWTNRemoteVideoSink{@link #IWTNStream#setWTNRemoteVideoSink} 绑定自定义渲染视图
     *        - 调用本接口后，可以通过 onWTNFirstRemoteAudioFrame{@link #IWTNStreamEventHandler#onWTNFirstRemoteAudioFrame} 回调 WTN 音频流的音频首帧解码情况。
     *        - 调用本接口后，可以通过 onWTNSEIMessageReceived{@link #IWTNStreamEventHandler#onWTNSEIMessageReceived} 回调 WTN 流中包含的 SEI 信息。
     * @list WTN
     * @order 1
     */
    /**
     * @locale en
     * @type api
     * @valid since 3.60. Since version 3.60, this interface replaces the `startPlayPublicStream` and `stopPlayPublicStream` methods for subscribing to and unsubscribing from WTN audio streams. If you are using these methods, please migrate to this interface.
     * @brief Subscribe/unsubscribe the WTN audio stream <br>
     *        A user can call this method to subscribe a WTN stream whether he/she has joined the room or not.
     * @param stream_id ID of the WTN stream. If the stream has not been published then, the local client will receive the WTN stream once it starts publishing.
     * @param subscribe Whether to subscribe to the WTN stream. <br>
     *       - true: Subscribe
     *       - false: Unsubscribe
     * @return
     *        - 0: Success. You will also be informed by onWTNAudioSubscribeStateChanged{@link #IWTNStreamEventHandler#onWTNAudioSubscribeStateChanged}.
     *        - !0: Failure because of invalid parameter or empty parameters.
     * @note
     *        - A client can play up to 5 WTN streams simultaneously. Please call subscribeWTNVideoStream{@link #IWTNStream#subscribeWTNVideoStream}/subscribeWTNAudioStream{@link #IWTNStream#subscribeWTNAudioStream} in time to cancel the subscription to WTN streams to avoid exceeding the limit of subscribed WTN streams.
     *        - We recommend to bind a view for the WTN stream before calling this API to subscribe a WTN stream.
     *              - Internal renderer: By calling setWTNRemoteVideoCanvas{@link #IWTNStream#setWTNRemoteVideoCanvas} or
     *              - Custom renderer: By calling setWTNRemoteVideoSink{@link #IWTNStream#setWTNRemoteVideoSink}
     *        - After calling this API, you will be informed once the first frame has been decoded successfully by onWTNFirstRemoteAudioFrame{@link #IWTNStreamEventHandler#onWTNFirstRemoteAudioFrame}.
     *        - If the WTN stream contains SEI information, you will be informed by onWTNSEIMessageReceived{@link #IWTNStreamEventHandler#onWTNSEIMessageReceived}.
     * @list WTN
     * @order 1
     */
    virtual int subscribeWTNAudioStream(const char* stream_id, bool subscribe) = 0;

    /**
     * @locale zh
     * @type api
     * @valid since 3.60. 自 3.60 起，该接口替代了 `setPublicStreamVideoCanvas` 方法来实现下述功能。你已升级至 3.60 及以上版本，并且仍在使用该方法，请迁移到此接口。
     * @brief 为指定 WTN 流绑定内部渲染视图
     * @param stream_id WTN 流 ID
     * @param canvas 内部渲染视图，如果需要解除视频的绑定视图，把 `videoCanvas` 设置为空。详见 VideoCanvas{@link #VideoCanvas}。
     * @return
     *        - 0：成功
     *        - !0：失败
     * @list WTN
     * @order 2
     */
    /**
     * @locale zh
     * @type api
     * @valid since 3.60. Since version 3.60, this interface replaces `setPublicStreamVideoCanvas` for the following function. If you have upgraded to version 3.60 or above and are still using this method, please migrate to this interface.
     * @brief Assign a internal render view to the WTN stream
     * @param stream_id ID of the WTN stream
     * @param canvas Internal render view. If you want to unbind the video, set `videoCanvas` to null. Refer to VideoCanvas{@link #VideoCanvas} for more details.
     * @return
     *        - 0: Success
     *        - !0: Failure
     * @list WTN
     * @order 2
     */
    virtual int setWTNRemoteVideoCanvas(const char* stream_id, const VideoCanvas& canvas) = 0;

    /**
     * @locale zh
     * @type api
     * @valid since 3.60. 自 3.60 起，该接口替代了 `setPublicStreamVideoSink` 方法来实现下述功能。你已升级至 3.60 及以上版本，并且仍在使用该方法，请迁移到此接口。
     * @brief 为指定 WTN 流绑定自定义渲染器。详见[自定义视频渲染](https://www.volcengine.com/docs/6348/81201)。
     * @param stream_id WTN 流 ID
     * @param video_sink 自定义视频渲染器，自定义视频渲染器，需要释放渲染器资源时，将  `videoSink` 设置为 `null`。参看 IVideoSink{@link #IVideoSink}
     * @param config 远端视频帧回调配置，参看 RemoteVideoSinkConfig{@link #RemoteVideoSinkConfig}
     * @return 
     *        - 0：成功；
     *        - <0：失败。具体失败原因参看 ReturnStatus{@link #ReturnStatus}。
     * @list WTN
     * @order 3
     */
    /**
     * @locale en
     * @type api
     * @valid since 3.60. Since version 3.60, this interface replaces `setPublicStreamVideoSink` for the following function. If you have upgraded to version 3.60 or above and are still using this method, please migrate to this interface.
     * @brief Assign a custom renderer to the WTN stream. See [Custom Video Rendering](https://docs.byteplus.com/en/docs/byteplus-rtc/docs-81201)
     * @param stream_id ID of the WTN stream
     * @param video_sink Custom renderer. Set to be `null` when you want to release the renderer. Refer to IVideoSink{@link #IVideoSink} for more details.
     * @param config Remote video frame callback configuration. Refer to RemoteVideoSinkConfig{@link #RemoteVideoSinkConfig} for more details.
     * @return 
     *        - 0: Success.
     *        - <0: Failure. See ReturnStatus{@link #ReturnStatus} for specific reasons.
     * @list WTN
     * @order 3
     */
    virtual int setWTNRemoteVideoSink(const char* stream_id, IVideoSink* video_sink, RemoteVideoSinkConfig& config) = 0;
    
    /**
     * @locale zh
     * @type api
     * @valid since 3.60. 自 3.60 起，该接口替代了 `setPublicStreamAudioPlaybackVolume` 方法来实现下述功能。你已升级至 3.60 及以上版本，并且仍在使用该方法，请迁移到此接口。
     * @brief 调节 WTN 流的音频播放音量。
     * @param stream_id WTN 流 ID
     * @param volume 音频播放音量值和原始音量值的比值，该比值的范围是 `[0, 400]`，单位为 %，且自带溢出保护。为保证更好的音频质量，建议设定在 `[0, 100]` 之间，其中 100 为系统默认值。
     * @return
     *         - 0: 成功调用。
     *         - -2: 参数错误。
     * @list WTN
     * @order 4
     */
    /**
     * @locale en
     * @type api
     * @valid since 3.60. Since version 3.60, this interface replaces `setPublicStreamAudioPlaybackVolume` for the following function. If you have upgraded to version 3.60 or above and are still using this method, please migrate to this interface.
     * @brief Set the audio playback volume of the WTN stream.
     * @param stream_id ID of the WTN stream.
     * @param volume Ratio(%) of the audio playback volume to the original volume, in the range `[0, 400]`, with overflow protection. The default volume is 100.<br>
     *               To ensure the audio quality, the recommended range is `[0,  100]`.
     * @return
     *         - 0: Success.
     *         - -2: Wrong parameter.
     * @list WTN
     * @order 4
     */
    virtual int setWTNRemoteAudioPlaybackVolume(const char* stream_id, int volume) = 0;
    /**
     * @locale zh
     * @type api
     * @valid since 3.60.
     * @brief 设置 WTN 流回调接口
     * @param handler WTN 回调类，参看 IWTNStreamEventHandler{@link #IWTNStreamEventHandler}。
     * @list WTN
     * @order 5
     */
    /**
     * @locale en
     * @type api
     * @valid since 3.60.
     * @brief Set the WTN event handler.
     * @param handler See IWTNStreamEventHandler{@link #IWTNStreamEventHandler}.
     * @list WTN
     * @order 5
     */
    virtual int setWTNStreamEventHandler(IWTNStreamEventHandler* handler) = 0;
};

} // namespace bytertc
