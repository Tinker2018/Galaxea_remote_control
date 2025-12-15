# RTele 实现说明

## 功能完成情况

✅ **已完成的功能**:

1. **RTC 初始化** - 集成真实的 VolcEngine RTC SDK
   - SDK 版本检测
   - 引擎创建和配置
   - 房间管理

2. **推流控制** (`--enable_publish` 标志)
   - 视频编码参数配置
   - 外部视频源设置 (`setVideoSourceType`)
   - 视频流推送 (`publishStreamVideo`)
   - 音频流推送 (`publishStreamAudio`)
   - 独立推流线程 (控制帧率)

3. **视频文件推流** (`--video_file` 参数)
   - YUV I420 格式文件读取
   - 循环播放支持
   - VideoFrameData 构造和推送
   - 如果未指定文件，生成黑色测试帧

4. **远端流订阅**
   - 根据 `subscribe_user_id` 选择性订阅
   - 自动订阅逻辑 (onUserPublishStreamVideo 回调)
   - 用户离开时清理订阅状态
   - 使用 `subscribeStreamVideo` API

5. **消息监听线程**
   - 独立线程监听房间消息
   - `onRoomMessageReceived` 回调处理
   - 线程生命周期管理

## 架构设计

```
apps/rtele/src/
├── main.cc                    # 主程序 (信号处理、配置、启动流程)
├── config/                    # 配置模块
│   ├── rtc_config.h/.cc      # RTCConfig 结构 (包含所有参数)
│   └── BUILD.bazel
├── rtc/                       # RTC 核心模块
│   ├── rtc_engine.h/.cc      # RTCEngine (SDK 封装、回调处理)
│   ├── message_listener.h/.cc # 消息监听线程
│   └── BUILD.bazel
└── util/                      # 工具模块
    ├── token_generator.h/.cc  # Token 生成 (HMAC-SHA256)
    ├── video_file_reader.h/.cc # YUV 文件读取
    └── BUILD.bazel

third_party/VolcEngineRTC_Linux_x86_64/  # VolcEngine RTC SDK
└── BUILD.bazel                          # SDK Bazel 配置
```

## 核心 API 使用

### 初始化和加入房间
```cpp
// 1. 创建引擎
bytertc::EngineConfig config;
config.app_id = app_id;
rtc_engine_ = bytertc::IRTCEngine::createRTCEngine(config, this);

// 2. 创建房间
rtc_room_ = rtc_engine_->createRTCRoom(room_id);
rtc_room_->setRTCRoomEventHandler(this);

// 3. 加入房间
bytertc::UserInfo user_info;
user_info.uid = user_id;
bytertc::RTCRoomConfig room_config;
room_config.is_auto_subscribe_audio = true;
room_config.is_auto_subscribe_video = false;  // 手动订阅

std::string token = TokenGenerator::Generate(...);
rtc_room_->joinRoom(token, user_info, true, room_config);
```

### 推流

### 推流
```cpp
// 1. 设置视频编码参数
bytertc::VideoEncoderConfig encoder_config;
encoder_config.width = 1280;
encoder_config.height = 720;
encoder_config.frame_rate = 30;
encoder_config.max_bitrate = 3000;
rtc_engine_->setVideoEncoderConfig(encoder_config);

// 2. 设置外部视频源
rtc_engine_->setVideoSourceType(bytertc::kVideoSourceTypeExternal);

// 3. 发布流
rtc_room_->publishStreamVideo(true);
rtc_room_->publishStreamAudio(true);

// 4. 推送视频帧 (在独立线程中循环)
bytertc::VideoFrameData video_frame;
video_frame.pixel_format = bytertc::kVideoPixelFormatI420;
video_frame.width = width;
video_frame.height = height;
video_frame.number_of_planes = 3;
video_frame.plane_data[0] = y_data;  // Y 平面
video_frame.plane_data[1] = u_data;  // U 平面
video_frame.plane_data[2] = v_data;  // V 平面
video_frame.plane_stride[0] = width;
video_frame.plane_stride[1] = width >> 1;
video_frame.plane_stride[2] = width >> 1;
video_frame.timestamp_us = current_time_us;

rtc_engine_->pushExternalVideoFrame(video_frame);
```

### 订阅
```cpp
// 在 onUserPublishStreamVideo 回调中
void onUserPublishStreamVideo(const char* stream_id, 
                              const StreamInfo& stream_info, 
                              bool is_publish) {
    if (is_publish && ShouldSubscribe(stream_info.user_id)) {
        std::string stream_id = user_id + "_main";
        rtc_room_->subscribeStreamVideo(stream_id.c_str(), true);
    }
}
```

## 命令行参数

### 必需参数
- `--room_id`: 房间 ID
- `--user_id`: 用户 ID  
- `--app_id`: VolcEngine App ID
- `--app_key`: VolcEngine App Key

### 可选参数
- `--enable_publish`: 是否允许推流 (默认: true)
- `--enable_video`: 是否启用视频 (默认: true)
- `--enable_audio`: 是否启用音频 (默认: false)
- `--subscribe_room_id`: 订阅的房间 ID (默认: 与 room_id 相同)
- `--subscribe_user_id`: 订阅的用户 ID (默认: 所有用户)
- `--video_file`: YUV 视频文件路径 (默认: 生成测试图案)

## 使用示例

### 构建
```bash
cd /home/eai/hexiaonan/k8s4r/apps/rtele
bazelisk build //src:rtele
```

### 示例 1: 推送视频文件
```bash
bazelisk run //src:rtele -- \
  --room_id=room001 \
  --user_id=robot001 \
  --app_id=your_app_id \
  --app_key=your_app_key \
  --video_file=/path/to/video.yuv
```

### 示例 2: 仅订阅模式
```bash
bazelisk run //src:rtele -- \
  --room_id=room001 \
  --user_id=operator001 \
  --app_id=your_app_id \
  --app_key=your_app_key \
  --enable_publish=false \
  --subscribe_user_id=robot001
```

### 示例 3: 推送测试图案并订阅
```bash
bazelisk run //src:rtele -- \
  --room_id=room001 \
  --user_id=user001 \
  --app_id=your_app_id \
  --app_key=your_app_key \
  --enable_publish=true
```

## 已知问题和TODO

### 待完善功能
1. **视频渲染** - 当前只订阅不渲染
   - 需要实现 IVideoSink 接口
   - 添加 SDL2 或其他渲染库
   
2. **Token 生成** - 当前为简化实现
   - 建议使用 VolcEngine 官方 TokenGenerator
   
3. **错误处理** - 需要更健壮的错误恢复
   - 网络断线重连
   - 推流失败重试
   
4. **性能优化**
   - 零拷贝视频帧传输
   - 内存池管理
   
5. **音频支持** - 当前未实现音频采集
   - 添加音频文件读取
   - 实现 pushExternalAudioFrame

### 参考资料
- QuickStart_Terminal_Demo: 推流和订阅的参考实现
- VolcEngine RTC SDK 文档: API 详细说明
- bytertc_video_frame.h: VideoFrameData 结构定义
- bytertc_room.h: 房间管理 API

## 信号处理

程序支持优雅退出:
- `Ctrl+C` (SIGINT) - 触发清理流程
- `SIGTERM` - 触发清理流程

清理流程:
1. 停止消息监听线程
2. 停止推流线程  
3. 取消发布流
4. 离开房间
5. 销毁 RTC 引擎

