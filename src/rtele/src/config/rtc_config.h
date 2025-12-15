#ifndef RTELE_CONFIG_RTC_CONFIG_H_
#define RTELE_CONFIG_RTC_CONFIG_H_

#include <string>

namespace rtele {
namespace config {

// RTC 配置结构
struct RTCConfig {
    // 基本配置
    std::string app_id;
    std::string app_key;
    std::string room_id;
    std::string user_id;
    std::string token;
    std::string param;
    
    // 推流配置
    bool enable_publish = true;  // 是否启用推流
    bool enable_video = true;    // 是否启用视频
    bool enable_audio = false;   // 是否启用音频
    
    // 订阅配置
    std::string subscribe_room_id;  // 订阅的房间ID（为空则使用room_id）
    std::string subscribe_user_id;  // 订阅的用户ID（为空则订阅所有用户）
    
    // 视频配置（用于推流）
    int video_width = 1280;
    int video_height = 720;
    int video_fps = 30;
    int video_bitrate = 3000;  // kbps
    
    // 视频编码配置
    // 注：video_codec 有两个用途:
    // 1. 通过 BuildParamJSON() 构造 param 传给 RTC SDK ("codec_name":"H264")
    // 2. 提供给 VideoFileReader 等工具进行编解码初始化
    std::string video_codec = "H264";  // 编码格式: H264, H265, VP8, VP9
    int video_max_bitrate = 5000;      // 最大码率 kbps
    int video_min_bitrate = 1000;      // 最小码率 kbps
    
    // 音频配置
    int audio_sample_rate = 48000;     // 采样率: 8000, 16000, 32000, 44100, 48000
    int audio_channels = 2;            // 声道数: 1=单声道, 2=立体声
    int audio_bitrate = 128;           // 音频码率 kbps
    std::string audio_codec = "OPUS";  // 编码格式: OPUS, AAC (用于 param 和音频工具)
    
    // 设备配置
    int video_device_index = -1;  // 视频设备索引，-1 表示自动选择第一个
    int audio_device_index = -1;  // 音频设备索引，-1 表示自动选择第一个
    
    // 视频文件路径（用于推流测试）
    std::string video_file_path;  // 如果为空，使用摄像头采集
    
    // 视频源类型（从文件或设备推流时的像素格式）
    std::string video_pixel_format = "I420";  // I420, NV12, RGBA
    
    // 获取实际的订阅房间ID
    std::string GetSubscribeRoomId() const {
        return subscribe_room_id.empty() ? room_id : subscribe_room_id;
    }
    
    // 验证配置
    bool Validate() const;
    
    // 打印配置
    void Print() const;
    
    // 根据 video_codec 构造 RTC param JSON
    std::string BuildParamJSON() const;
};

}  // namespace config
}  // namespace rtele

#endif  // RTELE_CONFIG_RTC_CONFIG_H_
