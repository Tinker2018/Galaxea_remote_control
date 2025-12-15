#include "src/config/rtc_config.h"
#include "absl/strings/str_format.h"
#include "spdlog/spdlog.h"

namespace rtele {
namespace config {

bool RTCConfig::Validate() const {
    if (app_id.empty()) {
        spdlog::error("Error: app_id is required");
        return false;
    }
    if (app_key.empty()) {
        spdlog::error("Error: app_key is required");
        return false;
    }
    if (room_id.empty()) {
        spdlog::error("Error: room_id is required");
        return false;
    }
    if (user_id.empty()) {
        spdlog::error("Error: user_id is required");
        return false;
    }
    return true;
}

void RTCConfig::Print() const {
    spdlog::info("=================================================");
    spdlog::info("RTele - RTC Configuration");
    spdlog::info("=================================================");
    spdlog::info("App ID         : {}", app_id);
    spdlog::info("App Key        : {}...", app_key.substr(0, 8));
    spdlog::info("Room ID        : {}", room_id);
    spdlog::info("User ID        : {}", user_id);
    spdlog::info("-------------------------------------------------");
    spdlog::info("Enable Video   : {}", enable_video ? "true" : "false");
    spdlog::info("Enable Audio   : {}", enable_audio ? "true" : "false");
    
    if (enable_video) {
        spdlog::info("-------------------------------------------------");
        spdlog::info("Video Configuration:");
        spdlog::info("  Resolution   : {}x{} @ {}fps",
                     video_width, video_height, video_fps);
        spdlog::info("  Codec        : {}", video_codec);
        spdlog::info("  Bitrate      : {} kbps (range: {}-{} kbps)",
                     video_bitrate, video_min_bitrate, video_max_bitrate);
        spdlog::info("  Pixel Format : {}", video_pixel_format);
        spdlog::info("  Device Index : {} {}", 
                     video_device_index,
                     video_device_index == -1 ? "(auto-select)" : "");
        if (!video_file_path.empty()) {
            spdlog::info("  Video File   : {}", video_file_path);
        }
    }
    
    if (enable_audio) {
        spdlog::info("-------------------------------------------------");
        spdlog::info("Audio Configuration:");
        spdlog::info("  Sample Rate  : {} Hz", audio_sample_rate);
        spdlog::info("  Channels     : {} ({})", 
                     audio_channels,
                     audio_channels == 1 ? "mono" : "stereo");
        spdlog::info("  Codec        : {}", audio_codec);
        spdlog::info("  Bitrate      : {} kbps", audio_bitrate);
        spdlog::info("  Device Index : {} {}", 
                     audio_device_index,
                     audio_device_index == -1 ? "(auto-select)" : "");
    }
    
    spdlog::info("-------------------------------------------------");
    spdlog::info("Subscribe Room : {}", 
                 subscribe_room_id.empty() ? "<same as room_id>" : subscribe_room_id);
    spdlog::info("Subscribe User : {}", 
                 subscribe_user_id.empty() ? "<all users>" : subscribe_user_id);
    spdlog::info("=================================================");
}

std::string RTCConfig::BuildParamJSON() const {
    // 根据 video_codec 构造 RTC SDK 的 param JSON
    // 参考 QuickStart_Terminal_Demo 的实现
    
    std::string codec_name = "auto";  // 默认自动选择
    
    // 将用户指定的 codec 转换为 SDK 识别的格式
    if (video_codec == "H264") {
        codec_name = "H264";
    } else if (video_codec == "H265" || video_codec == "HEVC") {
        codec_name = "H265";
    } else if (video_codec == "VP8") {
        codec_name = "VP8";
    } else if (video_codec == "VP9") {
        codec_name = "VP9";
    }
    codec_name = "auto";
    
    // 构造 JSON 字符串
    // 使用硬件编码以提高性能
    std::string param = "{\n"
        "    \"rtc.video_encoder\":{\n"
        "        \"codec_name\":\"" + codec_name + "\",\n"
        "        \"codec_mode\":\"hardware\"\n"
        "    }\n"
        "}";
    
    return param;
}

}  // namespace config
}  // namespace rtele
