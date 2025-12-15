#ifndef RTELE_UTIL_VIDEO_FILE_READER_H_
#define RTELE_UTIL_VIDEO_FILE_READER_H_

#include <string>
#include <vector>
#include <fstream>

namespace rtele {
namespace util {

// 视频文件读取器
// 支持多种视频格式的读取
class VideoFileReader {
public:
    VideoFileReader();
    ~VideoFileReader();
    
    // 打开视频文件
    // codec: 编码格式 ("RAW", "H264", "H265", "VP8", "VP9")
    // pixel_format: 像素格式 ("I420", "NV12", "RGBA")
    // 当前仅支持 codec="RAW", pixel_format="I420" 的 YUV 文件
    bool Open(const std::string& file_path, int width, int height, int fps,
              const std::string& codec = "RAW", 
              const std::string& pixel_format = "I420");
    
    // 读取下一帧
    // 返回值：frame_data 指向帧数据，返回 true 表示成功
    bool ReadNextFrame(std::vector<uint8_t>& frame_data);
    
    // 重置到文件开头（循环播放）
    void Reset();
    
    // 关闭文件
    void Close();
    
    // 获取帧大小
    size_t GetFrameSize() const { return frame_size_; }
    
    // 获取总帧数
    int GetTotalFrames() const { return total_frames_; }
    
    // 获取当前帧索引
    int GetCurrentFrameIndex() const { return current_frame_index_; }

private:
    std::ifstream file_;
    int width_;
    int height_;
    int fps_;
    std::string codec_;         // 编码格式
    std::string pixel_format_;  // 像素格式
    size_t frame_size_;         // 帧大小（根据 pixel_format 计算）
    int total_frames_;
    int current_frame_index_;
};

}  // namespace util
}  // namespace rtele

#endif  // RTELE_UTIL_VIDEO_FILE_READER_H_
