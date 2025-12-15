#include "src/util/video_file_reader.h"
#include <iostream>
#include <sys/stat.h>

namespace rtele {
namespace util {

VideoFileReader::VideoFileReader()
    : width_(0), height_(0), fps_(0), 
      codec_("RAW"), pixel_format_("I420"),
      frame_size_(0), total_frames_(0), current_frame_index_(0) {
}

VideoFileReader::~VideoFileReader() {
    Close();
}

bool VideoFileReader::Open(const std::string& file_path, int width, int height, int fps,
                           const std::string& codec, const std::string& pixel_format) {
    width_ = width;
    height_ = height;
    fps_ = fps;
    codec_ = codec;
    pixel_format_ = pixel_format;
    
    // 当前仅支持 RAW 格式的 YUV 文件
    if (codec_ != "RAW") {
        std::cerr << "Unsupported codec: " << codec_ 
                  << " (currently only RAW YUV is supported)\n";
        return false;
    }
    
    // 根据像素格式计算帧大小
    if (pixel_format_ == "I420") {
        // I420 格式：Y 分量 width*height，U 和 V 各 width*height/4
        frame_size_ = width * height * 3 / 2;
    } else if (pixel_format_ == "NV12") {
        // NV12 格式：Y 分量 width*height，UV 交错 width*height/2
        frame_size_ = width * height * 3 / 2;
    } else if (pixel_format_ == "RGBA") {
        // RGBA 格式：每像素 4 字节
        frame_size_ = width * height * 4;
    } else {
        std::cerr << "Unsupported pixel format: " << pixel_format_ << "\n";
        return false;
    }
    
    file_.open(file_path, std::ios::binary);
    if (!file_.is_open()) {
        std::cerr << "Failed to open video file: " << file_path << "\n";
        return false;
    }
    
    // 获取文件大小
    struct stat st;
    if (stat(file_path.c_str(), &st) == 0) {
        total_frames_ = st.st_size / frame_size_;
        std::cout << "Video file opened: " << file_path << "\n"
                  << "  Resolution: " << width << "x" << height << "\n"
                  << "  FPS: " << fps << "\n"
                  << "  Codec: " << codec_ << "\n"
                  << "  Pixel Format: " << pixel_format_ << "\n"
                  << "  Frame size: " << frame_size_ << " bytes\n"
                  << "  Total frames: " << total_frames_ << "\n";
    }
    
    current_frame_index_ = 0;
    return true;
}

bool VideoFileReader::ReadNextFrame(std::vector<uint8_t>& frame_data) {
    if (!file_.is_open()) {
        return false;
    }
    
    frame_data.resize(frame_size_);
    file_.read(reinterpret_cast<char*>(frame_data.data()), frame_size_);
    
    if (file_.gcount() != static_cast<std::streamsize>(frame_size_)) {
        // 读取失败或到达文件末尾，循环播放
        Reset();
        file_.read(reinterpret_cast<char*>(frame_data.data()), frame_size_);
        if (file_.gcount() != static_cast<std::streamsize>(frame_size_)) {
            return false;
        }
    }
    
    current_frame_index_++;
    if (current_frame_index_ >= total_frames_) {
        current_frame_index_ = 0;
    }
    
    return true;
}

void VideoFileReader::Reset() {
    if (file_.is_open()) {
        file_.clear();
        file_.seekg(0, std::ios::beg);
        current_frame_index_ = 0;
    }
}

void VideoFileReader::Close() {
    if (file_.is_open()) {
        file_.close();
    }
}

}  // namespace util
}  // namespace rtele
