#include<memory>
#include<string>

struct StuVideoCaptureConfig 
{
	int width = 1280;
	int height = 720;
	int fps = 30;
};

struct StuVideoEncoderConfig
{
	int width = 1280;
	int height = 720;
	int fps = 30;
	int max_bitrate = 3000;
};

struct StuAppData 
{
	std::string app_id;
	std::string app_key;
	std::string room_id;
	std::string user_id;
	int rtc_env = 0;
	bool enable_video;
	bool enable_external_audio;
	bool enable_external_video;
	std::string audio_file;
	std::string video_file;
	std::shared_ptr<StuVideoCaptureConfig> video_capture_config;
	std::shared_ptr<StuVideoEncoderConfig> video_encoder_config;
	int video_device_index = 0;
};

class AppDataManager {
public:
	AppDataManager();
	~AppDataManager();
	bool parse(const std::string &json_str);
	bool load(const std::string &json_file);
	std::shared_ptr<StuAppData> getAppData() { return m_appData; }
public:
	static AppDataManager *instance();
private:
	std::shared_ptr<StuAppData> m_appData;
};