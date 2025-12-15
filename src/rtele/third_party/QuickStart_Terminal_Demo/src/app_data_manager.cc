#include "app_data_manager.h"
#include "json11.hpp"
#include "util/util.h"
#include <functional>
#include <regex>

const std::string strNameRegex = "^[a-zA-Z0-9@._-]{1,128}$";
const std::string strDefaultConfig = "config.json";

AppDataManager::AppDataManager() {

}

AppDataManager::~AppDataManager() {

}

bool AppDataManager::parse(const std::string &json_str) {
	std::string err;
	auto configJson = json11::Json::parse(json_str, err);
	if (configJson.is_null()) {
		LOG_WARN("parser json failed, reason:" << err << " str:" << json_str);
		return false;
	}

	auto checkStr = [&](const std::string &data)->bool {
		if (data.empty()) {
			return false;
		}
		std::regex exp(strNameRegex);
		std::smatch base_match;
		auto bRet = std::regex_match(data,base_match,exp);
		return bRet;
	};
	
	m_appData = std::make_shared<StuAppData>();
	m_appData->app_id = configJson["app_id"].string_value();
	m_appData->room_id = configJson["room_id"].string_value();
	m_appData->user_id = configJson["user_id"].string_value();
	m_appData->rtc_env = configJson["rtc_env"].int_value();

	if (m_appData->app_id.empty()) {
		LOG_WARN("appid is empty!");
		return false;
	}

	if (!checkStr(m_appData->room_id)) {
		LOG_WARN("roomid is invalid!");
		return false;
	}

	if (!checkStr(m_appData->user_id)) {
		LOG_WARN("userid is invalid!");
		return false;
	}

	m_appData->app_key = configJson["app_key"].string_value();
	m_appData->enable_video = configJson["enable_video"].bool_value();
	m_appData->enable_external_audio = configJson["enable_external_audio"].bool_value();
	m_appData->enable_external_video = configJson["enable_external_video"].bool_value();
	m_appData->audio_file = configJson["audio_file"].string_value();
	m_appData->video_file = configJson["video_file"].string_value();

	auto videoCaptureConfigJson = configJson["video_capture_config"];
	if (!videoCaptureConfigJson.is_null()) {
		auto videoCapureConfig = std::make_shared<StuVideoCaptureConfig>();
		videoCapureConfig->width = videoCaptureConfigJson["width"].int_value();
		videoCapureConfig->height = videoCaptureConfigJson["height"].int_value();
		videoCapureConfig->fps = videoCaptureConfigJson["fps"].int_value();
		m_appData->video_capture_config = videoCapureConfig;
	}else {
		LOG_WARN("video_capture_config obj is null");
	}
	
	auto videoEncoderConfigJson = configJson["video_encoder_config"];
	if (!videoEncoderConfigJson.is_null()) {
		auto videoEncoderConfig = std::make_shared<StuVideoEncoderConfig>();
		videoEncoderConfig->width = videoEncoderConfigJson["width"].int_value();
		videoEncoderConfig->height = videoEncoderConfigJson["height"].int_value();
		videoEncoderConfig->fps = videoEncoderConfigJson["fps"].int_value();
		videoEncoderConfig->max_bitrate = videoEncoderConfigJson["max_bitrate"].int_value();
		m_appData->video_encoder_config = videoEncoderConfig;
	}
	else {
		LOG_WARN("video_capture_config obj is null");
	}

	m_appData->video_device_index = configJson["video_device_index"].int_value();
	LOG_INFO("video-device=" << m_appData->video_device_index);
	return true;
}

bool AppDataManager::load(const std::string &json_file) 
{
	auto data = bytertc::readFile(json_file.c_str(),"rt");
	if (data.empty()) {
		LOG_ERROR("json file:" << json_file << "is null");
		return false;
	}
	std::string str(reinterpret_cast<const char*>(data.data()));
	int  leftIndex = str.find_first_of("{");
	int rightIndex = str.find_last_of("}");
	if (leftIndex != -1 && rightIndex != -1) {
		str = str.substr(leftIndex, rightIndex - leftIndex + 1);
	}
	if (!parse(str)) {
		return false;
	}
	return true;
}

AppDataManager *AppDataManager::instance() {
	static AppDataManager appDataMgr;
	return &appDataMgr;
} 