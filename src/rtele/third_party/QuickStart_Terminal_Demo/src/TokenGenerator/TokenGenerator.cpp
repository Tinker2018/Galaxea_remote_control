#include "TokenGenerator.h"
#include "AccessToken.h"

std::string TokenGenerator::generateServerToken(
    const std::string& app_id, const std::string& app_key, 
    const std::string& room_id, const std::string& uid
) {
    if (app_id.empty() || app_key.empty() || room_id.empty() || uid.empty()) {
        return "";
    }

    rtc::tools::AccessToken token(app_id, app_key, room_id, uid);
    token.AddPrivilege(rtc::tools::AccessToken::Privileges::PrivSubscribeStream, time(0));
    token.AddPrivilege(rtc::tools::AccessToken::Privileges::PrivPublishStream, time(0) + 3600);
    token.ExpireTime(time(0) + 3600 * 2);

    std::string s = token.Serialize();
    return s;
}

std::string TokenGenerator::generate(
    const std::string& app_id, const std::string& app_key, 
    const std::string& room_id, const std::string& uid
) {
    std::string token_server;
    if (app_id.empty() || app_key.empty() || room_id.empty() || uid.empty()) {
        return token_server;
    }

    token_server = generateServerToken(app_id, app_key, room_id, uid);
    return token_server;
}