#include "src/util/token_generator.h"
#include <openssl/hmac.h>
#include <openssl/sha.h>
#include <ctime>
#include <sstream>
#include <iomanip>
#include <vector>

namespace rtele {
namespace util {

// Base64 编码
static std::string Base64Encode(const unsigned char* data, size_t len) {
    static const char encoding_table[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string result;
    result.reserve(((len + 2) / 3) * 4);
    
    for (size_t i = 0; i < len; i += 3) {
        uint32_t triple = (data[i] << 16) | ((i + 1 < len ? data[i + 1] : 0) << 8) | (i + 2 < len ? data[i + 2] : 0);
        
        result.push_back(encoding_table[(triple >> 18) & 0x3F]);
        result.push_back(encoding_table[(triple >> 12) & 0x3F]);
        result.push_back(i + 1 < len ? encoding_table[(triple >> 6) & 0x3F] : '=');
        result.push_back(i + 2 < len ? encoding_table[triple & 0x3F] : '=');
    }
    
    return result;
}

std::string TokenGenerator::Generate(const std::string& app_id,
                                     const std::string& app_key,
                                     const std::string& room_id,
                                     const std::string& token) {
    // 简化的 token 生成逻辑
    // 实际项目中应该使用 VolcEngine 提供的 TokenGenerator
    
    // 当前时间戳 + 24小时有效期
    time_t now = time(nullptr);
    time_t expire_time = now + 24 * 3600;
    
    // 构造签名内容：app_id + room_id + token + expire_time
    std::ostringstream oss;
    oss << app_id << room_id << token << expire_time;
    std::string content = oss.str();
    
    // 使用 HMAC-SHA256 签名
    unsigned char hash[SHA256_DIGEST_LENGTH];
    HMAC(EVP_sha256(), 
         app_key.c_str(), app_key.length(),
         reinterpret_cast<const unsigned char*>(content.c_str()), content.length(),
         hash, nullptr);
    
    // Base64 编码签名
    std::string signature = Base64Encode(hash, SHA256_DIGEST_LENGTH);
    
    // 返回格式：base64(app_id:expire_time:signature)
    std::ostringstream token_oss;
    token_oss << app_id << ":" << expire_time << ":" << signature;
    std::string token_content = token_oss.str();
    
    return Base64Encode(
        reinterpret_cast<const unsigned char*>(token_content.c_str()),
        token_content.length()
    );
}

}  // namespace util
}  // namespace rtele
