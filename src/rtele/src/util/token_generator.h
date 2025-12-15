#ifndef RTELE_UTIL_TOKEN_GENERATOR_H_
#define RTELE_UTIL_TOKEN_GENERATOR_H_

#include <string>

namespace rtele {
namespace util {

// 简单的 Token 生成器
// 生成用于 RTC 鉴权的 token
class TokenGenerator {
public:
    // 生成 token
    // 参数：app_id, app_key, room_id, user_id
    static std::string Generate(const std::string& app_id,
                                const std::string& app_key,
                                const std::string& room_id,
                                const std::string& user_id);
};

}  // namespace util
}  // namespace rtele

#endif  // RTELE_UTIL_TOKEN_GENERATOR_H_
