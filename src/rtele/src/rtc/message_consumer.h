#ifndef RTELE_RTC_MESSAGE_LISTENER_H_
#define RTELE_RTC_MESSAGE_LISTENER_H_

#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <thread>

namespace rtele {
namespace rtc {

// 房间消息回调类型
using MessageCallback = std::function<void(const std::string& user_id, const std::string& message)>;

// 房间消息监听器
// 在独立线程中监听房间广播消息
class MessageConsummer {
public:
    explicit MessageConsummer(MessageCallback callback);
    ~MessageConsummer();
    
    void Start();
    
    void Stop();
    
    bool IsRunning() const { return running_; }
    
    void SimulateMessage(const std::string& user_id, const std::string& message);

private:
    void ListenLoop();

    MessageCallback callback_;
    std::unique_ptr<std::thread> listen_thread_;
    std::atomic<bool> running_{false};
};

}  // namespace rtc
}  // namespace rtele

#endif  // RTELE_RTC_MESSAGE_LISTENER_H_
