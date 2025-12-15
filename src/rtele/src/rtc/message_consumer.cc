#include "src/rtc/message_consumer.h"
#include <chrono>
#include <iostream>
#include <thread>

namespace rtele {
namespace rtc {

MessageConsummer::MessageConsummer(MessageCallback callback) 
    : callback_(std::move(callback)) {
}

MessageConsummer::~MessageConsummer() {
    Stop();
}

void MessageConsummer::Start() {
    if (running_) {
        std::cerr << "MessageConsummer already running\n";
        return;
    }
    
    std::cout << "\n=== Starting Message Listener Thread ===\n";
    
    running_ = true;
    listen_thread_ = std::make_unique<std::thread>(&MessageConsummer::ListenLoop, this);
    
    std::cout << "✓ Message listener thread started\n";
}

void MessageConsummer::Stop() {
    if (!running_) {
        return;
    }
    
    std::cout << "\n=== Stopping Message Listener Thread ===\n";
    
    running_ = false;
    if (listen_thread_ && listen_thread_->joinable()) {
        listen_thread_->join();
    }
    listen_thread_.reset();
    
    std::cout << "✓ Message listener thread stopped\n";
}

void MessageConsummer::SimulateMessage(const std::string& user_id, 
                                       const std::string& message) {
    if (callback_) {
        callback_(user_id, message);
    }
}

void MessageConsummer::ListenLoop() {
    std::cout << "[Message Listener] Thread started\n";
    
    int tick_count = 0;
    
    while (running_) {
        // TODO: 实际监听消息队列
        // 在真实实现中，这里应该从 RTC SDK 的消息队列中读取消息
        // 例如通过回调或轮询方式获取房间广播消息
        
        // 模拟：每30秒打印一次状态
        if (tick_count % 30 == 0) {
            std::cout << "[Message Listener] Listening for room messages...\n";
        }
        
        tick_count++;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    
    std::cout << "[Message Listener] Thread stopped\n";
}

}  // namespace rtc
}  // namespace rtele
