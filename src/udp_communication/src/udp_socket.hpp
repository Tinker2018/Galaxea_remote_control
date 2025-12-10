#ifndef UDP_SOCKET_HPP_
#define UDP_SOCKET_HPP_

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>
#include <string>
#include <stdexcept>
#include <yaml-cpp/yaml.h>
#include "rclcpp/rclcpp.hpp"

namespace udp_communication {

// UDP配置结构体
struct UDPConfig {
    std::string role;          // robot/pc
    // 机器人端配置
    std::string robot_local_ip;
    uint16_t robot_local_port;
    std::string robot_target_ip;
    uint16_t robot_target_port;
    // PC端配置
    std::string pc_local_ip;
    uint16_t pc_local_port;
};

class UDPSocket {
public:
    // 构造函数（从yaml配置初始化）
    UDPSocket(const UDPConfig& config);
    ~UDPSocket();

    // 发送数据（机器人端用）
    bool send(const uint8_t* data, size_t len);

    // 接收数据（PC端用，阻塞）
    ssize_t receive(uint8_t* buffer, size_t buffer_size);

    // 读取yaml配置
    static UDPConfig load_config(const std::string& config_path);

private:
    // 初始化socket
    void init_socket(const std::string& local_ip, uint16_t local_port);

    // 成员变量
    int sock_fd_;
    UDPConfig config_;
    sockaddr_in target_addr_;  // 机器人端目标地址（PC）
};

}  // namespace udp_communication

#endif  // UDP_SOCKET_HPP_
