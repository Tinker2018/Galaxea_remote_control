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

namespace galaxea_robot_tele {  // 统一命名空间，修复原笔误

// UDP配置结构体
struct UDPConfig {
    std::string role;                  // 角色：robot/pc
    // 机器人端配置
    std::string robot_local_ip;        // 机器人本地IP
    uint16_t robot_local_port;         // 机器人本地端口
    std::string robot_target_ip;       // 机器人目标IP（PC）
    uint16_t robot_target_port;        // 机器人目标端口（PC）
    // PC端配置
    std::string pc_local_ip;           // PC本地IP
    uint16_t pc_local_port;            // PC本地端口
    std::string pc_target_ip;          // PC目标IP（机器人）
    uint16_t pc_target_port;           // PC目标端口（机器人）
    size_t buffer_size;                // 缓冲区大小
};

class UDPSocket {
public:
    // 构造函数（从yaml配置初始化）
    UDPSocket(const UDPConfig& config);
    ~UDPSocket();

    // 发送数据
    bool send(const uint8_t* data, size_t len);

    // 接收数据（带IP验证，非阻塞）
    ssize_t receive(uint8_t* buffer, size_t buffer_size);

    // 读取yaml配置
    static UDPConfig load_config(const std::string& config_path);

private:
    // 初始化socket（端口复用+非阻塞+绑定）
    void init_socket(const std::string& local_ip, uint16_t local_port);

    // 成员变量
    int sock_fd_;                      // Socket文件描述符
    UDPConfig config_;                 // 配置信息
    sockaddr_in target_addr_;          // 发送目标地址
    std::string expected_src_ip_;      // 新增：期望的接收端IP（验证用）
};

}  // namespace galaxea_robot_tele

#endif  // UDP_SOCKET_HPP_
