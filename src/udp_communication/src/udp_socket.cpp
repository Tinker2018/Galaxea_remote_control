#include "udp_socket.hpp"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>  // 新增：非阻塞模式头文件
#include <errno.h>
#include <string.h>
#include <stdexcept>
#include <string>

namespace udp_communication {

// 构造函数
UDPSocket::UDPSocket(const UDPConfig& config) : sock_fd_(-1), config_(config) {
    if (config.role == "robot") {
        // 机器人端：初始化本地socket+目标地址
        init_socket(config.robot_local_ip, config.robot_local_port);
        // 初始化目标地址（PC）
        memset(&target_addr_, 0, sizeof(target_addr_));
        target_addr_.sin_family = AF_INET;
        target_addr_.sin_port = htons(config.robot_target_port);
        if (inet_pton(AF_INET, config.robot_target_ip.c_str(), &target_addr_.sin_addr) <= 0) {
            throw std::runtime_error("无效的PC IP地址：" + config.robot_target_ip);
        }
    } else if (config.role == "pc") {
        // PC端：仅初始化本地socket
        init_socket(config.pc_local_ip, config.pc_local_port);
    } else {
        throw std::runtime_error("无效的角色配置：" + config.role + "（仅支持robot/pc）");
    }
}

// 析构函数
UDPSocket::~UDPSocket() {
    if (sock_fd_ >= 0) {
        close(sock_fd_);
    }
}

// 发送数据
bool UDPSocket::send(const uint8_t* data, size_t len) {
    if (sock_fd_ < 0 || config_.role != "robot") {
        return false;
    }

    ssize_t sent = sendto(
        sock_fd_, data, len, 0,
        reinterpret_cast<sockaddr*>(&target_addr_), sizeof(target_addr_)
    );
    return sent == static_cast<ssize_t>(len);
}

// 接收数据
ssize_t UDPSocket::receive(uint8_t* buffer, size_t buffer_size) {
    if (sock_fd_ < 0 || config_.role != "pc") {
        return -1;
    }

    sockaddr_in src_addr{};
    socklen_t src_len = sizeof(src_addr);
    return recvfrom(
        sock_fd_, buffer, buffer_size, 0,
        reinterpret_cast<sockaddr*>(&src_addr), &src_len
    );
}

// 初始化socket（核心修改：加端口复用+处理0.0.0.0）
void UDPSocket::init_socket(const std::string& local_ip, uint16_t local_port) {
    // 1. 创建UDP socket（新增：SOCK_NONBLOCK非阻塞模式，避免卡死）
    sock_fd_ = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);
    if (sock_fd_ < 0) {
        throw std::runtime_error("创建UDP Socket失败：" + std::string(strerror(errno)));
    }

    // 2. 新增：设置端口复用（解决绑定失败的核心！）
    int reuse = 1;
    if (setsockopt(sock_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
        close(sock_fd_);
        throw std::runtime_error("设置端口复用失败：" + std::string(strerror(errno)));
    }

    // 3. 绑定本地地址（修改：处理0.0.0.0为INADDR_ANY）
    sockaddr_in local_addr{};
    local_addr.sin_family = AF_INET;
    local_addr.sin_port = htons(local_port);
    
    // 关键修改：单独处理0.0.0.0（通配地址）
    if (local_ip == "0.0.0.0") {
        local_addr.sin_addr.s_addr = htonl(INADDR_ANY); // 0.0.0.0对应INADDR_ANY
    } else {
        if (inet_pton(AF_INET, local_ip.c_str(), &local_addr.sin_addr) <= 0) {
            close(sock_fd_);
            throw std::runtime_error("无效的本地IP地址：" + local_ip);
        }
    }

    if (bind(sock_fd_, reinterpret_cast<sockaddr*>(&local_addr), sizeof(local_addr)) < 0) {
        close(sock_fd_);
        throw std::runtime_error("绑定UDP端口失败：" + std::string(strerror(errno)));
    }
}

// 加载yaml配置
UDPConfig UDPSocket::load_config(const std::string& config_path) {
    UDPConfig config;
    try {
        YAML::Node yaml_node = YAML::LoadFile(config_path);
        auto udp_node = yaml_node["udp_config"];
        
        // 角色
        config.role = udp_node["role"].as<std::string>();
        
        // 机器人端配置
        auto robot_node = udp_node["robot"];
        config.robot_local_ip = robot_node["local_ip"].as<std::string>();
        config.robot_local_port = robot_node["local_port"].as<uint16_t>();
        config.robot_target_ip = robot_node["target_ip"].as<std::string>();
        config.robot_target_port = robot_node["target_port"].as<uint16_t>();
        
        // PC端配置
        auto pc_node = udp_node["pc"];
        config.pc_local_ip = pc_node["local_ip"].as<std::string>();
        config.pc_local_port = pc_node["local_port"].as<uint16_t>();
    } catch (const YAML::Exception& e) {
        throw std::runtime_error("加载YAML配置失败：" + std::string(e.what()));
    }
    return config;
}

}  // namespace udp_communication
