#include "udp_socket.hpp"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <stdexcept>
#include <string>
#include "rclcpp/rclcpp.hpp"

namespace galaxea_robot_tele {

// 构造函数：初始化Socket+目标地址+期望接收IP
UDPSocket::UDPSocket(const UDPConfig& config) : sock_fd_(-1), config_(config) {
    if (config.role == "robot") {
        // 机器人端：本地绑定 + 目标地址（PC） + 仅接收PC的IP
        init_socket(config.robot_local_ip, config.robot_local_port);
        memset(&target_addr_, 0, sizeof(target_addr_));
        target_addr_.sin_family = AF_INET;
        target_addr_.sin_port = htons(config.robot_target_port);
        if (inet_pton(AF_INET, config.robot_target_ip.c_str(), &target_addr_.sin_addr) <= 0) {
            throw std::runtime_error("无效的PC IP地址：" + config.robot_target_ip);
        }
        expected_src_ip_ = config.robot_target_ip;  // 机器人仅接收PC的IP

    } else if (config.role == "pc") {
        // PC端：本地绑定 + 目标地址（机器人） + 仅接收机器人的IP
        init_socket(config.pc_local_ip, config.pc_local_port);
        memset(&target_addr_, 0, sizeof(target_addr_));
        target_addr_.sin_family = AF_INET;
        target_addr_.sin_port = htons(config.pc_target_port);
        if (inet_pton(AF_INET, config.pc_target_ip.c_str(), &target_addr_.sin_addr) <= 0) {
            throw std::runtime_error("无效的机器人IP地址：" + config.pc_target_ip);
        }
        expected_src_ip_ = config.pc_target_ip;  // PC仅接收机器人的IP

    } else {
        throw std::runtime_error("无效的角色配置：" + config.role + "（仅支持robot/pc）");
    }
}

// 析构函数：关闭Socket
UDPSocket::~UDPSocket() {
    if (sock_fd_ >= 0) {
        close(sock_fd_);
    }
}

// 发送数据：UDP无连接发送
bool UDPSocket::send(const uint8_t* data, size_t len) {
    if (sock_fd_ < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("UDPSocket"), "Socket未初始化，发送失败");
        return false;
    }

    ssize_t sent = sendto(
        sock_fd_, data, len, 0,
        reinterpret_cast<sockaddr*>(&target_addr_), sizeof(target_addr_)
    );
    if (sent < 0) {
        RCLCPP_WARN(rclcpp::get_logger("UDPSocket"), "发送失败：%s", strerror(errno));
        return false;
    }
    return sent == static_cast<ssize_t>(len);
}

// 接收数据：核心新增IP验证，仅接收期望IP的数据包
ssize_t UDPSocket::receive(uint8_t* buffer, size_t buffer_size) {
    if (sock_fd_ < 0) {
        errno = EBADF;
        RCLCPP_ERROR(rclcpp::get_logger("UDPSocket"), "Socket未初始化，接收失败");
        return -1;
    }

    sockaddr_in src_addr{};
    socklen_t src_len = sizeof(src_addr);
    memset(buffer, 0, buffer_size);  // 清空缓冲区

    // 非阻塞接收数据
    ssize_t recv_len = recvfrom(
        sock_fd_, buffer, buffer_size, 0,
        reinterpret_cast<sockaddr*>(&src_addr), &src_len
    );

    // 1. 处理接收错误（非阻塞无数据是正常情况）
    if (recv_len < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            // 非阻塞模式下无数据，返回-1（上层可判断errno）
            return -1;
        }
        RCLCPP_WARN(rclcpp::get_logger("UDPSocket"), "接收错误：%s", strerror(errno));
        return -1;
    }

    // 2. 解析发送端IP地址
    char src_ip_str[INET_ADDRSTRLEN] = {0};
    if (inet_ntop(AF_INET, &src_addr.sin_addr, src_ip_str, INET_ADDRSTRLEN) == nullptr) {
        RCLCPP_WARN(rclcpp::get_logger("UDPSocket"), "解析发送端IP失败");
        errno = EINVAL;
        return -1;
    }
    std::string src_ip(src_ip_str);

    // 3. 核心验证：发送端IP是否匹配期望的IP（防止回环/非法数据）
    if (src_ip != expected_src_ip_) {
        RCLCPP_WARN(rclcpp::get_logger("UDPSocket"), 
                    "拒绝非法数据包！期望IP：%s，实际发送IP：%s",
                    expected_src_ip_.c_str(), src_ip.c_str());
        errno = ECONNREFUSED;  // 标记为“连接拒绝”
        return -1;
    }

    // 4. IP验证通过，返回接收字节数
    RCLCPP_DEBUG(rclcpp::get_logger("UDPSocket"), 
                 "成功接收来自%s的数据包，长度：%zd", src_ip.c_str(), recv_len);
    return recv_len;
}

// 初始化Socket：端口复用+非阻塞+绑定本地地址
void UDPSocket::init_socket(const std::string& local_ip, uint16_t local_port) {
    // 1. 创建非阻塞UDP Socket
    sock_fd_ = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);
    if (sock_fd_ < 0) {
        throw std::runtime_error("创建UDP Socket失败：" + std::string(strerror(errno)));
    }

    // 2. 开启端口复用（解决重复绑定问题）
    int reuse = 1;
    if (setsockopt(sock_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
        close(sock_fd_);
        throw std::runtime_error("设置端口复用失败：" + std::string(strerror(errno)));
    }

    // 3. 绑定本地地址（支持0.0.0.0通配地址）
    sockaddr_in local_addr{};
    local_addr.sin_family = AF_INET;
    local_addr.sin_port = htons(local_port);
    
    if (local_ip == "0.0.0.0") {
        local_addr.sin_addr.s_addr = htonl(INADDR_ANY);  // 通配地址
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

// 加载YAML配置：修复PC端配置赋值错误
UDPConfig UDPSocket::load_config(const std::string& config_path) {
    UDPConfig config;
    try {
        YAML::Node yaml_node = YAML::LoadFile(config_path);
        auto udp_node = yaml_node["udp_config"];
        
        // 基础配置
        config.role = udp_node["role"].as<std::string>();
        config.buffer_size = udp_node["buffer_size"].as<size_t>();
        
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
        config.pc_target_ip = pc_node["target_ip"].as<std::string>();  
        config.pc_target_port = pc_node["target_port"].as<uint16_t>();
    } catch (const YAML::Exception& e) {
        throw std::runtime_error("加载YAML配置失败：" + std::string(e.what()));
    }
    return config;
}

}  // namespace galaxea_robot_tele
