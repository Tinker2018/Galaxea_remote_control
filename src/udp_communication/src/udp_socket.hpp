#ifndef UDP_SOCKET_HPP
#define UDP_SOCKET_HPP

#include <string>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <stdexcept>
#include <rclcpp/logging.hpp>
#include <cstring>

class UDPSocket
{
public:
    // 构造函数声明（端口为uint16_t，无Logger成员）
    UDPSocket(const std::string& local_ip, uint16_t local_port,
              const std::string& remote_ip, uint16_t remote_port, size_t buffer_size);
    
    // 析构函数声明
    ~UDPSocket();

    // 成员函数声明
    bool create_socket();
    bool bind_socket();
    ssize_t send_data(const std::string& data);
    ssize_t recv_data(std::string& data);
    void close_socket();

private:
    // 移除Logger成员，仅保留核心变量
    std::string local_ip_;
    uint16_t local_port_;
    std::string remote_ip_;
    uint16_t remote_port_;
    size_t buffer_size_;
    int sock_fd_;
};

#endif // UDP_SOCKET_HPP