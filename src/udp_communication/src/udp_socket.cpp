#include "udp_socket.hpp"

// 构造函数实现（无Logger初始化，仅初始化核心变量）
UDPSocket::UDPSocket(const std::string& local_ip, uint16_t local_port,
                     const std::string& remote_ip, uint16_t remote_port, size_t buffer_size)
    : local_ip_(local_ip), local_port_(local_port),
      remote_ip_(remote_ip), remote_port_(remote_port),
      buffer_size_(buffer_size), sock_fd_(-1)
{
    if (buffer_size == 0) {
        throw std::invalid_argument("缓冲区大小不能为0");
    }
}

// 析构函数实现
UDPSocket::~UDPSocket()
{
    close_socket();
}

// 创建Socket（日志用临时Logger）
bool UDPSocket::create_socket()
{
    auto logger = rclcpp::get_logger("UDPSocket"); // 临时创建Logger
    sock_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd_ < 0) {
        RCLCPP_ERROR(logger, "创建Socket失败：%s", strerror(errno));
        return false;
    }
    RCLCPP_INFO(logger, "UDP Socket创建成功");
    return true;
}

// 绑定端口（日志用临时Logger）
bool UDPSocket::bind_socket()
{
    auto logger = rclcpp::get_logger("UDPSocket"); // 临时创建Logger
    sockaddr_in local_addr{};
    memset(&local_addr, 0, sizeof(local_addr));
    local_addr.sin_family = AF_INET;
    local_addr.sin_port = htons(local_port_);
    
    if (inet_pton(AF_INET, local_ip_.c_str(), &local_addr.sin_addr) <= 0) {
        RCLCPP_ERROR(logger, "本地IP格式错误：%s", local_ip_.c_str());
        return false;
    }

    if (bind(sock_fd_, (sockaddr*)&local_addr, sizeof(local_addr)) < 0) {
        RCLCPP_ERROR(logger, "绑定端口失败：%s", strerror(errno));
        return false;
    }
    RCLCPP_INFO(logger, "绑定到 %s:%d", local_ip_.c_str(), local_port_);
    return true;
}

// 发送数据（日志用临时Logger）
ssize_t UDPSocket::send_data(const std::string& data)
{
    auto logger = rclcpp::get_logger("UDPSocket"); // 临时创建Logger
    if (sock_fd_ < 0) {
        RCLCPP_ERROR(logger, "Socket未初始化");
        return -1;
    }

    sockaddr_in remote_addr{};
    memset(&remote_addr, 0, sizeof(remote_addr));
    remote_addr.sin_family = AF_INET;
    remote_addr.sin_port = htons(remote_port_);
    
    if (inet_pton(AF_INET, remote_ip_.c_str(), &remote_addr.sin_addr) <= 0) {
        RCLCPP_ERROR(logger, "远端IP格式错误：%s", remote_ip_.c_str());
        return -1;
    }

    ssize_t sent = sendto(sock_fd_, data.c_str(), data.size(), 0,
                          (sockaddr*)&remote_addr, sizeof(remote_addr));
    if (sent < 0) {
        RCLCPP_ERROR(logger, "发送数据失败：%s", strerror(errno));
        return -1;
    }
    RCLCPP_INFO(logger, "发送 %zd 字节到 %s:%d", sent, remote_ip_.c_str(), remote_port_);
    return sent;
}

// 接收数据（日志用临时Logger）
ssize_t UDPSocket::recv_data(std::string& data)
{
    auto logger = rclcpp::get_logger("UDPSocket"); // 临时创建Logger
    if (sock_fd_ < 0) {
        RCLCPP_ERROR(logger, "Socket未初始化");
        return -1;
    }

    char* buffer = new char[buffer_size_];
    memset(buffer, 0, buffer_size_);
    sockaddr_in remote_addr{};
    socklen_t addr_len = sizeof(remote_addr);

    ssize_t recv_len = recvfrom(sock_fd_, buffer, buffer_size_ - 1, 0,
                                (sockaddr*)&remote_addr, &addr_len);
    if (recv_len < 0) {
        RCLCPP_ERROR(logger, "接收数据失败：%s", strerror(errno));
        delete[] buffer;
        return -1;
    }

    char remote_ip[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &remote_addr.sin_addr, remote_ip, INET_ADDRSTRLEN);
    int remote_port = ntohs(remote_addr.sin_port);

    data.assign(buffer, recv_len);
    delete[] buffer;

    RCLCPP_INFO(logger, "从 %s:%d 接收 %zd 字节", remote_ip, remote_port, recv_len);
    return recv_len;
}

// 关闭Socket（日志用临时Logger）
void UDPSocket::close_socket()
{
    auto logger = rclcpp::get_logger("UDPSocket"); // 临时创建Logger
    if (sock_fd_ >= 0) {
        close(sock_fd_);
        sock_fd_ = -1;
        RCLCPP_INFO(logger, "UDP Socket已关闭");
    }
}