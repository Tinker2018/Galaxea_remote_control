#include "udp_socket.hpp"
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <chrono>
#include "udp_messages_generated.h"
#include <cinttypes>  // 必须包含，用于PRId64宏

// 显式使用命名空间
using namespace udp_communication;
using namespace flatbuffers;

class UDPCommNode : public rclcpp::Node
{
public:
    UDPCommNode() : Node("udp_communication_node"), send_count_(0)
    {
        declare_parameters();
        read_parameters();

        try {
            // 端口强转uint16_t，匹配Socket构造函数
            udp_socket_ = std::make_unique<UDPSocket>(
                local_ip_, static_cast<uint16_t>(port_),
                remote_ip_, static_cast<uint16_t>(port_),
                buffer_size_
            );
        } catch (const std::invalid_argument& e) {
            RCLCPP_FATAL(this->get_logger(), "UDP Socket初始化失败：%s", e.what());
            rclcpp::shutdown();
            return;
        }

        if (!udp_socket_->create_socket()) {
            RCLCPP_FATAL(this->get_logger(), "创建UDP Socket失败");
            rclcpp::shutdown();
            return;
        }

        if (role_ == "robot") {
            if (!udp_socket_->bind_socket()) {
                RCLCPP_FATAL(this->get_logger(), "Robot绑定端口失败");
                rclcpp::shutdown();
                return;
            }
            send_timer_ = this->create_wall_timer(
                std::chrono::seconds(2),
                std::bind(&UDPCommNode::robot_send_msg, this)
            );
            recv_thread_ = std::thread(&UDPCommNode::robot_recv_msg, this);
        } else if (role_ == "pc") {
            if (!udp_socket_->bind_socket()) {
                RCLCPP_FATAL(this->get_logger(), "PC绑定端口失败");
                rclcpp::shutdown();
                return;
            }
            send_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(2) ,
                std::bind(&UDPCommNode::pc_send_msg, this)
            );
            recv_thread_ = std::thread(&UDPCommNode::pc_recv_msg, this);
        } else {
            RCLCPP_FATAL(this->get_logger(), "无效角色！仅支持 pc/robot");
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "UDP %s 节点初始化完成", role_.c_str());
    }

    ~UDPCommNode()
    {
        if (recv_thread_.joinable()) {
            recv_thread_.join();
        }
    }

private:
    void declare_parameters()
    {
        this->declare_parameter("udp_config.pc_ip", "127.0.0.1");
        this->declare_parameter("udp_config.robot_ip", "127.0.0.1");
        this->declare_parameter("udp_config.udp_port", 8888);
        this->declare_parameter("udp_config.role", "pc");
        this->declare_parameter("udp_config.buffer_size", 1024);
    }

    void read_parameters()
    {
        this->get_parameter("udp_config.pc_ip", pc_ip_);
        this->get_parameter("udp_config.robot_ip", robot_ip_);
        this->get_parameter("udp_config.udp_port", port_);
        this->get_parameter("udp_config.role", role_);
        this->get_parameter("udp_config.buffer_size", buffer_size_);

        local_ip_ = (role_ == "pc") ? pc_ip_ : robot_ip_;
        remote_ip_ = (role_ == "pc") ? robot_ip_ : pc_ip_;
    }

    // PC→Robot 发送消息（完全匹配FlatBuffers生成规则）
    void pc_send_msg()
    {
        FlatBufferBuilder builder;
        Offset<void> msg_offset;
        Pc2RobotMsg msg_type;

        if (send_count_ % 2 == 0) {
            // 1. 构造控制指令
            auto cmd = CreateControlCmd(builder, 0.5f, 0.2f, 1);
            msg_offset = cmd.Union();
            // 关键修正：Union枚举名 = Union名_子类型名
            msg_type = Pc2RobotMsg::Pc2RobotMsg_ControlCmd;
            RCLCPP_INFO(this->get_logger(), "PC准备发送：控制指令（线速度0.5，角速度0.2）");
        } else {
            // 2. 构造心跳包
            int64_t ts = rclcpp::Clock().now().nanoseconds() / 1000000;
            auto heartbeat = CreateHeartbeat(builder, ts, true);
            msg_offset = heartbeat.Union();
            // 关键修正：Union枚举名 = Union名_子类型名
            msg_type = Pc2RobotMsg::Pc2RobotMsg_Heartbeat;
            // 关键修正：用PRId64格式化int64_t
            RCLCPP_INFO(this->get_logger(), "PC准备发送：心跳包（时间戳=%" PRId64 "，在线状态true）", ts);
        }

        // 封装到包装器
        auto wrapper = CreatePc2RobotWrapper(builder, msg_type, msg_offset);
        builder.Finish(wrapper);

        // 发送数据
        std::string data(reinterpret_cast<const char*>(builder.GetBufferPointer()), builder.GetSize());
        udp_socket_->send_data(data);
        send_count_++;
    }

    // PC接收Robot消息（修正根类型获取+Union枚举+格式化）
    void pc_recv_msg()
    {
        std::string recv_data;
        while (rclcpp::ok()) {
            ssize_t recv_bytes = udp_socket_->recv_data(recv_data);
            if (recv_bytes > 0) {
                // 关键修正：FlatBuffers根类型获取用GetRoot<类型>
                auto wrapper = GetRoot<Robot2PcWrapper>(recv_data.data());
                if (!wrapper) {
                    RCLCPP_WARN(this->get_logger(), "PC接收：无效的Robot消息格式");
                    continue;
                }

                // 匹配Union枚举
                switch (wrapper->msg_type()) {
                    case Robot2PcMsg::Robot2PcMsg_SensorData: {
                        auto sensor = wrapper->msg_as_SensorData();
                        RCLCPP_INFO(this->get_logger(), 
                            "PC接收：传感器数据 | 温度=%.2f℃，距离=%.2fm，时间戳=%" PRId64 "ms",
                            sensor->temperature(), sensor->distance(), sensor->timestamp());
                        break;
                    }
                    case Robot2PcMsg::Robot2PcMsg_StatusReport: {
                        auto status = wrapper->msg_as_StatusReport();
                        RCLCPP_INFO(this->get_logger(), 
                            "PC接收：状态上报 | 电量=%.1f%%，错误码=%d，时间戳=%" PRId64 "ms",
                            status->battery(), status->error_code(), status->timestamp());
                        break;
                    }
                    default:
                        RCLCPP_WARN(this->get_logger(), "PC接收：未知的Robot消息类型");
                        break;
                }
            }
        }
    }

    // Robot→PC 发送消息（完全匹配FlatBuffers生成规则）
    void robot_send_msg()
    {
        FlatBufferBuilder builder;
        Offset<void> msg_offset;
        Robot2PcMsg msg_type;

        if (send_count_ % 2 == 0) {
            // 1. 构造传感器数据
            int64_t ts = rclcpp::Clock().now().nanoseconds() / 1000000;
            auto sensor = CreateSensorData(builder, 30.5f, 2.8f, ts);
            msg_offset = sensor.Union();
            msg_type = Robot2PcMsg::Robot2PcMsg_SensorData;
            RCLCPP_INFO(this->get_logger(), "Robot准备发送：传感器数据（温度30.5℃，距离2.8m）");
        } else {
            // 2. 构造状态上报
            int64_t ts = rclcpp::Clock().now().nanoseconds() / 1000000;
            auto status = CreateStatusReport(builder, 85.5f, 0, ts);
            msg_offset = status.Union();
            msg_type = Robot2PcMsg::Robot2PcMsg_StatusReport;
            // 修正：%转义为%%
            RCLCPP_INFO(this->get_logger(), "Robot准备发送：状态上报（电量85.5%%，错误码0）");
        }

        // 封装到包装器
        auto wrapper = CreateRobot2PcWrapper(builder, msg_type, msg_offset);
        builder.Finish(wrapper);

        // 发送数据
        std::string data(reinterpret_cast<const char*>(builder.GetBufferPointer()), builder.GetSize());
        udp_socket_->send_data(data);
        send_count_++;
    }

    // Robot接收PC消息（修正根类型获取+Union枚举+格式化）
    void robot_recv_msg()
    {
        std::string recv_data;
        while (rclcpp::ok()) {
            ssize_t recv_bytes = udp_socket_->recv_data(recv_data);
            if (recv_bytes > 0) {
                // 关键修正：FlatBuffers根类型获取用GetRoot<类型>（无GetXxxWrapper函数）
                auto wrapper = GetRoot<Pc2RobotWrapper>(recv_data.data());
                if (!wrapper) {
                    RCLCPP_WARN(this->get_logger(), "Robot接收：无效的PC消息格式");
                    continue;
                }

                // 匹配Union枚举
                switch (wrapper->msg_type()) {
                    case Pc2RobotMsg::Pc2RobotMsg_ControlCmd: {
                        auto cmd = wrapper->msg_as_ControlCmd();
                        RCLCPP_INFO(this->get_logger(), 
                            "Robot接收：控制指令 | 线速度=%.2fm/s，角速度=%.2frad/s，模式=%d",
                            cmd->linear_vel(), cmd->angular_vel(), cmd->mode());
                        break;
                    }
                    case Pc2RobotMsg::Pc2RobotMsg_Heartbeat: {
                        auto heartbeat = wrapper->msg_as_Heartbeat();
                        RCLCPP_INFO(this->get_logger(), 
                            "Robot接收：心跳包 | 时间戳=%" PRId64 "ms，PC在线状态=%s",
                            heartbeat->timestamp(), heartbeat->pc_status() ? "在线" : "离线");
                        break;
                    }
                    default:
                        RCLCPP_WARN(this->get_logger(), "Robot接收：未知的PC消息类型");
                        break;
                }
            }
        }
    }

    // 成员变量
    std::unique_ptr<UDPSocket> udp_socket_;
    std::string pc_ip_, robot_ip_, local_ip_, remote_ip_;
    int port_;
    std::string role_;
    size_t buffer_size_;
    rclcpp::TimerBase::SharedPtr send_timer_;
    std::thread recv_thread_;
    int send_count_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UDPCommNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}