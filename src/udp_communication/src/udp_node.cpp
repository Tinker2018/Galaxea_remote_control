#include "udp_socket.hpp"
#include "udp_messages_generated.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <flatbuffers/flatbuffers.h>
#include <thread>
#include <atomic>
#include <ament_index_cpp/get_package_share_directory.hpp>

// 注意：这里移除了所有混入的fbs语法代码！
namespace udp_communication {

class UDPCommNode : public rclcpp::Node {
public:
    UDPCommNode() : Node("udp_comm_node"), is_running_(true) {
        // 加载配置文件
        std::string config_path = ament_index_cpp::get_package_share_directory("udp_communication") + "/config/udp_config.yaml";
        try {
            udp_config_ = UDPSocket::load_config(config_path);
            udp_socket_ = std::make_unique<UDPSocket>(udp_config_);
            RCLCPP_INFO(this->get_logger(), "UDP配置加载成功，角色：%s", udp_config_.role.c_str());
        } catch (const std::exception& e) {
            RCLCPP_FATAL(this->get_logger(), "UDP初始化失败：%s", e.what());
            rclcpp::shutdown();
            return;
        }

        // 根据角色初始化
        if (udp_config_.role == "robot") {
            init_robot_mode();
        } else if (udp_config_.role == "pc") {
            init_pc_mode();
        }
    }

    ~UDPCommNode() {
        is_running_ = false;
        if (recv_thread_.joinable()) {
            recv_thread_.join();
        }
    }

private:
    // 机器人端初始化
    void init_robot_mode() {
        // 订阅6个目标话题（传递sub_type参数，补全命名空间）
        sub_left_arm_joint_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/hdas/feedback_arm_left", 10,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                this->send_joint_state(udp_communication::JointPoseSubType_JOINT_STATE_LEFT_ARM, *msg);
            }
        );
        sub_right_arm_joint_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/hdas/feedback_arm_right", 10,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                this->send_joint_state(udp_communication::JointPoseSubType_JOINT_STATE_RIGHT_ARM, *msg);
            }
        );
        sub_left_gripper_joint_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/hdas/feedback_gripper_left", 10,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                this->send_joint_state(udp_communication::JointPoseSubType_JOINT_STATE_LEFT_GRIPPER, *msg);
            }
        );
        sub_right_gripper_joint_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/hdas/feedback_gripper_right", 10,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                this->send_joint_state(udp_communication::JointPoseSubType_JOINT_STATE_RIGHT_GRIPPER, *msg);
            }
        );
        sub_left_arm_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/motion_control/pose_ee_arm_left", 10,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                this->send_pose_stamped(udp_communication::JointPoseSubType_POSE_STAMPED_LEFT_ARM, *msg);
            }
        );
        sub_right_arm_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/motion_control/pose_ee_arm_right", 10,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                this->send_pose_stamped(udp_communication::JointPoseSubType_POSE_STAMPED_RIGHT_ARM, *msg);
            }
        );

        RCLCPP_INFO(this->get_logger(), "机器人模式初始化完成，已订阅所有目标话题");
    }

    // PC端初始化
    void init_pc_mode() {
        // 创建6个发布者
        pub_left_arm_joint_ = this->create_publisher<sensor_msgs::msg::JointState>("/hdas/feedback_arm_left", 10);
        pub_right_arm_joint_ = this->create_publisher<sensor_msgs::msg::JointState>("/hdas/feedback_arm_right", 10);
        pub_left_gripper_joint_ = this->create_publisher<sensor_msgs::msg::JointState>("/hdas/feedback_gripper_left", 10);
        pub_right_gripper_joint_ = this->create_publisher<sensor_msgs::msg::JointState>("/hdas/feedback_gripper_right", 10);
        pub_left_arm_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/motion_control/pose_ee_arm_left", 10);
        pub_right_arm_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/motion_control/pose_ee_arm_right", 10);

        // 启动接收线程
        recv_thread_ = std::thread(&UDPCommNode::pc_recv_loop, this);

        RCLCPP_INFO(this->get_logger(), "PC模式初始化完成，等待机器人数据...");
    }

    // 发送关节状态（参数改为JointPoseSubType，补全命名空间）
    void send_joint_state(udp_communication::JointPoseSubType sub_type, const sensor_msgs::msg::JointState& ros_msg) {
        flatbuffers::FlatBufferBuilder builder(1024);

        // 转换double到float
        std::vector<float> positions(ros_msg.position.begin(), ros_msg.position.end());
        std::vector<float> velocities(ros_msg.velocity.begin(), ros_msg.velocity.end());
        std::vector<float> efforts(ros_msg.effort.begin(), ros_msg.effort.end());
        
        auto names_vec = builder.CreateVectorOfStrings(ros_msg.name);
        auto positions_vec = builder.CreateVector(positions);
        auto velocities_vec = builder.CreateVector(velocities);
        auto efforts_vec = builder.CreateVector(efforts);

        // 构建JointState（包含sub_type）
        auto js = udp_communication::CreateJointState(
            builder,
            sub_type,  // 传入子类型（左/右臂/夹爪）
            names_vec, positions_vec, velocities_vec, efforts_vec,
            ros_msg.header.stamp.sec, ros_msg.header.stamp.nanosec,
            builder.CreateString(ros_msg.header.frame_id)
        );

        // 构建Robot2PcWrapper（使用Robot2PcMsg_JointState）
        auto wrapper = udp_communication::CreateRobot2PcWrapper(
            builder,
            udp_communication::Robot2PcMsg_JointState,  // FlatBuffers生成的union类型
            js.Union()
        );
        udp_communication::FinishRobot2PcWrapperBuffer(builder, wrapper);

        // 发送数据
        if (udp_socket_->send(builder.GetBufferPointer(), builder.GetSize())) {
            RCLCPP_DEBUG(this->get_logger(), "发送关节消息成功（类型：%d）", sub_type);
        } else {
            RCLCPP_WARN(this->get_logger(), "发送关节消息失败（类型：%d）", sub_type);
        }
    }

    // 发送位姿消息（参数改为JointPoseSubType，补全命名空间）
    void send_pose_stamped(udp_communication::JointPoseSubType sub_type, const geometry_msgs::msg::PoseStamped& ros_msg) {
        flatbuffers::FlatBufferBuilder builder(1024);

        // 构建Pose
        auto pose = udp_communication::CreatePose(
            builder,
            ros_msg.pose.position.x, ros_msg.pose.position.y, ros_msg.pose.position.z,
            ros_msg.pose.orientation.x, ros_msg.pose.orientation.y, ros_msg.pose.orientation.z, ros_msg.pose.orientation.w
        );

        // 构建PoseStamped（包含sub_type）
        auto ps = udp_communication::CreatePoseStamped(
            builder,
            sub_type,  // 传入子类型（左/右臂）
            builder.CreateString(ros_msg.header.frame_id),
            ros_msg.header.stamp.sec, ros_msg.header.stamp.nanosec,
            pose
        );

        // 构建Robot2PcWrapper（使用Robot2PcMsg_PoseStamped）
        auto wrapper = udp_communication::CreateRobot2PcWrapper(
            builder,
            udp_communication::Robot2PcMsg_PoseStamped,  // FlatBuffers生成的union类型
            ps.Union()
        );
        udp_communication::FinishRobot2PcWrapperBuffer(builder, wrapper);

        // 发送数据
        if (udp_socket_->send(builder.GetBufferPointer(), builder.GetSize())) {
            RCLCPP_DEBUG(this->get_logger(), "发送位姿消息成功（类型：%d）", sub_type);
        } else {
            RCLCPP_WARN(this->get_logger(), "发送位姿消息失败（类型：%d）", sub_type);
        }
    }

    // PC端接收循环（修正枚举引用，补全命名空间）
    void pc_recv_loop() {
        const size_t BUF_SIZE = 4096;
        uint8_t buffer[BUF_SIZE];

        while (is_running_ && rclcpp::ok()) {
            ssize_t recv_len = udp_socket_->receive(buffer, BUF_SIZE);
            if (recv_len <= 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }

            // 解析根对象
            auto wrapper = udp_communication::GetRobot2PcWrapper(buffer);
            if (!wrapper) {
                RCLCPP_WARN(this->get_logger(), "接收到无效数据");
                continue;
            }

            // 根据union类型解析
            switch (wrapper->msg_type()) {
                case udp_communication::Robot2PcMsg_JointState: {
                    auto js = wrapper->msg_as_JointState();
                    if (!js) break;
                    // 根据sub_type发布到对应话题（补全枚举命名空间）
                    switch (js->sub_type()) {
                        case udp_communication::JointPoseSubType_JOINT_STATE_LEFT_ARM:
                            parse_joint_state(js, pub_left_arm_joint_);
                            break;
                        case udp_communication::JointPoseSubType_JOINT_STATE_RIGHT_ARM:
                            parse_joint_state(js, pub_right_arm_joint_);
                            break;
                        case udp_communication::JointPoseSubType_JOINT_STATE_LEFT_GRIPPER:
                            parse_joint_state(js, pub_left_gripper_joint_);
                            break;
                        case udp_communication::JointPoseSubType_JOINT_STATE_RIGHT_GRIPPER:
                            parse_joint_state(js, pub_right_gripper_joint_);
                            break;
                        default:
                            RCLCPP_WARN(this->get_logger(), "未知关节消息子类型：%d", js->sub_type());
                            break;
                    }
                    break;
                }
                case udp_communication::Robot2PcMsg_PoseStamped: {
                    auto ps = wrapper->msg_as_PoseStamped();
                    if (!ps) break;
                    // 根据sub_type发布到对应话题（补全枚举命名空间）
                    switch (ps->sub_type()) {
                        case udp_communication::JointPoseSubType_POSE_STAMPED_LEFT_ARM:
                            parse_pose_stamped(ps, pub_left_arm_pose_);
                            break;
                        case udp_communication::JointPoseSubType_POSE_STAMPED_RIGHT_ARM:
                            parse_pose_stamped(ps, pub_right_arm_pose_);
                            break;
                        default:
                            RCLCPP_WARN(this->get_logger(), "未知位姿消息子类型：%d", ps->sub_type());
                            break;
                    }
                    break;
                }
                default:
                    RCLCPP_WARN(this->get_logger(), "未知消息类型：%d", wrapper->msg_type());
                    break;
            }
        }
    }

    // 解析关节状态并发布
    void parse_joint_state(const udp_communication::JointState* fb_js, const rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr& pub) {
        if (!fb_js || !pub) return;

        sensor_msgs::msg::JointState ros_msg;
        ros_msg.header.stamp.sec = fb_js->stamp_sec();
        ros_msg.header.stamp.nanosec = fb_js->stamp_nanosec();
        ros_msg.header.frame_id = fb_js->frame_id()->str();

        if (fb_js->names()) {
            for (const auto& name : *fb_js->names()) {
                ros_msg.name.push_back(name->str());
            }
        }
        if (fb_js->positions()) {
            for (float pos : *fb_js->positions()) {
                ros_msg.position.push_back(static_cast<double>(pos));
            }
        }
        if (fb_js->velocities()) {
            for (float vel : *fb_js->velocities()) {
                ros_msg.velocity.push_back(static_cast<double>(vel));
            }
        }
        if (fb_js->efforts()) {
            for (float eff : *fb_js->efforts()) {
                ros_msg.effort.push_back(static_cast<double>(eff));
            }
        }

        pub->publish(ros_msg);
    }

    // 解析位姿消息并发布
    void parse_pose_stamped(const udp_communication::PoseStamped* fb_ps, const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr& pub) {
        if (!fb_ps || !pub) return;

        geometry_msgs::msg::PoseStamped ros_msg;
        ros_msg.header.frame_id = fb_ps->frame_id()->str();
        ros_msg.header.stamp.sec = fb_ps->stamp_sec();
        ros_msg.header.stamp.nanosec = fb_ps->stamp_nanosec();

        auto pose = fb_ps->pose();
        if (pose) {
            ros_msg.pose.position.x = pose->x();
            ros_msg.pose.position.y = pose->y();
            ros_msg.pose.position.z = pose->z();
            ros_msg.pose.orientation.x = pose->qx();
            ros_msg.pose.orientation.y = pose->qy();
            ros_msg.pose.orientation.z = pose->qz();
            ros_msg.pose.orientation.w = pose->qw();
        }

        pub->publish(ros_msg);
    }

    // 成员变量
    std::unique_ptr<UDPSocket> udp_socket_;
    UDPConfig udp_config_;
    std::atomic<bool> is_running_;
    std::thread recv_thread_;

    // 机器人端订阅者
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_left_arm_joint_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_right_arm_joint_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_left_gripper_joint_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_right_gripper_joint_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_left_arm_pose_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_right_arm_pose_;

    // PC端发布者
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_left_arm_joint_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_right_arm_joint_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_left_gripper_joint_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_right_gripper_joint_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_left_arm_pose_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_right_arm_pose_;
};

}  // namespace udp_communication

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<udp_communication::UDPCommNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
