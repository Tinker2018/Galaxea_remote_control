from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 1. 声明启动参数
    role_arg = DeclareLaunchArgument(
        'role',
        default_value=TextSubstitution(text='pc'),
        description='Role of the node: pc / robot'
    )

    pc_ip_arg = DeclareLaunchArgument(
        'pc_ip',
        default_value=TextSubstitution(text='127.0.0.1'),
        description='IP address of PC'
    )

    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value=TextSubstitution(text='127.0.0.1'),
        description='IP address of Robot'
    )

    port_arg = DeclareLaunchArgument(
        'udp_port',
        default_value=TextSubstitution(text='8888'),
        description='UDP communication port'
    )

    # 2. 获取通用配置文件路径
    config_file = os.path.join(
        get_package_share_directory('udp_communication'),
        'config',
        'udp_config.yaml'
    )

    # 3. 定义UDP节点（动态覆盖参数）
    udp_node = Node(
        package='udp_communication',
        executable='udp_comm_node',
        name='udp_communication_node',
        parameters=[
            config_file,  # 基础配置
            {
                'udp_config.role': LaunchConfiguration('role'),
                'udp_config.pc_ip': LaunchConfiguration('pc_ip'),
                'udp_config.robot_ip': LaunchConfiguration('robot_ip'),
                'udp_config.udp_port': LaunchConfiguration('udp_port')
            }
        ],
        output='screen'
    )

    # 4. 组装Launch描述
    return LaunchDescription([
        role_arg,
        pc_ip_arg,
        robot_ip_arg,
        port_arg,
        udp_node
    ])