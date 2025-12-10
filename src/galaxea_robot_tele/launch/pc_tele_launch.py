from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='galaxea_robot_tele',
            executable='pc_tele_node',
            name='pc_tele_node',
            output='screen',
            parameters=[],
            arguments=[]
        )
    ])
