from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('sancho_bringup'),
        'config',
        'sancho_params.yaml'
    )
    return LaunchDescription([
        Node(
            package='sancho_perception',
            executable='camera_node',
            name='camera_node',
            parameters=[config],
            output='screen',
        ),
        Node(
            package='sancho_control',
            executable='controller_node',
            name='controller_node',
            parameters=[config],
            output='screen',
        ),
        Node(
            package='sancho_bridge',
            executable='motor_bridge_node',
            name='motor_bridge_node',
            parameters=[config],
            output='screen',
        ),
    ])
