from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = PathJoinSubstitution([
        FindPackageShare('sancho_rover'),
        'config',
        'sancho_params.yaml',
    ])

    return LaunchDescription([
        Node(
            package='sancho_rover',
            executable='camera_node',
            name='camera_node',
            parameters=[params_file],
        ),
        Node(
            package='sancho_rover',
            executable='controller_node',
            name='controller_node',
            parameters=[params_file],
        ),
        Node(
            package='sancho_rover',
            executable='motor_bridge_node',
            name='motor_bridge_node',
            parameters=[params_file],
        ),
        Node(
            package='sancho_rover',
            executable='sensor_node',
            name='sensor_node',
            parameters=[params_file],
        ),
    ])
