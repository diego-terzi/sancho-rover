from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('sancho_bringup'),
        'config',
        'sancho_params.yaml'
    )

    wheel_separation_arg = DeclareLaunchArgument(
        'wheel_separation', default_value='2.0',
        description='Effective lateral wheel separation in metres.',
    )

    return LaunchDescription([
        wheel_separation_arg,
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
            parameters=[
                config,
                {
                    'wheel_separation': ParameterValue(
                        LaunchConfiguration('wheel_separation'),
                        value_type=float,
                    ),
                },
            ],
            output='screen',
        ),
        Node(
            package='sancho_perception',
            executable='sensor_node',
            name='sensor_node',
            parameters=[config],
            output='screen',
        ),
    ])
