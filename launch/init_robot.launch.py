from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    json_arg = DeclareLaunchArgument(
        'json',
        default_value='shuyuan',
        description='Config name under ~/.ros/dvrk_configs without extension',
    )

    config_json_path = PathJoinSubstitution([
        EnvironmentVariable('HOME'),
        '.ros',
        'dvrk_configs',
        [LaunchConfiguration('json'), '.json'],
    ])

    init_robot_node = Node(
        package='dvrk_utils',
        executable='init_robot',
        name='init_robot',
        output='screen',
        arguments=['--config-json', config_json_path],
    )

    return LaunchDescription([
        json_arg,
        init_robot_node,
    ])
