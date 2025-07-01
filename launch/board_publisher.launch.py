from launch import LaunchDescription
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_share_directory = get_package_share_directory('dvrk_utils')
    transform_file_path = os.path.join(package_share_directory, 'resource', 'ECM_base_T_Board.npy')

    return LaunchDescription([
        Node(
            package='dvrk_utils',
            executable='tf_publisher',
            name='board_publisher',
            output='screen',
            arguments=[transform_file_path],
            parameters=[
                {
                    'publish_rate': 100.0  # Default rate is 100 Hz
                }
            ]
        )
    ])
