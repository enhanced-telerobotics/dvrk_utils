from launch import LaunchDescription
from launch_ros.actions import Node
import os
from glob import glob

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_share_directory = get_package_share_directory('dvrk_utils')
    transform_file_paths = glob(os.path.join(package_share_directory, 'resource', '*.npy'))

    transform_nodes = []
    for i, transform_file_path in enumerate(transform_file_paths):
        node = Node(
                package='dvrk_utils',
                executable='tf_publisher',
                name=f'tf_publisher_{i}',
                output='screen',
                arguments=[transform_file_path],
                parameters=[
                    {
                        'publish_rate': 100.0
                    }
                ]
            )
        transform_nodes.append(node)
        
    return LaunchDescription(transform_nodes)
