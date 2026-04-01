from launch import LaunchDescription
from launch_ros.actions import Node
from pathlib import Path

def generate_launch_description():
    transform_file_paths = sorted(Path('~/.ros/tf_static').expanduser().glob('*.npy'))

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
