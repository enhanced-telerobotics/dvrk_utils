from launch import LaunchDescription
from launch_ros.actions import Node
from pathlib import Path
import os
from glob import glob
import numpy as np
from scipy.spatial.transform import Rotation as R

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_share_directory = get_package_share_directory('dvrk_utils')
    transform_file_paths = glob(os.path.join(package_share_directory, 'resource', '*.npy'))

    transform_nodes = []
    for i, file_path in enumerate(transform_file_paths):
        # Load the transformation matrix from the file
        T = np.load(file_path)
        translation = T[:3, 3]
        rotation = R.from_matrix(T[:3, :3]).as_quat()
        print(T)

        # Extract parent and child frame names from the file name
        file_path = Path(file_path).expanduser()
        parent_frame, child_frame = file_path.stem.split("_T_", maxsplit=1)

        node = Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name=f'tf_publisher_{i}',
                output='screen',
                arguments=[
                    '--x', f"{translation[0]:.17g}",
                    '--y', f"{translation[1]:.17g}",
                    '--z', f"{translation[2]:.17g}",
                    '--qx', f"{rotation[0]:.17g}",
                    '--qy', f"{rotation[1]:.17g}",
                    '--qz', f"{rotation[2]:.17g}",
                    '--qw', f"{rotation[3]:.17g}",
                    '--frame-id', parent_frame,
                    '--child-frame-id', child_frame
                ]
            )
        transform_nodes.append(node)
        
    return LaunchDescription(transform_nodes)
