# DVRK Utils

This package provides utilities for working with the dVRK (da Vinci Research Kit) in ROS 2. It includes tools for publishing and retrieving transforms between frames using static and dynamic TF2.

## Features

- **TF Listener**: Retrieve and display transforms between frames.
- **TF Publisher**: Publish static transforms from a 4x4 transformation matrix file.
- **Launch File**: Automate the publishing of static transforms using a launch file.

## Installation

Ensure you have ROS 2 installed and sourced. Clone this repository into your ROS 2 workspace and build it:

```bash
cd ~/ros2_ws/src
# Clone the repository here
colcon build --packages-select dvrk_utils
source ~/ros2_ws/install/setup.bash
```

## Usage

### 1. TF Listener

#### a. Use the node instance
```
from spatialmath import SE3, SO3

from dvrk_utils.tf_listener import TransformRetriever
from dvrk_utils.utils import (
    stamped_to_se3,
    frame_to_se3,
    se3_to_frame,
)
```

```
node = TransformRetriever()
camera_T_center = stamped_to_se3(node.get_transform("ECM", "Center"))

center_T_peg1 = SE3.Rx(np.pi)
center_T_peg1.t[0] += 0.0125

camera_T_peg1 = camera_T_center @ center_T_peg1
psm1.move_cp(se3_to_frame(camera_T_peg1))
```

#### b. Check though CLI

The `tf_listener` node retrieves and displays the transform between two frames. Example usage:

```bash
ros2 run dvrk_utils tf_listener <parent_frame> <child_frame>
```

#### Example:

```bash
ros2 run dvrk_utils tf_listener ECM_base Board
```

Output:

```
[INFO] [<timestamp>] [transform_retriever]: Transform from ECM_base to Board: 
  -0.9989    0.02241   0.04148   0.2635    
  -0.01309   0.7134   -0.7006    0.1855    
  -0.0453   -0.7004   -0.7123    0.01187   
   0         0         0         1         
```

### 2. TF Publisher

The `tf_publisher` node publishes a static transform from a 4x4 transformation matrix file. Example usage:

```bash
ros2 run dvrk_utils tf_publisher <transform_file>
```

#### Example:

```bash
ros2 run dvrk_utils tf_publisher /home/erie_lab/ros2_ws/src/dvrk_utils/resource/ECM_base_T_Board.npy
```

### Transform File Naming Convention

The `transform_file` should follow the naming convention `<parent_frame>_T_<child_frame>.npy`. This format is essential for the `tf_publisher` node to correctly extract the parent and child frame names from the file name.

#### Example:

For a file named `ECM_base_T_Board.npy`:
- `parent_frame`: `ECM_base`
- `child_frame`: `Board`

The file must contain a 4x4 homogeneous transformation matrix representing the transform from the `parent_frame` to the `child_frame`. Ensure the file is saved in `.npy` format and placed in the appropriate directory (e.g., `resource/`).

### 3. Launch File

The `board_publisher.launch.py` file automates the publishing of a static transform. Example usage:

```bash
ros2 launch dvrk_utils board_publisher.launch.py
```

This will publish the transform defined in the `ECM_base_T_Board.npy` file located in the `resource` directory.

## License

TODO: Add license information.

## Maintainer

- **Erie Lab**  
  Email: sxy841@case.edu
