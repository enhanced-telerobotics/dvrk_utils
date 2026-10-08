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

### 4. Robot Initialization

```bash
ros2 launch dvrk_utils init_robot.launch.py json:=bimanual
```

Joint targets are reached when the sum of absolute joint errors is below
`0.10` for MTML/MTMR and `0.05` for other arms. To override the limit, add
`"jp_threshold": 0.12` beside `"jp"` in the relevant arm's config in
`~/.ros/dvrk_configs/bimanual.json`. The threshold uses command units
(radians for rotational joints, meters for insertion), rather than the
degrees and millimeters used by the target config. Jaw and Cartesian
tolerances are unchanged.

### 5. Save Current Robot Pose

Create or update an initialization config from measured positions:

```bash
ros2 run dvrk_utils save_robot --json bimanual
ros2 run dvrk_utils save_robot --json bimanual --arms MTML MTMR --decimals 1
ros2 run dvrk_utils save_robot --config-json /tmp/current_pose.json
```

Existing configs determine the default arm list; new configs capture ECM,
PSM1, PSM2, MTMR, and MTML. Joint values use the existing initialization
convention (degrees, with the third element in millimeters), rounded to
whole numbers by default. PSM jaw angles are also saved in degrees.
Existing Cartesian targets are updated from measured poses, rounded to
six decimal places. Other config settings and unselected arms are preserved.
JSON5 input is accepted; output is formatted JSON, so comments are removed.
An existing file is copied to `<filename>.bak` before replacement (the backup
is replaced on each save). Missing measurements abort the save after
`--timeout 10` seconds without modifying the file. The command only reads
robot measurements and does not command motion.

## License

TODO: Add license information.

## Maintainer

- **Erie Lab**  
  Email: sxy841@case.edu
