# dVRK Utils

ROS 2 utilities for browser-based control, robot pose configuration, and TF transforms.

## Build

From your ROS 2 workspace, with ROS sourced:

```bash
cd ~/ros2_ws
rosdep install --from-paths src/dvrk_utils --ignore-src -r -y
colcon build --packages-select dvrk_utils
source install/setup.bash
```

## Control panel

Start the normal dVRK ROS system, then run:

```bash
ros2 run dvrk_utils control_panel
```

Connect your phone or computer to the same network, scan the QR code in the
PC window, and choose **Take control** in the browser. The panel provides arm
status, power, homing, reset, teleoperation, motion scale, and volume controls.
Video streaming is not included.

```bash
# Select the PC address shown in the QR code.
ros2 run dvrk_utils control_panel --advertise-address 192.168.1.20

# Set the console namespace or reuse a panel JSON config.
ros2 run dvrk_utils control_panel -C console
ros2 run dvrk_utils control_panel -c panel.json

# Run without the PC window.
ros2 run dvrk_utils control_panel --headless --port 8081
```

The server listens on `0.0.0.0:8080` by default. Use `--host` to select a
specific interface. Close the PC window or press Ctrl+C to stop it.
Only one browser can control the robot at a time. Disconnecting keeps control
reserved; release it in that browser or use **Regenerate connection / release
control** on the PC. Regeneration invalidates previous links and sessions.
The pairing token is saved in `~/.config/dvrk_utils/control_panel.token`
(or under `$XDG_CONFIG_HOME`) and reused after restarts. To explicitly replace
it from the terminal, run `ros2 run dvrk_utils control_panel --reset-pairing`.
The link and QR stay the same while the advertised hostname and port stay the same.

Settings are read from `~/.config/dvrk_utils/control_panel.json` (or under
`$XDG_CONFIG_HOME`), created automatically on first run:

```json
{"hostname": "robot-pc.local", "host": "0.0.0.0", "port": 8080, "console": "console"}
```

The default `hostname` is empty: the panel uses the first alias from `hostname -a`,
or the PC hostname when there are no aliases. The tablet must be able to resolve
that name. CLI options override the file; `-c` selects a different JSON file.
Use `--advertise-address` to use an IP address instead.

Use a trusted local network: HTTP traffic is not encrypted. A wireless
disconnection does not stop the robot; use the existing dVRK emergency stop.
The panel requires `rclpy`, `crtk_msgs`, and `aiohttp`; the QR window also uses
GTK, Pillow, and `qrcode`. These dependencies are declared in `package.xml`.

## Robot poses

```bash
# Initialize from ~/.ros/dvrk_configs/bimanual.json.
ros2 launch dvrk_utils init_robot.launch.py json:=bimanual

# Save measured positions to an initialization config.
ros2 run dvrk_utils save_robot --json bimanual
ros2 run dvrk_utils save_robot --json bimanual --arms MTML MTMR --decimals 1
ros2 run dvrk_utils save_robot --config-json /tmp/current_pose.json
```

Initialization uses summed joint-error limits of `0.10` for MTML/MTMR and
`0.05` for other arms. Override an arm's limit with `"jp_threshold"` beside
`"jp"` in its config. Limits use radians and meters; target configs use degrees
and millimeters.

Saving only reads measurements. It preserves other settings and unselected
arms, backs up an existing file to `<filename>.bak`, and aborts on missing
measurements. Joint and jaw values use degrees, with the third joint in
millimeters. JSON5 input is accepted; output is JSON without comments.

## TF utilities

```bash
ros2 run dvrk_utils tf_listener ECM_base Board
ros2 run dvrk_utils tf_publisher /path/to/ECM_base_T_Board.npy
ros2 launch dvrk_utils board_publisher.launch.py
```

Transform files contain a 4×4 homogeneous matrix and must be named
`<parent_frame>_T_<child_frame>.npy` so the publisher can infer the frames.

For Python use, import `TransformRetriever` from `dvrk_utils.tf_listener`.
Conversion helpers `stamped_to_se3`, `frame_to_se3`, and `se3_to_frame` are
available from `dvrk_utils.utils`.

Maintainer: Erie Lab · sxy841@case.edu
