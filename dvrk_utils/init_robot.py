import argparse
import json5
import time
from pathlib import Path
from typing import Any, Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, Float64

from dvrk_utils.utils import (
    joint_state_message,
    jp_config_to_command,
    np_to_se3,
    pose_stamped_to_se3,
    se3_to_pose_stamped,
)

DEFAULT_CONFIG_JSON = Path('~/.ros') / 'dvrk_configs' / 'shuyuan.json'


def main(config_json: Optional[str] = None) -> None:
    if config_json is None:
        config_json = parse_args().config_json

    config = load_config(config_json)

    rclpy.init()
    node = ArmTopicController(config)

    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Initialize arm topics from a JSON config file.')
    parser.add_argument(
        '-j', '--config-json',
        default=str(DEFAULT_CONFIG_JSON),
        help='Path to a JSON config file',
    )
    args, _ = parser.parse_known_args()
    return args


def load_config(config_json: str) -> dict:
    config_path = Path(config_json).expanduser().resolve()
    with config_path.open('r', encoding='utf-8') as file_handle:
        return json5.load(file_handle)


def is_arm_config(value) -> bool:
    return isinstance(value, dict) and any(key in value for key in ('jp', 'jaw_jp', 'cp'))


def rotation_error_deg(measured_rotation: np.ndarray, expected_rotation: np.ndarray) -> float:
    # Use the geodesic angle of relative rotation instead of element-wise matrix norm.
    relative_rotation = measured_rotation.T @ expected_rotation
    cos_theta = (np.trace(relative_rotation) - 1.0) / 2.0
    cos_theta = float(np.clip(cos_theta, -1.0, 1.0))
    return float(np.rad2deg(np.arccos(cos_theta)))


class ArmTopicController(Node):
    def __init__(self, config: dict) -> None:
        super().__init__('init_robot')
        self.config = config
        self.arm_names = [name for name,
                          value in config.items() if is_arm_config(value)]
        self.latest_measured_js: dict[str, Optional[JointState]] = {
            arm_name: None for arm_name in self.arm_names
        }
        self.latest_measured_jaw_js: dict[str, Optional[JointState]] = {
            arm_name: None for arm_name in self.arm_names
        }
        self.latest_measured_cp: dict[str, Optional[PoseStamped]] = {
            arm_name: None for arm_name in self.arm_names
        }
        self.move_jp_publishers: dict[str, Any] = {}
        self.jaw_move_jp_publishers: dict[str, Any] = {}
        self.move_cp_publishers: dict[str, Any] = {}

        self.console_power_on = self.create_publisher(
            Empty, '/console/power_on', 10)
        self.console_home = self.create_publisher(Empty, '/console/home', 10)
        self.console_teleop_set_scale = self.create_publisher(
            Float64, '/console/teleop/set_scale', 10)

        self.measured_js_subscriptions = []
        self.measured_jaw_js_subscriptions = []
        self.measured_cp_subscriptions = []

        for arm_name in self.arm_names:
            self.move_jp_publishers[arm_name] = self.create_publisher(
                JointState, f'/{arm_name}/move_jp', 10)
            self.jaw_move_jp_publishers[arm_name] = self.create_publisher(
                JointState, f'/{arm_name}/jaw/move_jp', 10)
            self.move_cp_publishers[arm_name] = self.create_publisher(
                PoseStamped, f'/{arm_name}/move_cp', 10)

            self.measured_js_subscriptions.append(
                self.create_subscription(
                    JointState,
                    f'/{arm_name}/measured_js',
                    lambda message, arm_name=arm_name: self._on_measured_js(
                        arm_name, message),
                    10,
                )
            )
            self.measured_jaw_js_subscriptions.append(
                self.create_subscription(
                    JointState,
                    f'/{arm_name}/jaw/measured_js',
                    lambda message, arm_name=arm_name: self._on_measured_jaw_js(
                        arm_name, message),
                    10,
                )
            )
            self.measured_cp_subscriptions.append(
                self.create_subscription(
                    PoseStamped,
                    f'/{arm_name}/measured_cp',
                    lambda message, arm_name=arm_name: self._on_measured_cp(
                        arm_name, message),
                    10,
                )
            )

    def _on_measured_js(self, arm_name: str, message: JointState) -> None:
        self.latest_measured_js[arm_name] = message

    def _on_measured_jaw_js(self, arm_name: str, message: JointState) -> None:
        self.latest_measured_jaw_js[arm_name] = message

    def _on_measured_cp(self, arm_name: str, message: PoseStamped) -> None:
        self.latest_measured_cp[arm_name] = message

    def publish_console_command(self) -> None:
        self.console_power_on.publish(Empty())
        self.console_home.publish(Empty())

        teleop_scale = self.config.get('teleop_scale')
        if teleop_scale is not None:
            self.console_teleop_set_scale.publish(
                Float64(data=float(teleop_scale)))

    def publish_targets(self) -> None:
        for arm_name in self.arm_names:
            target = self.config[arm_name]

            if 'jp' in target:
                self.move_jp_publishers[arm_name].publish(
                    joint_state_message(jp_config_to_command(target['jp'])))

            if 'jaw_jp' in target:
                self.jaw_move_jp_publishers[arm_name].publish(
                    joint_state_message(np.deg2rad(np.asarray(target['jaw_jp'], dtype=float))))

            if 'cp' in target:
                self.move_cp_publishers[arm_name].publish(
                    se3_to_pose_stamped(np_to_se3(target['cp']), arm_name))

    def targets_reached(
        self,
        joint_threshold: float = 5e-2,
        translation_threshold: float = 5e-2,
        rotation_threshold_deg: float = 5.0,
    ) -> bool:
        all_reached = True

        for arm_name in self.arm_names:
            target = self.config[arm_name]

            if 'jp' in target:
                measured_js = self.latest_measured_js[arm_name]
                if measured_js is None:
                    self.get_logger().warning(
                        f'{arm_name}: target jp not reached, measured_js is missing')
                    all_reached = False
                    continue
                measured_jp = np.asarray(measured_js.position, dtype=float)
                expected_jp = jp_config_to_command(target['jp'])
                if measured_jp.shape != expected_jp.shape:
                    self.get_logger().warning(
                        f'{arm_name}: target jp not reached, '
                        f'shape mismatch measured={measured_jp.shape} expected={expected_jp.shape}')
                    all_reached = False
                jp_error = float(np.abs(measured_jp - expected_jp).sum())
                if jp_error >= joint_threshold:
                    self.get_logger().warning(
                        f'{arm_name}: target jp not reached, error={jp_error:.6f}, '
                        f'threshold={joint_threshold:.6f}')
                    all_reached = False

            if 'jaw_jp' in target:
                measured_jaw_js = self.latest_measured_jaw_js[arm_name]
                measured_js = self.latest_measured_js[arm_name]
                if measured_jaw_js is None and measured_js is None:
                    self.get_logger().warning(
                        f'{arm_name}: target jaw_jp not reached, '
                        'jaw and arm measured_js are missing')
                    all_reached = False
                    continue
                if measured_jaw_js is not None and measured_jaw_js.position:
                    measured_jaw = np.asarray(
                        measured_jaw_js.position, dtype=float)
                else:
                    if measured_js is None or not measured_js.position:
                        self.get_logger().warning(
                            f'{arm_name}: target jaw_jp not reached, '
                            'fallback arm measured_js has no position')
                        all_reached = False
                        continue
                    measured_jaw = np.asarray(
                        [measured_js.position[-1]], dtype=float)
                expected_jaw = np.deg2rad(
                    np.asarray(target['jaw_jp'], dtype=float))
                jaw_error = float(np.abs(measured_jaw - expected_jaw).sum())
                if jaw_error >= joint_threshold:
                    self.get_logger().warning(
                        f'{arm_name}: target jaw_jp not reached, error={jaw_error:.6f}, '
                        f'threshold={joint_threshold:.6f}')
                    all_reached = False

            if 'cp' in target:
                measured_cp = self.latest_measured_cp[arm_name]
                if measured_cp is None:
                    self.get_logger().warning(
                        f'{arm_name}: target cp not reached, measured_cp is missing')
                    all_reached = False
                    continue
                measured_pose = pose_stamped_to_se3(measured_cp)
                expected_pose = np_to_se3(target['cp'])
                translation_error = float(np.linalg.norm(
                    measured_pose.t - expected_pose.t))
                if translation_error >= translation_threshold:
                    self.get_logger().warning(
                        f'{arm_name}: target cp translation not reached, '
                        f'error={translation_error:.6f}, threshold={translation_threshold:.6f}')
                    all_reached = False
                rotation_error = rotation_error_deg(measured_pose.R, expected_pose.R)
                if rotation_error >= rotation_threshold_deg:
                    self.get_logger().warning(
                        f'{arm_name}: target cp rotation not reached, '
                        f'error={rotation_error:.3f} deg, '
                        f'threshold={rotation_threshold_deg:.3f} deg')
                    all_reached = False

        return all_reached

    def run(self) -> None:
        self.get_logger().info('Publishing console commands and arm targets.')

        while rclpy.ok():
            self.publish_console_command()
            self.publish_targets()

            deadline = time.monotonic() + 1.0
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.1)

            if self.targets_reached():
                self.get_logger().info('All configured targets reached.')
                return

            self.get_logger().info('Waiting for measured state to match targets.')
            time.sleep(3.0)


if __name__ == '__main__':
    main()
