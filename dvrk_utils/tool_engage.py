import argparse
import numpy as np
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty

from dvrk_utils.utils import joint_state_message

ENGAGE_MAX = np.deg2rad([120, 60, 60, 45], dtype=float)
ENGAGE_MIN = np.deg2rad([-120, -60, -60, 0], dtype=float)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Engage and disengage a dVRK PSM tool jaw sequence.')
    parser.add_argument(
        '-a', '--arm-name',
        default='PSM1',
        help='PSM arm name to control (default: PSM1).',
    )
    parser.add_argument(
        '--position-threshold',
        type=float,
        default=5e-2,
        help='Absolute joint error threshold in radians/meters (default: 0.05).',
    )
    parser.add_argument(
        '--move-timeout',
        type=float,
        default=8.0,
        help='Timeout in seconds to reach each target (default: 8.0).',
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = ToolEngageController(args.arm_name)

    try:
        node.run(
            position_threshold=args.position_threshold,
            move_timeout=args.move_timeout,
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


class ToolEngageController(Node):
    def __init__(self, arm_name: str) -> None:
        super().__init__('tool_engage')
        self.arm_name = arm_name

        self.console_power_on = self.create_publisher(Empty, '/console/power_on', 10)
        self.console_home = self.create_publisher(Empty, '/console/home', 10)

        self.move_jp_publisher = self.create_publisher(
            JointState, f'/{arm_name}/move_jp', 10)
        self.jaw_move_jp_publisher = self.create_publisher(
            JointState, f'/{arm_name}/jaw/move_jp', 10)

        self.latest_measured_js: JointState | None = None
        self.latest_measured_jaw_js: JointState | None = None

        self.measured_js_subscription = self.create_subscription(
            JointState,
            f'/{arm_name}/measured_js',
            self._on_measured_js,
            10,
        )
        self.measured_jaw_js_subscription = self.create_subscription(
            JointState,
            f'/{arm_name}/jaw/measured_js',
            self._on_measured_jaw_js,
            10,
        )

    def _on_measured_js(self, message: JointState) -> None:
        self.latest_measured_js = message

    def _on_measured_jaw_js(self, message: JointState) -> None:
        self.latest_measured_jaw_js = message

    def publish_console_command(self) -> None:
        self.console_power_on.publish(Empty())
        self.console_home.publish(Empty())

    def spin_for(self, duration_sec: float) -> None:
        deadline = time.monotonic() + duration_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

    def wait_for_initial_measurements(self, timeout_sec: float = 5.0) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest_measured_js is not None:
                return True
        return False

    def measured_jaw(self) -> np.ndarray | None:
        if self.latest_measured_jaw_js is not None and self.latest_measured_jaw_js.position:
            return np.asarray(self.latest_measured_jaw_js.position, dtype=float)
        if self.latest_measured_js is not None and self.latest_measured_js.position:
            # Fallback for setups where jaw/measured_js is unavailable.
            return np.asarray([self.latest_measured_js.position[-1]], dtype=float)
        return None

    def target_reached(
        self,
        target_arm: np.ndarray,
        target_jaw: np.ndarray,
        position_threshold: float,
    ) -> bool:
        if self.latest_measured_js is None:
            return False

        measured_arm = np.asarray(self.latest_measured_js.position, dtype=float)
        if measured_arm.shape != target_arm.shape:
            self.get_logger().warning(
                f'{self.arm_name}: arm shape mismatch measured={measured_arm.shape} target={target_arm.shape}')
            return False

        measured_jaw = self.measured_jaw()
        if measured_jaw is None or measured_jaw.shape != target_jaw.shape:
            return False

        arm_error = float(np.max(np.abs(measured_arm - target_arm)))
        jaw_error = float(np.max(np.abs(measured_jaw - target_jaw)))
        return arm_error < position_threshold and jaw_error < position_threshold

    def move_arm(
        self,
        target_arm: np.ndarray,
        target_jaw: np.ndarray,
        position_threshold: float,
        timeout_sec: float,
    ) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            self.move_jp_publisher.publish(joint_state_message(target_arm))
            self.jaw_move_jp_publisher.publish(joint_state_message(target_jaw))

            self.spin_for(0.2)
            if self.target_reached(target_arm, target_jaw, position_threshold):
                return True

        return False

    def run(self, position_threshold: float, move_timeout: float) -> None:
        self.get_logger().info(f'Engaging tool sequence on {self.arm_name}.')

        for _ in range(10):
            self.publish_console_command()
            self.spin_for(0.1)

        if not self.wait_for_initial_measurements(timeout_sec=5.0):
            raise RuntimeError(
                f'No measured joint state received for {self.arm_name} within timeout.')

        current_arm = np.asarray(self.latest_measured_js.position, dtype=float)
        current_jaw = self.measured_jaw()
        if current_jaw is None:
            raise RuntimeError(f'No measured jaw state available for {self.arm_name}.')

        if current_arm.size < 6:
            raise RuntimeError(
                f'Expected at least 6 arm joints for {self.arm_name}, got {current_arm.size}.')

        engage_max_arm = current_arm.copy()
        engage_max_arm[3:6] = ENGAGE_MAX[:3]
        engage_max_jaw = np.asarray([ENGAGE_MAX[3]], dtype=float)

        engage_min_arm = current_arm.copy()
        engage_min_arm[3:6] = ENGAGE_MIN[:3]
        engage_min_jaw = np.asarray([ENGAGE_MIN[3]], dtype=float)

        if not self.move_arm(engage_max_arm, engage_max_jaw, position_threshold, move_timeout):
            raise RuntimeError('Failed to reach engage max target before timeout.')
        self.spin_for(1.0)

        if not self.move_arm(engage_min_arm, engage_min_jaw, position_threshold, move_timeout):
            raise RuntimeError('Failed to reach engage min target before timeout.')
        self.spin_for(1.0)

        if not self.move_arm(current_arm, current_jaw, position_threshold, move_timeout):
            raise RuntimeError('Failed to return to initial joint target before timeout.')
        self.spin_for(1.0)

        self.get_logger().info('Tool engage sequence complete.')


if __name__ == "__main__":
    main()
