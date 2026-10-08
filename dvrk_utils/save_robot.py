"""Capture measured robot targets in the initialization config format."""

import argparse
import json
import shutil
import tempfile
import time
from pathlib import Path

import json5
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

from dvrk_utils.init_robot import is_arm_config
from dvrk_utils.utils import pose_stamped_to_se3

DEFAULT_ARMS = ['ECM', 'PSM1', 'PSM2', 'MTMR', 'MTML']


def rounded(values, decimals):
    values = np.asarray(values, dtype=float)
    if not np.all(np.isfinite(values)):
        raise ValueError('Measured positions must be finite')
    result = np.round(values, decimals)
    result[result == 0] = 0  # Avoid negative zero in JSON.
    return result.astype(int).tolist() if decimals == 0 else result.tolist()


def jp_to_config(positions, decimals):
    # Invert jp_config_to_command, including the existing third-joint convention.
    result = np.asarray(positions, dtype=float).copy()
    result[:2] = np.rad2deg(result[:2])
    if result.size >= 3:
        result[2] *= 1000.0
    if result.size > 3:
        result[3:] = np.rad2deg(result[3:])
    return rounded(result, decimals)


class PoseRecorder(Node):
    def __init__(self, config, arms):
        super().__init__('save_robot')
        self.measurements = {}
        self.subscriptions_to_keep = []
        self.required = []
        for arm in arms:
            target = config.get(arm, {})
            kinds = ['jp'] if 'cp' not in target or 'jp' in target else []
            if 'cp' in target:
                kinds.append('cp')
            if arm.startswith('PSM') or 'jaw_jp' in target:
                kinds.append('jaw_jp')
            for kind in kinds:
                key = (arm, kind)
                self.required.append(key)
                topic = {'jp': 'measured_js', 'jaw_jp': 'jaw/measured_js',
                         'cp': 'measured_cp'}[kind]
                message_type = PoseStamped if kind == 'cp' else JointState
                self.subscriptions_to_keep.append(self.create_subscription(
                    message_type, f'/{arm}/{topic}',
                    lambda message, key=key: self.record(key, message), 10))

    def record(self, key, message):
        if key[1] == 'cp' or len(message.position) > 0:
            self.measurements[key] = message

    def capture(self, config, timeout, decimals):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if all(key in self.measurements for key in self.required):
                break
            rclpy.spin_once(self, timeout_sec=min(0.1, max(0, deadline - time.monotonic())))
        missing = [f'{arm}/{kind}' for arm, kind in self.required
                   if (arm, kind) not in self.measurements]
        if not rclpy.ok() or missing:
            raise RuntimeError('Capture incomplete; file unchanged. Missing: ' + ', '.join(missing))
        for (arm, kind), message in self.measurements.items():
            target = config.setdefault(arm, {})
            if kind == 'jp':
                target[kind] = jp_to_config(message.position, decimals)
            elif kind == 'jaw_jp':
                target[kind] = rounded(np.rad2deg(message.position), decimals)
            else:
                # Keep sufficient precision for translations and rotation matrices.
                target[kind] = rounded(pose_stamped_to_se3(message).A, 6)
        return config


def write_config(path, config):
    content = json.dumps(config, indent=4, allow_nan=False) + '\n'
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        shutil.copy2(path, path.with_suffix(path.suffix + '.bak'))
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8',
                                         dir=path.parent, delete=False) as handle:
            temporary = Path(handle.name)
            handle.write(content)
        temporary.replace(path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    destination = parser.add_mutually_exclusive_group(required=True)
    destination.add_argument('--json', help='Config name under ~/.ros/dvrk_configs')
    destination.add_argument('-j', '--config-json', help='Config file path')
    parser.add_argument('--arms', nargs='+', help='Arms to capture (default: existing config arms or all five)')
    parser.add_argument('--decimals', type=int, default=0, help='Decimal places for degrees/mm (default: 0)')
    parser.add_argument('--timeout', type=float, default=10.0, help='Measurement timeout in seconds')
    args, _ = parser.parse_known_args()
    if args.decimals < 0 or not np.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('--decimals must be nonnegative and --timeout must be positive and finite')
    if args.json and (Path(args.json).name != args.json or args.json in ('.', '..')):
        parser.error('--json must be a config name, not a path; use --config-json for paths')
    path = (Path(args.config_json).expanduser() if args.config_json else
            Path.home() / '.ros' / 'dvrk_configs' / (args.json + '.json'))
    config = json5.loads(path.read_text()) if path.exists() else {}
    if not isinstance(config, dict):
        parser.error('Config must be a JSON object')
    arms = args.arms or [name for name, value in config.items() if is_arm_config(value)] or DEFAULT_ARMS
    if any(name in config and not isinstance(config[name], dict) for name in arms):
        parser.error('Selected arm entries must be JSON objects')
    rclpy.init()
    node = PoseRecorder(config, arms)
    try:
        node.get_logger().info(f'Capturing measured poses for {", ".join(arms)}')
        config = node.capture(config, args.timeout, args.decimals)
        write_config(path, config)
        node.get_logger().info(f'Saved {path} (existing file backed up as .json.bak)')
    except KeyboardInterrupt:
        pass
    except (RuntimeError, ValueError) as error:
        node.get_logger().error(str(error))
        raise SystemExit(1) from error
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
