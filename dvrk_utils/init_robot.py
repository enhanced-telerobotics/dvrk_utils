import argparse
import json5
import time
from pathlib import Path
from typing import Optional

import crtk
import dvrk
import numpy as np
from spatialmath import SE3

from dvrk_utils import frame_to_se3, se3_to_frame

DEFAULT_CONFIG_JSON = Path('~/.ros') / 'dvrk_configs' / 'shuyuan.json'


def main(config_json: Optional[str] = None):
    if config_json is None:
        config_json = parse_args().config_json

    config = load_config(config_json)

    ral = crtk.ral('init_node')
    ral.check_connections()
    ral.spin()

    console = dvrk.console(ral, 'console')
    arms = build_arms(ral, config)

    while not is_homed(arms):
        home_all(console)
        time.sleep(5)

    while not check(arms, config):
        move(arms, config)
        time.sleep(5)

    ral.shutdown()


def parse_args():
    parser = argparse.ArgumentParser(
        description='Initialize dVRK arms from a JSON config file.')
    parser.add_argument(
        '-j', '--config-json',
        default=str(DEFAULT_CONFIG_JSON),
        help=f'Path to a JSON config file',
    )
    args, _ = parser.parse_known_args()
    return args


def load_config(config_json):
    config_path = Path(config_json).expanduser().resolve()
    with config_path.open('r', encoding='utf-8') as file_handle:
        return json5.load(file_handle)


def build_arms(ral, config):
    arms = {}
    for arm_name in config:
        arms[arm_name] = create_arm(ral, arm_name)
    return arms


def create_arm(ral, arm_name):
    upper_name = arm_name.upper()
    if upper_name.startswith('ECM'):
        return dvrk.ecm(ral, arm_name)
    if upper_name.startswith('PSM'):
        return dvrk.psm(ral, arm_name)
    if upper_name.startswith('MTM'):
        return dvrk.mtm(ral, arm_name)

    raise ValueError(f'Unsupported arm type in config: {arm_name}')


def home_all(console):
    console.power_on()
    console.home()


def is_homed(arms):
    return all(arm.is_homed() for arm in arms.values())


def move(arms, config):
    for arm_name, arm in arms.items():
        target = config[arm_name]

        if 'jp' in target:
            arm.move_jp(jp_config_to_command(target['jp']))

        if 'jaw_jp' in target and hasattr(arm, 'jaw'):
            arm.jaw.move_jp(np.deg2rad(
                np.asarray(target['jaw_jp'], dtype=float)))

        if 'cp' in target:
            arm.move_cp(se3_to_frame(np_to_se3(target['cp'])))


def check(arms, config, threshold=5e-2):
    for arm_name, arm in arms.items():
        target = config[arm_name]

        if 'jp' in target:
            measured_jp = np.asarray(arm.measured_jp(), dtype=float)
            expected_jp = jp_config_to_command(target['jp'])
            if np.abs(measured_jp - expected_jp).sum() >= threshold:
                return False

        if 'jaw_jp' in target and hasattr(arm, 'jaw'):
            measured_jaw = np.asarray(arm.jaw.measured_jp(), dtype=float)
            expected_jaw = np.deg2rad(
                np.asarray(target['jaw_jp'], dtype=float))
            if np.abs(measured_jaw - expected_jaw).sum() >= threshold:
                return False

        if 'cp' in target:
            measured_cp = frame_to_se3(arm.measured_cp())
            expected_cp = np_to_se3(target['cp'])
            if np.linalg.norm(measured_cp.t - expected_cp.t) >= threshold:
                return False
            if np.linalg.norm(measured_cp.R - expected_cp.R) >= threshold:
                return False

    return True


def jp_config_to_command(jp_config):
    jp_command = np.asarray(jp_config, dtype=float).copy()
    if jp_command.size >= 2:
        jp_command[:2] = np.deg2rad(jp_command[:2])
    if jp_command.size >= 3:
        jp_command[2] = jp_command[2] / 1000.0
    if jp_command.size > 3:
        jp_command[3:] = np.deg2rad(jp_command[3:])
    return jp_command


def np_to_se3(matrix):
    return SE3(np.asarray(matrix, dtype=float))


if __name__ == "__main__":
    main()
