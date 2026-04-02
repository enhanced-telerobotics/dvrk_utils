import argparse
import numpy as np
import time

import crtk
import dvrk

ENGAGE_MAX = np.deg2rad([120, 60, 60, 45], dtype=float)
ENGAGE_MIN = np.deg2rad([-120, -60, -60, 0], dtype=float)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Engage and disengage a dVRK PSM tool jaw sequence.')
    parser.add_argument(
        '-a', '--arm-name',
        default='PSM1',
        help='PSM arm name to control (default: PSM2).',
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    ral = crtk.ral('tool_engage_node')
    try:
        ral.check_connections()
        ral.spin()

        arm = dvrk.psm(ral, args.arm_name)
        arm.enable(2) and arm.home(2)

        current_jp = np.concatenate(
            (arm.measured_jp(), arm.jaw.measured_jp()), dtype=float)

        target_jp = current_jp.copy()
        target_jp[3:] = ENGAGE_MAX
        move_arm(arm, target_jp)
        time.sleep(1)

        target_jp[3:] = ENGAGE_MIN
        move_arm(arm, target_jp)
        time.sleep(1)

        move_arm(arm, current_jp)
        time.sleep(1)

    finally:
        ral.shutdown()


def move_arm(arm: dvrk.arm, target_jp: np.ndarray) -> None:
    arm.move_jp(target_jp[:-1])
    arm.jaw.move_jp(target_jp[-1:])


if __name__ == "__main__":
    main()
