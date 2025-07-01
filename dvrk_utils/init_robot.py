import time
import crtk
import dvrk
import numpy as np


ECM_CONFIG = np.deg2rad([0, -15, np.rad2deg(0.02), 0])
PSM1_CONFIG = np.deg2rad([50, -5, np.rad2deg(0.14), 20, 15, 0])


def main():
    ral = crtk.ral('dvrk_python_node')
    ral.check_connections()
    ral.spin()

    console = dvrk.console(ral, 'console')

    ecm = dvrk.ecm(ral, 'ECM')
    psm1 = dvrk.psm(ral, 'PSM1')

    arms = [ecm, psm1]

    while not is_homed(arms):
        home_all(console)
        time.sleep(2)

    if is_homed(arms):
        # Move the arms to their initial positions
        while not check(arms):
            move(arms)
            time.sleep(5)

    ral.shutdown()


def home_all(console):
    console.power_on()
    console.home()

def is_homed(arms):
    return all(arm.is_homed() for arm in arms)

def move(arms):
    arms[0].move_jp(ECM_CONFIG)
    arms[1].move_jp(PSM1_CONFIG)
    arms[1].jaw.open()

def check(arms, threshold: float = 5e-2) -> bool:
    ecm_jp = arms[0].measured_jp()
    psm1_jp = arms[1].measured_jp()

    if (
        np.abs(ecm_jp - ECM_CONFIG).sum() < threshold and
        np.abs(psm1_jp - PSM1_CONFIG).sum() < threshold
    ):
        return True

    return False


if __name__ == "__main__":
    main()