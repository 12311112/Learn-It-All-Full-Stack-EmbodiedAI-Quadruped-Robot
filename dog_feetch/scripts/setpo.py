import os
import sys
import time
import numpy as np

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from runtime.position_hwi import HWI

hwi = HWI(usb_port="/dev/ttyUSB0")
joint_names = list(hwi.joints.keys())

target_dict = {name: float(np.pi) for name in joint_names}

hwi.turn_on()
time.sleep(1.0)
hwi.set_position_all(target_dict)
print("All servos commanded to pi rad.")
