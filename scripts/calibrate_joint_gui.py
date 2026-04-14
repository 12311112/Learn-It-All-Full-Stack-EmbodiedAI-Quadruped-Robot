#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
关节方向/零位标定脚本（独立于 walk_by_openloop）。

用途：
- 只动一条腿或一个关节，快速确认方向是否正确。
- 使用 real_pose(机械角) / init_pos(舵机角) 映射发送指令。
"""

import argparse
import math
import os
import sys
import time
from typing import Dict


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)


LEG_JOINTS = {
    "left_front": ["left_front_hip_joint", "left_front_knee_joint", "left_front_ankle_joint"],
    "right_front": ["right_front_hip_joint", "right_front_knee_joint", "right_front_ankle_joint"],
    "left_back": ["left_back_hip_joint", "left_back_knee_joint", "left_back_ankle_joint"],
    "right_back": ["right_back_hip_joint", "right_back_knee_joint", "right_back_ankle_joint"],
}


def mech_to_servo(hwi, joint_name: str, mech_angle: float) -> float:

    real_local = hwi.real_pose[joint_name]

    return (mech_angle - real_local)* hwi.real_pose_signs.get(joint_name, 1.0) + hwi.init_pos[joint_name] 


def send_mech_pose(hwi, mech_pose: Dict[str, float]):
    ids = []
    positions = []
    for joint_name, mech in mech_pose.items():
        ids.append(hwi.joints[joint_name])
        positions.append(mech_to_servo(hwi, joint_name, mech))
    hwi.io.write_goal_position(ids, positions)


def setup_gui():
    import pybullet as p

    if p.getConnectionInfo()["isConnected"] == 0:
        p.connect(p.GUI)

    sliders = {
        "delta": p.addUserDebugParameter("target delta(rad)", -1.2, 1.2, 0.0),
        "period": p.addUserDebugParameter("sweep period(s)", 0.4, 4.0, 1.2),
        "mode": p.addUserDebugParameter("mode: 0=hold 1=sine", 0, 1, 0),
    }
    return p, sliders


def main():
    parser = argparse.ArgumentParser(description="机械狗单关节方向标定 GUI")
    parser.add_argument("--usb-port", default="/dev/ttyUSB0")
    parser.add_argument("--dt", type=float, default=0.02)
    parser.add_argument("--duration", type=float, default=0.0, help="秒，0=持续运行")
    parser.add_argument(
        "--leg",
        required=True,
        choices=["left_front", "right_front", "left_back", "right_back"],
        help="要标定的腿",
    )
    parser.add_argument(
        "--joint",
        required=True,
        choices=["hip", "knee", "ankle", "all"],
        help="要标定的关节，all=整条腿三个关节同时动",
    )
    args = parser.parse_args()

    from runtime.position_hwi import HWI

    if args.joint == "all":
        active_joints = LEG_JOINTS[args.leg]
    else:
        joint_idx = {"hip": 0, "knee": 1, "ankle": 2}[args.joint]
        active_joints = [LEG_JOINTS[args.leg][joint_idx]]

    hwi = HWI(usb_port=args.usb_port)
    print(f"[INFO] connect: {args.usb_port}")
    print(f"[INFO] active_leg={args.leg}, active_joints={active_joints}")
    print("[INFO] angle map: servo = (mech - real_pose) * real_pose_sign + init_pos + offset")
    for joint_name in active_joints:
        print(f"[INFO] real_pose_sign[{joint_name}]={int(hwi.real_pose_signs.get(joint_name, 1.0)):+d}")

    hwi.turn_on()
    p, sliders = setup_gui()
    print("[INFO] GUI ready, press q or ESC to quit")

    base_pose = hwi.real_pose.copy()
    start_t = time.time()
    next_t = start_t
    tick = 0

    try:
        while True:
            now = time.time()
            if args.duration > 0.0 and now - start_t >= args.duration:
                break

            keys = p.getKeyboardEvents()
            if keys.get(ord("q")) or keys.get(27):
                break

            delta = float(p.readUserDebugParameter(sliders["delta"]))
            period = max(0.4, float(p.readUserDebugParameter(sliders["period"])))
            mode = int(round(p.readUserDebugParameter(sliders["mode"])))

            mech_pose = base_pose.copy()
            if mode == 0:
                delta_now = delta
            else:
                phase = (now - start_t) * (2.0 * math.pi / period)
                delta_now = delta * math.sin(phase)
            for joint_name in active_joints:
                mech_pose[joint_name] = base_pose[joint_name] + delta_now

            send_mech_pose(hwi, mech_pose)

            if tick % 50 == 0:
                for joint_name in active_joints:
                    target = mech_pose[joint_name]
                    sign = hwi.real_pose_signs.get(joint_name, 1.0)
                    servo = mech_to_servo(hwi, joint_name, target)
                    print(
                        f"[DEBUG] {joint_name} mech={target:+.4f} "
                        f"real={hwi.real_pose[joint_name]:+.4f} sign={int(sign):+d} init={hwi.init_pos[joint_name]:+.4f} "
                        f"servo={servo:+.4f} mode={'hold' if mode == 0 else 'sine'}"
                    )
            tick += 1

            next_t += args.dt
            sleep_t = next_t - time.time()
            if sleep_t > 0:
                time.sleep(sleep_t)
            else:
                next_t = time.time()

    except KeyboardInterrupt:
        pass
    finally:
        print("[INFO] stopping...")
        try:
            hwi.set_position_all(hwi.init_pos)
            time.sleep(0.5)
        except Exception:
            pass
        try:
            p.disconnect()
        except Exception:
            pass
        hwi.turn_off()
        print("[INFO] torque disabled")


if __name__ == "__main__":
    main()
