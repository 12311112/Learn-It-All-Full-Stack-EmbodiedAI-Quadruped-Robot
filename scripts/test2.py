#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import math
import os
import sys
import time

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from runtime.position_hwi import HWI


JOINT_NAME = "left_back_ankle_joint"
ANGLE_SEQUENCE = [3, 3.5, -3.5, -3.5,3.5]


def monitor_joint(hwi, seconds, hz, target_rad):
    period = 1.0 / max(hz, 0.1)
    joint_idx = list(hwi.joints.keys()).index(JOINT_NAME)
    end_t = time.time() + max(seconds, 0.0)

    while time.time() < end_t:
        pos = hwi.get_present_positions()
        if pos is None:
            print("读取失败，重试中...")
            time.sleep(period)
            continue

        cur = float(pos[joint_idx])
        err = cur - target_rad
        print(
            f"目标 {target_rad:+.3f} rad ({math.degrees(target_rad):+.1f} deg) | "
            f"当前 {cur:+.3f} rad ({math.degrees(cur):+.1f} deg) | "
            f"误差 {err:+.3f} rad"
        )
        time.sleep(period)


def main():
    parser = argparse.ArgumentParser(
        description="测试 left_back_ankle_joint 依次下发 3.00, 3.10, -3.10, -3.00"
    )
    parser.add_argument("--usb-port", default="/dev/ttyUSB0", help="舵机总线串口")
    parser.add_argument("--hold-seconds", type=float, default=1.5, help="每个目标保持秒数")
    parser.add_argument("--hz", type=float, default=10.0, help="打印频率(Hz)")
    parser.add_argument("--kp", type=float, default=8.0, help="测试时该关节 Kp")
    args = parser.parse_args()

    hwi = HWI(usb_port=args.usb_port)
    servo_id = hwi.joints[JOINT_NAME]

    print(f"已连接: {args.usb_port}")
    print(f"测试关节: {JOINT_NAME} (ID={servo_id})")
    print(f"角度序列: {ANGLE_SEQUENCE}")
    print("按 Ctrl+C 可随时停止。")

    try:
        hwi.set_kp(servo_id, float(args.kp))
    except Exception as e:
        print(f"设置 Kp 失败（继续）: {e}")

    try:
        for i, target in enumerate(ANGLE_SEQUENCE, start=1):
            print(f"\n[{i}/{len(ANGLE_SEQUENCE)}] 下发目标: {target:+.3f} rad")
            hwi.set_position(JOINT_NAME, target)
            monitor_joint(hwi, seconds=args.hold_seconds, hz=args.hz, target_rad=target)

        print("\n序列下发完成。")
    except KeyboardInterrupt:
        print("\n用户中断，停止测试。")


if __name__ == "__main__":
    main()
