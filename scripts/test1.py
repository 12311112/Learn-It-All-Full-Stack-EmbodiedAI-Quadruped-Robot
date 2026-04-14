#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import os
import sys
import time

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from runtime.position_hwi import HWI


def main():
    parser = argparse.ArgumentParser(description="实时显示12个舵机当前位置")
    parser.add_argument(
        "--usb-port",
        default="/dev/ttyUSB0",
        help="舵机总线串口，例如 /dev/ttyUSB0",
    )
    parser.add_argument(
        "--hz",
        type=float,
        default=10.0,
        help="刷新频率（Hz），默认 10",
    )
    args = parser.parse_args()
    if args.hz <= 0:
        raise ValueError("--hz 必须大于 0")

    hwi = HWI(usb_port=args.usb_port)
    period = 1.0 / args.hz
    print(f"已连接: {args.usb_port}")
    print(f"刷新频率: {args.hz:.2f} Hz")
    print("按 Ctrl+C 退出。")

    try:
        while True:
            positions = hwi.get_present_positions()
            if positions is None:
                print("读取当前位置失败，正在重试...")
                time.sleep(period)
                continue

            print("\033[2J\033[H", end="")
            print("12个舵机当前位置 (rad)")
            print("-" * 40)
            for idx, (joint_name, joint_id) in enumerate(hwi.joints.items()):
                print(
                    f"{idx:02d} | ID {joint_id:02d} | {joint_name:<24} | {positions[idx]:>7.3f}"
                )
            time.sleep(period)
    except KeyboardInterrupt:
        print("\n已退出实时检查。")


if __name__ == "__main__":
    main()
