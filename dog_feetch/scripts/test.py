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
    parser = argparse.ArgumentParser(description="一次性初始化 12 个舵机")
    parser.add_argument(
        "--usb-port",
        default="/dev/ttyUSB0",
        help="舵机总线串口，例如 /dev/ttyUSB0",
    )
    parser.add_argument("--kp", type=float, default=30.0, help="初始化刚度 Kp")
    parser.add_argument("--kd", type=float, default=0.0, help="初始化阻尼 Kd")
    parser.add_argument(
        "--hold-seconds",
        type=float,
        default=2.0,
        help="到达 init_pos 后保持时长（秒）",
    )
    parser.add_argument(
        "--keep-torque",
        action="store_true",
        help="初始化后保持使能，不自动断扭矩",
    )
    args = parser.parse_args()

    hwi = HWI(usb_port=args.usb_port)
    n = len(hwi.joints)
    print(f"已连接: {args.usb_port}")
    print(f"检测到舵机数量: {n}")
    if n != 12:
        print("警告: 当前关节数量不是 12，请确认配置。")

    # 一次性对全部舵机设置控制参数并执行上电到 init_pos
    hwi.set_kps([float(args.kp)] * n)
    hwi.set_kds([float(args.kd)] * n)
    print(f"设置完成: kp={args.kp}, kd={args.kd}")

    print("开始初始化全部舵机（low_kp -> init_pos -> target_kp）...")
    hwi.turn_on()
    print(f"初始化完成，保持 {args.hold_seconds:.1f} 秒...")
    time.sleep(max(0.0, args.hold_seconds))

    if args.keep_torque:
        print("已完成初始化（保持使能）。")
    else:
        hwi.turn_off()
        print("已完成初始化并断扭矩。")



if __name__ == "__main__":
    main()
