#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
将 12 个舵机回到中位（0 rad）的小工具。
默认串口为 /dev/ttyUSB0，可通过参数覆盖。
"""

import argparse
import os
import sys
import time

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from runtime.position_hwi import HWI


def main():
    parser = argparse.ArgumentParser(description="12个舵机回中位")
    parser.add_argument(
        "--usb-port",
        default="/dev/ttyUSB0",
        help="舵机总线串口，例如 /dev/ttyUSB0",
    )
    parser.add_argument(
        "--kp",
        type=float,
        default=8.0,
        help="回中位时的刚度（默认 8.0，较温和）",
    )
    parser.add_argument(
        "--hold-seconds",
        type=float,
        default=2.0,
        help="到位后保持时间（秒）",
    )
    args = parser.parse_args()

    print(f"连接硬件: {args.usb_port}")
    hwi = HWI(usb_port=args.usb_port)
    servo_ids = list(hwi.joints.values())

    print(f"设置刚度 kp={args.kp}")
    hwi.io.set_kps(servo_ids, [float(args.kp)] * len(servo_ids))

    print("发送中位目标（12 个舵机 -> 0 rad）")
    hwi.set_position_all(hwi.zero_pos)

    print(f"保持 {args.hold_seconds:.1f}s ...")
    time.sleep(max(args.hold_seconds, 0.0))
    print("完成。")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n用户中断。")
