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
    parser = argparse.ArgumentParser(description="执行一次舵机 turn_on 流程")
    parser.add_argument(
        "--usb-port",
        default="/dev/ttyUSB0",
        help="舵机总线串口，例如 /dev/ttyUSB0",
    )
    args = parser.parse_args()

    hwi = HWI(usb_port=args.usb_port)
    print(f"已连接: {args.usb_port}")
    print("开始执行 turn_on 流程...")
    hwi.turn_on()
    print("turn_on 完成，待机 2 秒...")
    time.sleep(2)
   # hwi.turn_off()
    print("已失能。")



if __name__ == "__main__":
    main()
