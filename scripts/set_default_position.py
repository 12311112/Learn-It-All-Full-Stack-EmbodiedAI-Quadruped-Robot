#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
机械狗默认位置定标工具
流程：
1) 失能所有舵机，手动摆姿态
2) 读取所有关节当前位置
3) 保存到 default_position.json
"""

import json
import os
import sys

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from runtime.position_hwi import HWI

DEFAULT_POSITION_FILE = "default_position.json"


def rad_to_deg(rad):
    return rad * 180.0 / 3.14159265359


def main():
    print("=" * 72)
    print("机械狗默认位置定标工具")
    print("=" * 72)

    print("连接硬件...")
    hwi = HWI(usb_port="/dev/ttyUSB0")
    servo_ids = list(hwi.joints.values())
    print("硬件连接成功。")

    input("\n按回车开始：先失能全部舵机...\n")
    hwi.io.disable_torque(servo_ids)
    print("已失能全部舵机，现在可以手动掰动关节。")

    print("\n请把机器人摆到你想要的默认姿态。")
    input("摆好后按回车读取当前位置...\n")

    present_positions = hwi.io.read_present_position(servo_ids)

    print("\n" + "=" * 72)
    print(f"{'joint_name':<32} {'id':<4} {'position(rad)':>14} {'position(deg)':>14}")
    print("-" * 72)

    result = {}
    init_pos_dict = {}

    for joint_name, joint_id in hwi.joints.items():
        idx = servo_ids.index(joint_id)
        position_rad = float(present_positions[idx])
        position_deg = rad_to_deg(position_rad)

        result[joint_name] = {
            "id": joint_id,
            "position_rad": round(position_rad, 6),
            "position_deg": round(position_deg, 3),
        }
        init_pos_dict[joint_name] = round(position_rad, 6)

        print(
            f"{joint_name:<32} {joint_id:<4} {position_rad:>14.6f} {position_deg:>14.3f}"
        )

    print("=" * 72)

    output_path = os.path.join(PROJECT_ROOT, DEFAULT_POSITION_FILE)
    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(result, f, indent=2, ensure_ascii=False)

    print(f"\n已保存：{output_path}")

    print("\n可直接复制到 runtime/position_hwi.py 的 init_pos:")
    print("init_pos = {")
    for joint_name, value in init_pos_dict.items():
        print(f'    "{joint_name}": {value},')
    print("}")

    choice = input("\n是否以低刚度锁定当前位置？(y/n，默认 y): ").strip().lower()
    if choice != "n":
        hwi.io.set_kps(servo_ids, [2.0] * len(servo_ids))
        print("已设置低刚度。")
    else:
        print("保持失能状态。")

    print("\n定标完成。")



if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n用户中断。")
