#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
机械狗默认位置定标工具
流程：
1) 逐条腿失能舵机，手动摆姿态
2) 逐条腿读取当前位置
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
LEG_GROUPS = {
    "lb": "left_back",
    "rb": "right_back",
    "lf": "left_front",
    "rf": "right_front",
}


def rad_to_deg(rad):
    return rad * 180.0 / 3.14159265359


def get_leg_joints(hwi, leg_prefix):
    return [
        joint_name
        for joint_name in hwi.joints.keys()
        if joint_name.startswith(leg_prefix)
    ]


def calibrate_one_leg(hwi, leg_key, leg_prefix, result, init_pos_dict):
    leg_joint_names = get_leg_joints(hwi, leg_prefix)
    leg_servo_ids = [hwi.joints[joint_name] for joint_name in leg_joint_names]

    print("\n" + "=" * 72)
    print(f"开始定标腿: {leg_key.upper()} ({leg_prefix})")
    print("=" * 72)

    input("按回车失能该腿舵机...\n")
    hwi.io.disable_torque(leg_servo_ids)
    print(f"已失能 {leg_key.upper()}，请只调整这条腿。")
    input("摆好后按回车读取该腿当前位置...\n")

    present_positions = hwi.io.read_present_position(leg_servo_ids)

    print(f"\n{leg_key.upper()} 读取结果:")
    print(f"{'joint_name':<32} {'id':<4} {'position(rad)':>14} {'position(deg)':>14}")
    print("-" * 72)

    for idx, joint_name in enumerate(leg_joint_names):
        joint_id = hwi.joints[joint_name]
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

    print(f"{leg_key.upper()} 定标完成。")


def main():
    print("=" * 72)
    print("机械狗默认位置定标工具")
    print("=" * 72)

    print("连接硬件...")
    hwi = HWI(usb_port="/dev/ttyUSB0")
    servo_ids = list(hwi.joints.values())
    print("硬件连接成功。")
    hwi.turn_on()
    result = {}
    init_pos_dict = {}

    print("\n将按腿依次定标：LB -> RB -> LF -> RF")
    for leg_key, leg_prefix in LEG_GROUPS.items():
        calibrate_one_leg(hwi, leg_key, leg_prefix, result, init_pos_dict)

    print("\n" + "=" * 72)
    print("全部腿定标完成，总览：")
    print(f"{'joint_name':<32} {'id':<4} {'position(rad)':>14} {'position(deg)':>14}")
    print("-" * 72)
    for joint_name, info in result.items():
        print(
            f"{joint_name:<32} {info['id']:<4} "
            f"{info['position_rad']:>14.6f} {info['position_deg']:>14.3f}"
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
