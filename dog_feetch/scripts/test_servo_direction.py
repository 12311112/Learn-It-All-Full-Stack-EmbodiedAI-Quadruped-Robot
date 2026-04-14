#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
舵机正方向逐一测试工具
======================
功能：
  1. 先将所有舵机移动到 init_pos（初始站立姿态）并保持。
  2. 按舵机 ID 顺序（0 → 11）依次测试每个舵机：
     - 在 init_pos 基础上向 **正方向** 转动 3°（≈0.05236 rad），然后立刻回到原位。
  3. 每测试完一个舵机后，用户可以选择：
     - 按 Enter / 输入 'r'：再次测试当前舵机（重复转动 +3° 并回位）
     - 输入 'n'：跳到下一个舵机

用法：
  python test_servo_direction.py [--usb-port /dev/ttyUSB0] [--angle 3] [--kp 15]
"""

import argparse
import math
import os
import sys
import time

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from runtime.position_hwi import HWI


def deg2rad(deg):
    return deg * math.pi / 180.0


def main():
    parser = argparse.ArgumentParser(description="舵机正方向逐一测试工具")
    parser.add_argument(
        "--usb-port",
        default="/dev/ttyUSB0",
        help="舵机总线串口，例如 /dev/ttyUSB0（默认）",
    )
    parser.add_argument(
        "--angle",
        type=float,
        default=5.0,
        help="测试转动角度（度），默认 3°",
    )
    args = parser.parse_args()

    delta_rad = deg2rad(args.angle)

    # ========== 1. 连接硬件 ==========
    print(f"[INFO] 连接硬件: {args.usb_port}")
    hwi = HWI(usb_port=args.usb_port)

    # 建立 ID → 关节名 的映射表，用于按 ID 顺序遍历
    id_to_name = {v: k for k, v in hwi.joints.items()}
    sorted_ids = sorted(id_to_name.keys())  # [0, 1, 2, ..., 11]

    # ========== 2. 按 turn_on() 流程移动到 init_pos ==========
    #  与原代码一致: low_torque_kps(2) → init_pos → kps(30) + kds(0)
    print("[INFO] 启动舵机（按 turn_on 流程）...")
    hwi.turn_on()
    print("[INFO] 已到达 init_pos，准备开始测试。\n")

    # ========== 3. 逐一测试 ==========
    print("=" * 60)
    print(f"  舵机正方向测试 —— 每次在 init_pos 基础上 +{args.angle}°")
    print(f"  共 {len(sorted_ids)} 个舵机，按 ID 0→11 顺序")
    print("=" * 60)
    print("  操作说明：")
    print("    Enter 或 'r' = 再次测试当前舵机")
    print("    'n'          = 下一个舵机")
    print("    'q'          = 退出测试")
    print("=" * 60 + "\n")

    for servo_id in sorted_ids:
        joint_name = id_to_name[servo_id]
        init_value = hwi.init_pos[joint_name]

        print(f">>> [{servo_id:2d}] {joint_name}")
        print(f"    init_pos = {init_value:.6f} rad ({math.degrees(init_value):.2f}°)")
        print(f"    目标位置 = {init_value + delta_rad:.6f} rad ({math.degrees(init_value + delta_rad):.2f}°)")

        # 第一次自动执行
        _do_test(hwi, joint_name, servo_id, init_value, delta_rad)

        # 等待用户选择
        while True:
            user_input = input(f"    [{servo_id:2d}] 再次测试(Enter/r) | 下一个(n) | 退出(q): ").strip().lower()
            if user_input in ("", "r"):
                _do_test(hwi, joint_name, servo_id, init_value, delta_rad)
            elif user_input == "n":
                print()
                break
            elif user_input == "q":
                print("\n[INFO] 用户退出测试。")
                _safe_shutdown(hwi)
                return
            else:
                print("    无效输入，请输入 r / n / q")

    print("\n[INFO] 全部 12 个舵机测试完毕！")
    _safe_shutdown(hwi)


def _do_test(hwi, joint_name, servo_id, init_value, delta_rad):
    """执行一次测试：从 init_pos 向正方向转 delta_rad，然后立刻回位。"""
    target_value = init_value + delta_rad

    # 转到 +delta 位置
    print(f"    -> 正方向转动 +{math.degrees(delta_rad):.1f}° ...", end="", flush=True)
    hwi.set_position(joint_name, target_value)
    time.sleep(0.6)

    # 回到 init_pos
    print(" 回位 ...", end="", flush=True)
    hwi.set_position(joint_name, init_value)
    time.sleep(0.4)
    print(" OK")


def _safe_shutdown(hwi):
    """安全关闭：回到 init_pos 后释放力矩。"""
    print("[INFO] 回到 init_pos ...")
    hwi.set_position_all(hwi.init_pos)
    time.sleep(1)
    print("[INFO] 释放力矩，测试结束。")
    hwi.turn_off()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[INFO] 用户中断 (Ctrl+C)。")
