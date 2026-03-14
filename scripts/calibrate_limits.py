#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
舵机关节限位标定工具
流程：
1) 失能所有舵机，手动掰动
2) 逐个标定每个舵机的极限位置（实时单行显示）
3) 按回车确认，全部完成后计算相对限位并写回 position_hwi.py
"""

import argparse
import os
import re
import select
import sys
import time

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from runtime.position_hwi import HWI


def main():
    parser = argparse.ArgumentParser(description="舵机关节限位标定工具")
    parser.add_argument("--usb-port", default="/dev/ttyUSB0", help="串口")
    parser.add_argument("--hz", type=float, default=5.0, help="刷新频率")
    args = parser.parse_args()

    hwi = HWI(usb_port=args.usb_port)
    servo_ids = sorted(hwi.joints.values())
    id_to_joint = {v: k for k, v in hwi.joints.items()}

    hwi.io.disable_torque(servo_ids)
    print("已失能全部舵机，可以自由掰动。")
    print("=" * 60)
    print("逐个标定：掰动舵机到极限位置，按回车确认。")
    print("=" * 60)

    period = 1.0 / args.hz
    confirmed_min = {}
    confirmed_max = {}

    try:
        for sid in servo_ids:
            jname = id_to_joint[sid]

            print(f"\n>> ID={sid} ({jname}) - 掰动后按回车确认")

            # 读取初始位置
            prev_raw = None
            while prev_raw is None:
                try:
                    positions = hwi.io.read_present_position([sid])
                    prev_raw = float(positions[0])
                except Exception:
                    time.sleep(period)

            cumul = 0.0  # 累积相对位移
            mn = 0.0
            mx = 0.0

            while True:
                try:
                    positions = hwi.io.read_present_position([sid])
                    raw = float(positions[0])
                except Exception as e:
                    sys.stdout.write(f"\r  [读取失败: {e}]")
                    sys.stdout.flush()
                    time.sleep(period)
                    continue

                # 增量展开：对 delta 做 unwrap，避免 ±π 跳变
                delta = raw - prev_raw
                if delta > 3.14159:
                    delta -= 2 * 3.14159265
                elif delta < -3.14159:
                    delta += 2 * 3.14159265
                cumul += delta
                prev_raw = raw

                if cumul < mn:
                    mn = cumul
                if cumul > mx:
                    mx = cumul

                sys.stdout.write(
                    f"\r  相对={cumul:>8.4f}"
                    f"  最小={mn:>8.4f}"
                    f"  最大={mx:>8.4f}   "
                )
                sys.stdout.flush()

                # 非阻塞检测回车
                if select.select([sys.stdin], [], [], 0)[0]:
                    sys.stdin.readline()
                    break

                time.sleep(period)

            confirmed_min[sid] = round(mn, 6)
            confirmed_max[sid] = round(mx, 6)
            print(
                f"\r  最小={mn:>8.4f}  最大={mx:>8.4f}  ✓"
            )

    except KeyboardInterrupt:
        print("\n用户中断。")
        return

    # --- 计算相对限位并写回 position_hwi.py ---
    print("\n" + "=" * 60)
    print("标定结果")
    print("=" * 60)

    relative_limits = {}
    for sid in servo_ids:
        jname = id_to_joint[sid]
        rel_min = confirmed_min[sid]
        rel_max = confirmed_max[sid]
        if rel_min > rel_max:
            rel_min, rel_max = rel_max, rel_min
        relative_limits[jname] = [rel_min, rel_max]
        print(
            f"ID={sid:2d} {jname:<28}"
            f" rel=[{rel_min:.4f}, {rel_max:.4f}]"
        )

    hwi_path = os.path.join(PROJECT_ROOT, "runtime", "position_hwi.py")
    with open(hwi_path, "r", encoding="utf-8") as f:
        content = f.read()

    pattern = r"(self\.limit_relate\s*=\s*\{)[^}]*(})"

    def build_replacement(match):
        lines = [match.group(1)]
        for jname in hwi.joints.keys():
            lim = relative_limits[jname]
            lines.append(f'        "{jname}": [{lim[0]}, {lim[1]}],')
        lines.append("        " + match.group(2))
        return "\n".join(lines)

    new_content = re.sub(pattern, build_replacement, content, count=1)
    with open(hwi_path, "w", encoding="utf-8") as f:
        f.write(new_content)

    print(f"\n已将相对限位写入: {hwi_path}")
    print("标定完成。")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n用户中断。")
