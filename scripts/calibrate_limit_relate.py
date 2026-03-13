#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
逐个标定 12 个舵机的绝对位置区间，并自动回填到 runtime/position_hwi.py 的 self.limit_relate。

流程：
1) 连接硬件并失能全部舵机（可手动掰动）。
2) 按 ID=0~11 逐个标定：实时显示 12 舵机当前绝对位置，且显示当前标定舵机的最小/最大值。
3) 每当按下回车，锁定当前 ID 的 [min, max]，进入下一个 ID。
4) 全部完成后，用绝对区间减去 HWI.init_pos，得到相对区间，自动写入 self.limit_relate。
"""

import argparse
import os
import select
import sys
import time
from typing import Dict, List, Tuple


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from runtime.position_hwi import HWI


def fmt(v):
    if v is None:
        return "   --   "
    return f"{v:8.4f}"


def enter_pressed_nonblock() -> bool:
    if not sys.stdin.isatty():
        return False
    ready, _, _ = select.select([sys.stdin], [], [], 0)
    if ready:
        sys.stdin.readline()
        return True
    return False


def render_limit_relate_block(hwi: HWI, limit_relate: Dict[str, List[float]]) -> List[str]:
    lines = []
    lines.append("        self.limit_relate = {\n")
    for joint_name in hwi.limit_relate.keys():
        lo, hi = limit_relate[joint_name]
        lines.append(f'        "{joint_name}": [{lo:.6f}, {hi:.6f}],\n')
    lines.append("        }\n")
    return lines


def replace_limit_relate_in_file(file_path: str, new_block_lines: List[str]) -> None:
    with open(file_path, "r", encoding="utf-8") as f:
        lines = f.readlines()

    start = None
    for i, line in enumerate(lines):
        if "self.limit_relate = {" in line:
            start = i
            break
    if start is None:
        raise RuntimeError("在 position_hwi.py 中未找到 self.limit_relate = {")

    brace_balance = lines[start].count("{") - lines[start].count("}")
    end = start
    while brace_balance > 0:
        end += 1
        if end >= len(lines):
            raise RuntimeError("解析 self.limit_relate 字典块失败：缺少闭合大括号")
        brace_balance += lines[end].count("{") - lines[end].count("}")

    updated = lines[:start] + new_block_lines + lines[end + 1 :]
    with open(file_path, "w", encoding="utf-8") as f:
        f.writelines(updated)


def main():
    parser = argparse.ArgumentParser(description="标定12个舵机的limit_relate区间")
    parser.add_argument("--usb-port", default="/dev/ttyUSB0", help="串口，如 /dev/ttyUSB0")
    parser.add_argument(
        "--interval",
        type=float,
        default=0.08,
        help="刷新周期（秒），默认 0.08",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="只打印结果，不写回 runtime/position_hwi.py",
    )
    args = parser.parse_args()

    print("=" * 72)
    print("12舵机关节区间标定（绝对值 -> 相对值）")
    print("=" * 72)

    hwi = HWI(usb_port=args.usb_port)
    id_to_name = {sid: name for name, sid in hwi.joints.items()}
    servo_ids = sorted(id_to_name.keys())
    if servo_ids != list(range(12)):
        raise RuntimeError(f"舵机ID不是0~11，当前ID列表：{servo_ids}")

    print("连接成功。准备失能全部舵机...")
    input("按回车后失能全部舵机并开始标定。")
    hwi.turn_off()
    print("已失能全部舵机。现在可以自由掰动。")
    time.sleep(0.3)

    absolute_limits: Dict[int, Tuple[float, float]] = {}

    try:
        for active_id in servo_ids:
            active_name = id_to_name[active_id]
            live_min = None
            live_max = None

            input(
                f"\n准备标定 ID={active_id} ({active_name})。"
                "请掰动该关节到两端后，按回车开始实时采样。"
            )

            while True:
                positions = hwi.io.read_present_position(servo_ids)
                id_to_pos = {sid: float(pos) for sid, pos in zip(servo_ids, positions)}

                cur = id_to_pos[active_id]
                live_min = cur if live_min is None else min(live_min, cur)
                live_max = cur if live_max is None else max(live_max, cur)

                print("\033[2J\033[H", end="")
                print("=" * 100)
                print(
                    f"正在标定 ID={active_id} ({active_name}) | "
                    "实时绝对位置 | 当前舵机按回车锁定[min,max]"
                )
                print("=" * 100)
                print(f"{'ID':>3}  {'joint_name':<24}  {'current':>10}  {'min_seen':>10}  {'max_seen':>10}")
                print("-" * 100)

                for sid in servo_ids:
                    name = id_to_name[sid]
                    current = id_to_pos[sid]

                    if sid in absolute_limits:
                        mn, mx = absolute_limits[sid]
                    elif sid == active_id:
                        mn, mx = live_min, live_max
                    else:
                        mn, mx = None, None

                    print(
                        f"{sid:>3}  {name:<24}  {current:>10.4f}  {fmt(mn):>10}  {fmt(mx):>10}"
                    )

                print("-" * 100)
                print("提示：持续掰动当前关节到两端；达到期望后直接按回车锁定当前ID。")

                if enter_pressed_nonblock():
                    break
                time.sleep(max(args.interval, 0.01))

            absolute_limits[active_id] = (float(live_min), float(live_max))
            print(
                f"\n已锁定 ID={active_id}: "
                f"[{absolute_limits[active_id][0]:.6f}, {absolute_limits[active_id][1]:.6f}]"
            )

    except KeyboardInterrupt:
        print("\n用户中断，退出。")
        return

    abs_by_joint: Dict[str, List[float]] = {}
    relate_by_joint: Dict[str, List[float]] = {}
    for joint_name in hwi.limit_relate.keys():
        sid = hwi.joints[joint_name]
        abs_min, abs_max = absolute_limits[sid]
        init_val = float(hwi.init_pos[joint_name])
        rel_min = abs_min - init_val
        rel_max = abs_max - init_val
        abs_by_joint[joint_name] = [abs_min, abs_max]
        relate_by_joint[joint_name] = [rel_min, rel_max]

    print("\n" + "=" * 72)
    print("绝对区间（按 joint_name）")
    print("=" * 72)
    for joint_name, (lo, hi) in abs_by_joint.items():
        print(f"{joint_name:<28} [{lo:.6f}, {hi:.6f}]")

    print("\n" + "=" * 72)
    print("相对区间（absolute - init_pos，可写入 self.limit_relate）")
    print("=" * 72)
    for joint_name, (lo, hi) in relate_by_joint.items():
        print(f"{joint_name:<28} [{lo:.6f}, {hi:.6f}]")

    target_file = os.path.join(PROJECT_ROOT, "runtime", "position_hwi.py")
    if args.dry_run:
        print("\n[dry-run] 未写文件。")
    else:
        new_block = render_limit_relate_block(hwi, relate_by_joint)
        replace_limit_relate_in_file(target_file, new_block)
        print(f"\n已写入: {target_file}")

    print("\n建议复制保存本次输出，便于追溯。")
    print("标定完成。")


if __name__ == "__main__":
    main()
