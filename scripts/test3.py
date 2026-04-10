#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import os
import sys
import time
from typing import Dict, List

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from runtime.position_hwi import HWI


def probe_id(hwi: HWI, servo_id: int):
    """Return (ok, position_or_err)."""
    try:
        pos = hwi.io.read_present_position([servo_id])[0]
        return True, float(pos)
    except Exception as e:
        return False, str(e)


def scan_online_ids(hwi: HWI, scan_min: int, scan_max: int) -> List[int]:
    online = []
    for servo_id in range(scan_min, scan_max + 1):
        ok, _ = probe_id(hwi, servo_id)
        if ok:
            online.append(servo_id)
    return online


def nudge_joint(hwi: HWI, joint_name: str, servo_id: int, delta: float, hold: float):
    ok, val = probe_id(hwi, servo_id)
    if not ok:
        print(f"ID {servo_id:02d} ({joint_name}) 离线，跳过。")
        return

    current = float(val)
    try:
        hwi.set_kp(servo_id, 6.0)
    except Exception as e:
        print(f"设置低 Kp 失败（继续）: {e}")

    try:
        hwi.io.write_goal_position([servo_id], [current + delta])
        time.sleep(hold)
        hwi.io.write_goal_position([servo_id], [current])
        time.sleep(hold)
        hwi.io.write_goal_position([servo_id], [current - delta])
        time.sleep(hold)
        hwi.io.write_goal_position([servo_id], [current])
        time.sleep(hold)
        print(
            f"已点动 {joint_name} (ID {servo_id:02d})，中心 {current:+.3f} rad，幅度 ±{delta:.3f} rad"
        )
    except Exception as e:
        print(f"点动失败 {joint_name} (ID {servo_id:02d}): {e}")


def main():
    parser = argparse.ArgumentParser(
        description="检查舵机 ID 映射，并支持逐个关节点动确认"
    )
    parser.add_argument(
        "--usb-port",
        default="/dev/ttyUSB0",
        help="舵机总线串口，例如 /dev/ttyUSB0",
    )
    parser.add_argument("--scan-min", type=int, default=0, help="扫描起始 ID")
    parser.add_argument("--scan-max", type=int, default=20, help="扫描结束 ID")
    parser.add_argument(
        "--delta",
        type=float,
        default=0.20,
        help="每次点动幅度（rad），默认 0.20",
    )
    parser.add_argument(
        "--hold",
        type=float,
        default=0.35,
        help="每个动作保持时长（秒），默认 0.35",
    )
    args = parser.parse_args()

    if args.scan_min < 0 or args.scan_max < args.scan_min:
        raise ValueError("请保证 0 <= scan-min <= scan-max")

    hwi = HWI(usb_port=args.usb_port)
    configured: Dict[str, int] = hwi.joints

    print(f"已连接: {args.usb_port}")
    print("\n[1] 配置中的关节 -> ID")
    print("-" * 72)
    for idx, (joint_name, servo_id) in enumerate(configured.items()):
        print(f"{idx:02d} | ID {servo_id:02d} | {joint_name}")

    print("\n[2] 检查配置 ID 是否在线")
    print("-" * 72)
    configured_ids = set(configured.values())
    ok_count = 0
    fail_count = 0
    for idx, (joint_name, servo_id) in enumerate(configured.items()):
        ok, val = probe_id(hwi, servo_id)
        if ok:
            ok_count += 1
            print(
                f"{idx:02d} | ID {servo_id:02d} | {joint_name:<24} | ONLINE | pos={val:+.3f} rad"
            )
        else:
            fail_count += 1
            print(
                f"{idx:02d} | ID {servo_id:02d} | {joint_name:<24} | MISSING | {val}"
            )

    print("\n[3] 扫描总线，找“在线但未配置”的 ID")
    print("-" * 72)
    online_ids = scan_online_ids(hwi, args.scan_min, args.scan_max)
    unknown_ids = [sid for sid in online_ids if sid not in configured_ids]

    print(f"扫描范围: [{args.scan_min}, {args.scan_max}]")
    print(f"在线 ID: {online_ids if online_ids else '无'}")
    print(f"配置外在线 ID: {unknown_ids if unknown_ids else '无'}")

    print("\n总结")
    print("-" * 72)
    print(f"配置总数: {len(configured)}")
    print(f"配置内在线: {ok_count}")
    print(f"配置内离线: {fail_count}")
    if fail_count == 0 and len(unknown_ids) == 0:
        print("ID 映射看起来是干净的。")
    else:
        print("存在异常：请重点检查离线关节和配置外在线 ID。")

    print("\n[4] 逐个关节点动确认（按回车测试当前关节）")
    print("-" * 72)
    print("操作说明：")
    print("1) 直接按回车：执行当前关节点动")
    print("2) 输入 s 回车：跳过当前关节")
    print("3) 输入 q 回车：退出点动模式")

    for idx, (joint_name, servo_id) in enumerate(configured.items()):
        cmd = input(
            f"\n[{idx+1:02d}/{len(configured)}] {joint_name} (ID {servo_id:02d}) "
            "按回车开始点动: "
        ).strip().lower()
        if cmd == "q":
            print("收到退出指令，结束点动模式。")
            break
        if cmd == "s":
            print("已跳过。")
            continue
        nudge_joint(hwi, joint_name, servo_id, delta=args.delta, hold=args.hold)


if __name__ == "__main__":
    main()
