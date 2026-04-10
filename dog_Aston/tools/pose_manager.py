"""
姿态管理工具

功能:
1. 执行姿态 — 切换到指定姿态并锁定舵机
2. 回归默认 — 所有舵机回到校准零位并锁定
3. 放松舵机 — 失能所有舵机扭矩
4. 新建/编辑/删除自定义姿态

姿态数据格式: {servo_id_str: 角度偏移(弧度)}, 相对于校准零位
存储在 config/poses.json
"""

import math
import time
from config.robot_config import RobotConfig
from core.hardware_interface import HardwareInterface


def _apply_pose(config: RobotConfig, hw: HardwareInterface, pose_data: dict,
                duration: float = 1.0):
    """
    平滑过渡到目标姿态并锁定舵机。

    参数:
        pose_data: {servo_id_str: 角度偏移(弧度)}
        duration: 过渡时间(秒)
    """
    target_positions = config.pose_to_raw_positions(pose_data)

    # 读取当前位置
    current_positions = {}
    for sid in range(12):
        current_positions[sid] = hw.read_position(sid)

    # 使能扭矩
    hw.enable_all_torque()

    # 平滑插值过渡
    steps = max(int(duration * config.control_frequency), 1)
    for step in range(1, steps + 1):
        alpha = step / steps
        interp = {}
        for sid in range(12):
            cur = current_positions[sid]
            tgt = target_positions.get(sid, cur)
            interp[sid] = int(cur + (tgt - cur) * alpha)
        hw.write_all_positions(interp)
        time.sleep(config.control_dt)

    print("舵机已锁定在目标姿态。")


def _print_pose_detail(config: RobotConfig, pose_data: dict):
    """打印姿态详情"""
    for leg, joint_names in config.legs.items():
        leg_cn = config.leg_names_cn[leg]
        print(f"  {leg_cn} ({leg}):")
        for jname in joint_names:
            sid = config.joints[jname]
            angle_rad = pose_data.get(str(sid), 0.0)
            angle_deg = math.degrees(angle_rad)
            jtype = jname.split("_")[1]
            jtype_cn = config.joint_type_cn[jtype]
            print(f"    {jtype_cn} (ID {sid:>2}): {angle_deg:>+7.1f}°")


def _input_pose_data(config: RobotConfig, base_data: dict) -> dict:
    """交互式输入姿态数据, 返回新的姿态字典"""
    pose = dict(base_data)

    print("请输入各关节角度 (度数, 相对于校准零位)。")
    print("直接按回车保持当前值。\n")
    print("  关节限位:  hip ±45°  |  thigh ±80°  |  calf 0°~70°\n")

    for leg, joint_names in config.legs.items():
        leg_cn = config.leg_names_cn[leg]
        print(f"--- {leg_cn} ({leg}) ---")

        for jname in joint_names:
            sid = config.joints[jname]
            lower_rad, upper_rad = config._urdf_joint_ranges[jname]
            cur_rad = pose.get(str(sid), 0.0)
            cur_deg = math.degrees(cur_rad)
            jtype = jname.split("_")[1]
            jtype_cn = config.joint_type_cn[jtype]
            lo_deg = math.degrees(lower_rad)
            hi_deg = math.degrees(upper_rad)

            prompt = (f"  {jtype_cn} (ID {sid}) "
                      f"[{lo_deg:.0f}°~{hi_deg:.0f}°] "
                      f"当前 {cur_deg:+.1f}°: ")
            val = input(prompt).strip()
            if val:
                try:
                    pose[str(sid)] = math.radians(float(val))
                except ValueError:
                    print("    无效输入, 保持原值。")
        print()

    return pose


def _menu_execute(config: RobotConfig, hw: HardwareInterface):
    """执行姿态子菜单"""
    names = list(config.poses.keys())
    if not names:
        print("没有可用的姿态。")
        return

    print("\n可用姿态:")
    for i, name in enumerate(names, 1):
        print(f"  {i}. {name}")

    choice = input("\n选择姿态编号 (回车取消): ").strip()
    if not choice:
        return
    try:
        idx = int(choice) - 1
        if not (0 <= idx < len(names)):
            print("无效编号。")
            return
    except ValueError:
        print("无效输入。")
        return

    name = names[idx]
    pose_data = config.poses[name]

    # 验证
    valid, err = config.validate_pose(pose_data)
    if not valid:
        print(f"错误: {err}")
        return

    dur = input("过渡时间 (秒, 默认1.0): ").strip()
    dur = float(dur) if dur else 1.0

    print(f"\n正在执行姿态 '{name}'...")
    _apply_pose(config, hw, pose_data, dur)


def _menu_create(config: RobotConfig):
    """新建/编辑姿态子菜单"""
    name = input("\n姿态名称: ").strip()
    if not name:
        print("名称不能为空。")
        return

    if name in config.poses:
        print(f"姿态 '{name}' 已存在, 将进行编辑。")
        base = dict(config.poses[name])
    else:
        base = {str(i): 0.0 for i in range(12)}

    print()
    pose_data = _input_pose_data(config, base)

    # 验证
    valid, err = config.validate_pose(pose_data)
    if not valid:
        print(f"\n错误: {err}")
        print("姿态未保存。")
        return

    config.poses[name] = pose_data
    config.save_poses()
    print(f"姿态 '{name}' 已保存。")


def _menu_delete(config: RobotConfig):
    """删除姿态子菜单"""
    names = list(config.poses.keys())
    if not names:
        print("没有可用的姿态。")
        return

    print("\n可用姿态:")
    for i, name in enumerate(names, 1):
        print(f"  {i}. {name}")

    choice = input("\n选择要删除的编号 (回车取消): ").strip()
    if not choice:
        return
    try:
        idx = int(choice) - 1
        if not (0 <= idx < len(names)):
            print("无效编号。")
            return
    except ValueError:
        print("无效输入。")
        return

    name = names[idx]
    if name == "默认":
        print("不能删除默认姿态。")
        return

    confirm = input(f"确认删除 '{name}'? (y/n): ").strip().lower()
    if confirm == "y":
        del config.poses[name]
        config.save_poses()
        print(f"姿态 '{name}' 已删除。")


def _menu_list(config: RobotConfig):
    """查看所有姿态"""
    if not config.poses:
        print("没有可用的姿态。")
        return

    for name, pose_data in config.poses.items():
        print(f"\n[ {name} ]")
        _print_pose_detail(config, pose_data)


def run_pose_manager():
    """姿态管理主入口"""
    config = RobotConfig()

    if not config.is_calibrated():
        print("错误: 尚未完成舵机校准, 请先运行校准工具。")
        return

    hw = HardwareInterface(config)
    if not hw.connect():
        print("无法连接舵机, 请检查串口连接。")
        return

    try:
        while True:
            print("\n" + "=" * 50)
            print("  姿态管理")
            print("=" * 50)
            print()
            print("  1. 执行姿态 (切换并锁定)")
            print("  2. 回归默认姿态 (锁定)")
            print("  3. 放松所有舵机")
            print("  4. 新建/编辑姿态")
            print("  5. 删除姿态")
            print("  6. 查看所有姿态")
            print("  0. 返回主菜单")
            print()

            choice = input("请选择: ").strip()

            if choice == "1":
                _menu_execute(config, hw)
            elif choice == "2":
                default_pose = config.poses.get("默认",
                    {str(i): 0.0 for i in range(12)})
                print("\n正在回归默认姿态...")
                _apply_pose(config, hw, default_pose, 1.0)
            elif choice == "3":
                hw.disable_all_torque()
                print("所有舵机已放松。")
            elif choice == "4":
                _menu_create(config)
            elif choice == "5":
                _menu_delete(config)
            elif choice == "6":
                _menu_list(config)
            elif choice == "0":
                break
            else:
                print("无效选择。")
    finally:
        hw.disconnect()


if __name__ == "__main__":
    run_pose_manager()
