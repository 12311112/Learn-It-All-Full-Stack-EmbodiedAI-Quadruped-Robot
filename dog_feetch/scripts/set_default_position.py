#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
机械狗默认位置设置工具
功能：失能所有12个舵机，让用户手动调整姿态，然后读取并保存为默认位置
"""

import os
import sys
import time
import json

# 添加项目根目录到路径
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from runtime.position_hwi import HWI

# ==============================================
# 配置参数
# ==============================================
DEFAULT_POSITION_FILE = "default_position.json"


class DefaultPositionSetter:
    def __init__(self):
        self.hwi = None
        self.default_positions = {}

    def init_hardware(self):
        """初始化硬件接口"""
        print("🔌 正在连接舵机控制器...")
        try:
            self.hwi = HWI()
            print("✅ 硬件连接成功")
            return True
        except Exception as e:
            print(f"❌ 硬件连接失败: {e}")
            print("请检查：")
            print("  1. 机械狗是否已上电")
            print("  2. USB连接是否正常")
            print("  3. 串口设备路径是否正确")
            return False

    def disable_all_servos(self):
        """失能所有舵机，释放扭力"""
        print("\n🔓 正在失能所有舵机...")
        try:
            servo_ids = list(self.hwi.joints.values())
            self.hwi.io.disable_torque(servo_ids)
            print("✅ 所有舵机已失能，现在可以自由掰动关节")
            return True
        except Exception as e:
            print(f"❌ 失能舵机失败: {e}")
            return False

    def read_all_positions(self):
        """读取所有舵机的当前位置（弧度）"""
        print("\n📡 正在读取所有舵机位置...")
        positions = {}

        try:
            # 读取所有舵机位置
            servo_ids = list(self.hwi.joints.values())
            present_positions = self.hwi.io.read_present_position(servo_ids)

            print("\n" + "="*70)
            print(f"{'关节名称':<35} | {'ID':<4} | {'位置 (弧度)':<12} | {'位置 (度)':<10}")
            print("-"*70)

            for joint_name, joint_id in self.hwi.joints.items():
                # 获取对应ID的位置
                idx = servo_ids.index(joint_id)
                position_rad = present_positions[idx]
                position_deg = position_rad * 180.0 / 3.14159265359

                positions[joint_name] = {
                    "position_rad": round(position_rad, 4),
                    "position_deg": round(position_deg, 2),
                    "id": joint_id
                }

                print(f"{joint_name:<35} | {joint_id:<4} | {position_rad:>12.4f} | {position_deg:>10.2f}°")

            print("="*70)
            return positions

        except Exception as e:
            print(f"❌ 读取位置失败: {e}")
            return None

    def save_positions(self, positions):
        """保存位置到JSON文件"""
        try:
            # 保存完整数据
            output_path = os.path.join(PROJECT_ROOT, DEFAULT_POSITION_FILE)
            with open(output_path, 'w', encoding='utf-8') as f:
                json.dump(positions, f, indent=4, ensure_ascii=False)

            print(f"\n✅ 默认位置已保存到: {output_path}")

            # 生成可直接用于代码的字典格式
            print("\n📋 可直接复制到代码中的格式：")
            print("-"*70)
            print("self.init_pos = {")
            for joint_name, data in positions.items():
                print(f'    "{joint_name}": {data["position_rad"]},')
            print("}")
            print("-"*70)

            return True
        except Exception as e:
            print(f"❌ 保存失败: {e}")
            return False

    def enable_all_servos_with_low_torque(self):
        """使能所有舵机并设置低扭矩，锁定当前位置"""
        print("\n🔒 正在使能所有舵机（低扭矩模式）...")
        try:
            servo_ids = list(self.hwi.joints.values())
            # 设置低扭矩
            self.hwi.io.set_kps(servo_ids, self.hwi.low_torque_kps)
            print("✅ 所有舵机已使能并锁定（低扭矩模式）")
            return True
        except Exception as e:
            print(f"❌ 使能舵机失败: {e}")
            return False

    def run(self):
        """主流程"""
        print("="*70)
        print("       🤖 机械狗默认位置设置工具")
        print("="*70)
        print("功能说明：")
        print("1. 程序会失能所有12个舵机，释放扭力")
        print("2. 你可以手动调整机械狗到理想的默认姿态")
        print("3. 调整完成后，程序会读取并保存所有关节的位置")
        print("4. 保存的位置可以作为机械狗的默认站立姿态")
        print("="*70)

        input("\n👉 按回车键开始...")

        # 失能所有舵机
        if not self.disable_all_servos():
            return

        # 等待用户手动调整
        print("\n" + "="*70)
        print("💡 现在所有关节都可以自由掰动了")
        print("💡 请将机械狗调整到你想要的默认姿态")
        print("💡 建议姿态：四条腿自然站立，身体水平")
        print("="*70)
        input("\n👉 调整完成后，按回车键继续...")

        # 读取所有位置
        positions = self.read_all_positions()
        if positions is None:
            print("❌ 无法读取位置，操作终止")
            return

        # 保存到文件
        self.save_positions(positions)

        # 询问是否使能舵机
        print("\n" + "="*70)
        choice = input("是否使能舵机锁定当前位置？(y/n，默认y): ").strip().lower()
        if choice != 'n':
            self.enable_all_servos_with_low_torque()
        else:
            print("⚠️  舵机保持失能状态，请注意机械狗可能会倒下")

        print("\n✅ 操作完成！")
        print(f"💡 默认位置已保存到 {DEFAULT_POSITION_FILE}")
        print("💡 你可以将生成的字典复制到 runtime/position_hwi.py 的 init_pos 中")

    def shutdown(self):
        """关闭硬件连接"""
        print("\n👋 程序已退出")


if __name__ == "__main__":
    setter = DefaultPositionSetter()

    if not setter.init_hardware():
        exit(1)

    try:
        setter.run()
    except KeyboardInterrupt:
        print("\n\n🛑 用户中断操作")
        # 尝试失能所有舵机
        try:
            print("正在失能所有舵机...")
            servo_ids = list(setter.hwi.joints.values())
            setter.hwi.io.disable_torque(servo_ids)
            print("✅ 所有舵机已失能")
        except:
            pass
    finally:
        setter.shutdown()
