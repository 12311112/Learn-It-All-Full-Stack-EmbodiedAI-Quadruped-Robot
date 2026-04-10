"""
舵机校准工具

流程:
1. 连接舵机, 放松所有扭矩
2. 依次提示用户手动调整每个舵机 (ID 0 ~ 11)
3. 用户按回车确认当前位置
4. 保存所有12个舵机的原始位置值作为零位偏移
"""

import time
from config.robot_config import RobotConfig
from core.hardware_interface import HardwareInterface


def run_calibration():
    """执行舵机校准流程"""
    config = RobotConfig()
    hw = HardwareInterface(config)

    print("=" * 60)
    print("  舵机零位校准工具")
    print("=" * 60)
    print()
    print("本工具将引导你逐个调整12个舵机的零位。")
    print("所有舵机将被放松, 你可以自由转动它们。")
    print("调整好一个舵机后, 按回车键确认并进入下一个。")
    print()

    # 连接
    if not hw.connect():
        print("无法连接舵机, 请检查串口连接。")
        return False

    try:
        # 放松所有舵机
        hw.disable_all_torque()
        print("所有舵机已放松, 可以自由转动。\n")
        time.sleep(0.5)

        offsets = {}

        for servo_id in range(12):
            info = config.get_joint_info(servo_id)
            print(f"--- [{servo_id + 1}/12] {info} ---")
            print("请手动调整该舵机到期望的零位位置。")

            # 等待用户确认
            input("调整好后按 [回车] 确认 >>> ")

            # 读取当前位置
            position = hw.read_position(servo_id)
            offsets[str(servo_id)] = position
            print(f"已保存: 舵机 {servo_id} 位置 = {position}\n")

        # 保存校准结果
        config.save_offsets(offsets)
        print("=" * 60)
        print("  校准完成!")
        print("=" * 60)
        print(f"\n校准数据已保存到: {config.calibration_file}")
        print("\n各舵机零位:")
        for sid in range(12):
            info = config.get_joint_info(sid)
            pos = offsets[str(sid)]
            print(f"  {info}  ->  位置值: {pos}")

        return True

    finally:
        hw.disconnect()


if __name__ == "__main__":
    run_calibration()
